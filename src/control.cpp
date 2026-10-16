#include "rebot_rt/control.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/time_types.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

namespace rebot_rt {
namespace {
constexpr auto kPeriod = std::chrono::milliseconds(10);
constexpr auto kFeedbackLimit = std::chrono::milliseconds(100);
constexpr auto kCanWriteTimeout = std::chrono::milliseconds(20);
constexpr uint32_t kRsOperationControl = 1;
constexpr uint32_t kRsOperationStatus = 2;
constexpr uint32_t kRsEnable = 3;
constexpr uint32_t kRsDisable = 4;
constexpr uint32_t kRsReadParameter = 17;
constexpr uint32_t kRsWriteParameter = 18;
constexpr uint32_t kRsFaultReport = 21;
constexpr uint32_t kRsActiveReport = 24;
constexpr uint16_t kRsHostId = 0x00FD;

std::string rs_status_description(uint8_t flags) {
  const std::array<const char*, 6> names{
      "undervoltage", "three-phase overcurrent", "overtemperature",
      "magnetic encoder fault", "stall overload", "encoder uncalibrated"};
  std::string result;
  for (int bit = 0; bit < 6; ++bit) {
    if (!(flags & (1u << bit))) continue;
    if (!result.empty()) result += ", ";
    result += names[bit];
  }
  return result;
}

uint32_t rs_ext_id(uint32_t communication_type, uint16_t extra_data, uint8_t node_id) {
  return (communication_type << 24) | (static_cast<uint32_t>(extra_data) << 8) | node_id;
}

uint32_t float_bits(double value) {
  const float f = static_cast<float>(value);
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof(bits));
  return bits;
}

void put_u32(std::array<uint8_t, 8>& data, int offset, uint32_t value) {
  for (int j = 0; j < 4; ++j) data[offset + j] = static_cast<uint8_t>(value >> (8 * j));
}

uint32_t get_u32(const std::array<uint8_t, 8>& data, int offset) {
  uint32_t value = 0;
  for (int j = 0; j < 4; ++j) value |= static_cast<uint32_t>(data[offset + j]) << (8 * j);
  return value;
}

speed_t serial_speed(uint32_t baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported DM serial baud (use 115200/230400/460800/921600)");
  }
}

uint16_t encode(double value, double lo, double hi, int bits) {
  const auto clamped = std::clamp(value, lo, hi);
  return static_cast<uint16_t>((clamped - lo) / (hi - lo) * ((1u << bits) - 1u));
}
double decode(uint16_t value, double lo, double hi, int bits) {
  return lo + static_cast<double>(value) / ((1u << bits) - 1u) * (hi - lo);
}
std::array<uint8_t, 8> mit_frame(double q, double dq, double kp, double kd, double tau,
                                 double pmax, double vmax, double tmax,
                                 double kpmax = 500.0, double kdmax = 5.0) {
  const auto p = encode(q, -pmax, pmax, 16);
  const auto v = encode(dq, -vmax, vmax, 12);
  const auto k = encode(kp, 0.0, kpmax, 12);
  const auto d = encode(kd, 0.0, kdmax, 12);
  const auto t = encode(tau, -tmax, tmax, 12);
  return {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
          static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (k >> 8)),
          static_cast<uint8_t>(k), static_cast<uint8_t>(d >> 4),
          static_cast<uint8_t>((d << 4) | (t >> 8)), static_cast<uint8_t>(t)};
}

std::array<uint8_t, 8> rs_mit_payload(double q, double dq, double kp, double kd,
                                      double pmax, double vmax, double kpmax, double kdmax) {
  const auto p = encode(q, -pmax, pmax, 16);
  const auto v = encode(dq, -vmax, vmax, 16);
  const auto k = encode(kp, 0.0, kpmax, 16);
  const auto d = encode(kd, 0.0, kdmax, 16);
  return {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
          static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v),
          static_cast<uint8_t>(k >> 8), static_cast<uint8_t>(k),
          static_cast<uint8_t>(d >> 8), static_cast<uint8_t>(d)};
}
bool valid_rotation(const Eigen::Matrix3d& r) {
  return r.allFinite() && std::abs(r.determinant() - 1.0) < 1e-3 &&
         (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-3;
}
} // namespace

void MotorConfig::set_model(Family family, const std::string& new_model) {
  struct Spec { const char* name; double p; double v; double t; };
  static const std::array<Spec, 16> dm{{
      {"3507", 12.566, 50.0, 5.0}, {"4310", 12.5, 30.0, 10.0},
      {"4310P", 12.5, 50.0, 10.0}, {"4340", 12.5, 10.0, 28.0},
      {"4340P", 12.5, 10.0, 28.0}, {"4340_v20", 12.5, 20.0, 28.0},
      {"6006", 12.5, 45.0, 20.0}, {"8006", 12.5, 45.0, 40.0},
      {"8009", 12.5, 45.0, 54.0}, {"10010L", 12.5, 25.0, 200.0},
      {"10010", 12.5, 20.0, 200.0}, {"H3510", 12.5, 280.0, 1.0},
      {"G6215", 12.5, 45.0, 10.0}, {"H6220", 12.5, 45.0, 10.0},
      {"JH11", 12.5, 10.0, 12.0}, {"6248P", 12.566, 20.0, 120.0},
  }};
  static const std::array<Spec, 7> rs{{
      {"rs-00", 4.0 * M_PI, 33.0, 14.0}, {"rs-01", 4.0 * M_PI, 44.0, 17.0},
      {"rs-02", 4.0 * M_PI, 44.0, 17.0}, {"rs-03", 4.0 * M_PI, 20.0, 60.0},
      {"rs-04", 4.0 * M_PI, 15.0, 120.0}, {"rs-05", 4.0 * M_PI, 50.0, 5.5},
      {"rs-06", 4.0 * M_PI, 50.0, 36.0},
  }};
  const auto match = [&](const auto& specs) {
    for (const auto& spec : specs) {
      if (new_model == spec.name) {
        model = new_model;
        pmax = spec.p; vmax = spec.v; tmax = spec.t;
        return true;
      }
    }
    return false;
  };
  if (!(family == Family::DM ? match(dm) : match(rs)))
    throw std::invalid_argument("unknown motor model: " + new_model);
  kpmax = family == Family::RS &&
          (new_model == "rs-03" || new_model == "rs-04" || new_model == "rs-06") ? 5000.0 : 500.0;
  kdmax = kpmax == 5000.0 ? 100.0 : 5.0;
}

std::array<MotorConfig, kMotorCount> default_motor_configs(Family family) {
  std::array<MotorConfig, kMotorCount> out{};
  for (int i = 0; i < kMotorCount; ++i) {
    auto& m = out[i];
    m.send_id = i + 1;
    m.feedback_id = family == Family::DM ? 0x11 + i : kRsHostId;
    m.direction = 1.0; // bundled URDF uses the motor's raw, zeroed joint frame
    if (family == Family::DM) {
      m.set_model(family, i < 3 ? "4340P" : "4310");
      m.kp = i < 3 ? 45.0 : (i == 4 ? 9.0 : 8.0);
      m.kd = i < 3 ? 5.0 : (i == 6 ? 0.3 : 1.0);
      m.vel_kp = i == 0 ? 0.0125 : (i < 3 ? 0.013 : 0.0008);
      m.vel_ki = i < 3 ? 0.004 : 0.002;
      m.pos_kp = i == 0 ? 150.0 : (i < 3 ? 200.0 : 50.0);
      m.pos_ki = i == 0 ? 0.5 : (i < 3 ? 10.0 : 1.0);
    } else {
      m.set_model(family, i < 3 ? "rs-06" : "rs-00");
      m.kp = i == 0 ? 50.0 : (i < 3 ? 150.0 : (i == 6 ? 12.0 : 50.0));
      m.kd = i == 0 ? 3.0 : (i < 3 ? 10.0 : (i == 6 ? 0.05 : (i == 4 || i == 5 ? 4.0 : 5.0)));
      m.vel_kp = i == 0 ? 12.0 : (i < 3 ? 14.0 : (i == 3 ? 5.0 : 4.0));
      m.vel_ki = 0.1;
      m.pos_kp = i == 0 ? 13.0 : (i == 1 ? 16.0 : (i == 2 ? 14.0 : (i == 3 ? 20.0 : 10.0)));
    }
  }
  return out;
}

Robot::Robot(Family family, std::string channel, std::string urdf_path,
             std::string end_frame, bool has_gripper, double max_joint_speed,
             double command_timeout_sec, uint32_t dm_serial_baud,
             std::vector<MotorConfig> motor_configs, std::string control_mode, bool mit_velocity_feedforward,
             std::vector<double> max_joint_speeds, bool coordinate_cartesian_speed)
    : family_(family), control_mode_(std::move(control_mode)), mit_velocity_feedforward_(mit_velocity_feedforward),
      channel_(std::move(channel)), end_frame_(std::move(end_frame)),
      has_gripper_(has_gripper), max_joint_speed_(max_joint_speed),
      coordinate_cartesian_speed_(coordinate_cartesian_speed),
      command_timeout_sec_(command_timeout_sec), dm_serial_baud_(dm_serial_baud), model_(urdf_path),
      frame_id_(end_frame_.empty() ? model_.end_effector_frame_id() : model_.frame_id(end_frame_)) {
  if (channel_.empty()) throw std::invalid_argument("channel must not be empty");
  if (control_mode_ != "mit" && control_mode_ != "pos_vel")
    throw std::invalid_argument("control_mode must be 'mit' or 'pos_vel'");
  if (family_ == Family::RS && channel_.front() == '/')
    throw std::invalid_argument("RS requires a SocketCAN interface (for example can0)");
  if (family_ == Family::DM && dm_serial_baud_ == 0)
    throw std::invalid_argument("dm_serial_baud must be positive");
  if (model_.nq() < kArmDof || model_.nv() < kArmDof)
    throw std::invalid_argument("URDF must contain six actuated arm joints");
  if (!std::isfinite(max_joint_speed_) || max_joint_speed_ <= 0 ||
      !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0)
    throw std::invalid_argument("speed and timeout must be finite and positive");
  if (!motor_configs.empty() && motor_configs.size() != kMotorCount)
    throw std::invalid_argument("motor_configs must contain exactly seven motors");
  if (!max_joint_speeds.empty() && max_joint_speeds.size() != kArmDof)
    throw std::invalid_argument("max_joint_speeds must contain exactly six finite positive speeds");
  joint_speed_limits_.fill(max_joint_speed_);
  for (int i = 0; i < kArmDof; ++i) {
    if (!max_joint_speeds.empty()) joint_speed_limits_[i] = max_joint_speeds[i];
    if (!std::isfinite(joint_speed_limits_[i]) || joint_speed_limits_[i] <= 0)
      throw std::invalid_argument("max_joint_speeds must contain exactly six finite positive speeds");
  }
  target_speed_limits_ = joint_speed_limits_;
  const auto defaults = default_motor_configs(family_);
  const auto limits = model_.joint_limits();
  for (int i = 0; i < kMotorCount; ++i) {
    auto& m = motors_[i];
    const auto& config = motor_configs.empty() ? defaults[i] : motor_configs[i];
    if (config.send_id == 0 || config.send_id > (family_ == Family::DM ? 0x7FFu : 0xFFu) ||
        config.feedback_id > (family_ == Family::DM ? 0x7FFu : 0xFFu) ||
        !std::isfinite(config.pmax) || config.pmax <= 0 ||
        !std::isfinite(config.vmax) || config.vmax <= 0 ||
        !std::isfinite(config.tmax) || config.tmax <= 0 ||
        !std::isfinite(config.kpmax) || config.kpmax <= 0 ||
        !std::isfinite(config.kdmax) || config.kdmax <= 0 ||
        !std::isfinite(config.kp) || config.kp < 0 || config.kp > config.kpmax ||
        !std::isfinite(config.kd) || config.kd < 0 || config.kd > config.kdmax ||
        !std::isfinite(config.vel_kp) || config.vel_kp < 0 ||
        !std::isfinite(config.vel_ki) || config.vel_ki < 0 ||
        !std::isfinite(config.pos_kp) || config.pos_kp < 0 ||
        !std::isfinite(config.pos_ki) || config.pos_ki < 0 ||
        (config.direction != -1.0 && config.direction != 1.0))
      throw std::invalid_argument("invalid motor config at joint " + std::to_string(i + 1));
    for (int j = 0; j < i; ++j) {
      const auto& prev = motor_configs.empty() ? defaults[j] : motor_configs[j];
      if (config.send_id == prev.send_id ||
          (family_ == Family::DM && config.feedback_id == prev.feedback_id))
        throw std::invalid_argument("duplicate motor or feedback ID");
    }
    m.send_id = config.send_id; m.feedback_id = config.feedback_id;
    if (i < kArmDof && joint_speed_limits_[i] > config.vmax)
      throw std::invalid_argument("joint speed limit exceeds motor velocity encoding range at joint " +
                                  std::to_string(i + 1));
    m.pmax = config.pmax; m.vmax = config.vmax; m.tmax = config.tmax;
    m.kp = config.kp; m.kd = config.kd;
    m.vel_kp = config.vel_kp; m.vel_ki = config.vel_ki;
    m.pos_kp = config.pos_kp; m.pos_ki = config.pos_ki;
    m.kpmax = config.kpmax; m.kdmax = config.kdmax;
    m.direction = config.direction;
    if (i < kArmDof) {
      m.lower = std::max(limits.at(i).first, -m.pmax);
      m.upper = std::min(limits.at(i).second, m.pmax);
    } else {
      m.lower = -4.71238898038469;
      m.upper = 0.0;
    }
  }
  last_command_ = Clock::now();
}

Robot::~Robot() { close(); }

void Robot::connect() {
  if (connected()) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.fault = false;
    state_.fault_reason.clear();
    state_.feedback_count.fill(0);
    state_.can_rx_dropped = 0;
  }
  serial_ = family_ == Family::DM && channel_.front() == '/';
  if (serial_) {
    const auto speed = serial_speed(dm_serial_baud_);
    fd_ = ::open(channel_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "open " + channel_);
    termios tty{};
    if (::tcgetattr(fd_, &tty) < 0) {
      const int e = errno; ::close(fd_); fd_ = -1;
      throw std::system_error(e, std::generic_category(), "tcgetattr " + channel_);
    }
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (::tcsetattr(fd_, TCSANOW, &tty) < 0) {
      const int e = errno; ::close(fd_); fd_ = -1;
      throw std::system_error(e, std::generic_category(), "tcsetattr " + channel_);
    }
    serial_rx_.clear();
  } else {
    const int sock = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (sock < 0) throw std::system_error(errno, std::generic_category(), "socket(PF_CAN)");
    const int enabled = 1;
    if (::setsockopt(sock, SOL_SOCKET, SO_TIMESTAMPNS_NEW, &enabled, sizeof(enabled)) < 0 ||
        ::setsockopt(sock, SOL_SOCKET, SO_RXQ_OVFL, &enabled, sizeof(enabled)) < 0) {
      const int e = errno; ::close(sock);
      throw std::system_error(e, std::generic_category(), "CAN receive timestamp/drop reporting");
    }
    const unsigned int index = ::if_nametoindex(channel_.c_str());
    if (index == 0) { ::close(sock); throw std::runtime_error("unknown CAN interface: " + channel_); }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = static_cast<int>(index);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const int e = errno;
      ::close(sock);
      throw std::system_error(e, std::generic_category(), "bind " + channel_);
    }
    fd_ = sock;
  }
  try {
    if (family_ == Family::RS) recover_rs_startup_faults();
    const auto request_start = Clock::now();
    for (int i = 0; i < kMotorCount; ++i) if (i < kArmDof || has_gripper_) request_feedback(i);
    const auto deadline = request_start + std::chrono::milliseconds(250);
    while (Clock::now() < deadline) {
      read_feedback_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(5)));
      bool complete = true;
      for (int i = 0; i < kMotorCount; ++i) {
        if (i == kArmDof && !has_gripper_) continue;
        complete &= motors_[i].last_feedback >= request_start;
      }
      if (complete) break;
    }
    const auto now = Clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (int i = 0; i < kMotorCount; ++i) {
        if (i == kArmDof && !has_gripper_) continue;
        if (motors_[i].last_feedback == Clock::time_point{} ||
            now - motors_[i].last_feedback > kFeedbackLimit)
          throw std::runtime_error("missing initial feedback from motor " + std::to_string(i + 1));
      }
      if (state_.fault) throw std::runtime_error(state_.fault_reason);
    }
    {
      // Configure once while disabled. Never switch run_mode in the servo loop.
      for (int i = 0; i < kMotorCount; ++i)
        if (i < kArmDof || has_gripper_) send_special(i, 0xFD);
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      for (int i = 0; i < kMotorCount; ++i)
        if (i < kArmDof || has_gripper_) configure_motor(i);
      const auto refresh_start = Clock::now();
      for (int i = 0; i < kMotorCount; ++i)
        if (i < kArmDof || has_gripper_) request_feedback(i);
      const auto refresh_deadline = refresh_start + std::chrono::milliseconds(250);
      while (Clock::now() < refresh_deadline) {
        read_feedback_until(std::min(refresh_deadline, Clock::now() + std::chrono::milliseconds(5)));
        bool complete = true;
        for (int i = 0; i < kMotorCount; ++i)
          if (i < kArmDof || has_gripper_)
            complete &= motors_[i].last_feedback >= refresh_start;
        if (complete) break;
      }
    }
    std::lock_guard<std::mutex> state_lock(mutex_);
    for (int i = 0; i < kMotorCount; ++i) {
      if (i == kArmDof && !has_gripper_) continue;
      if (Clock::now() - motors_[i].last_feedback > kFeedbackLimit)
        throw std::runtime_error("stale initial feedback from motor " + std::to_string(i + 1));
    }
    if (state_.fault) throw std::runtime_error(state_.fault_reason);
    target_ = state_.q;
    sent_ = target_;
    cancel_cartesian_locked();
    target_speed_limits_ = joint_speed_limits_;
    state_.command_velocity.fill(0.0);
    state_.mit_command_velocity.fill(0.0);
    last_command_ = Clock::now();
  } catch (...) {
    disable_all();
    ::close(fd_);
    fd_ = -1;
    serial_ = false;
    throw;
  }
}

void Robot::start() {
  if (running()) return;
  if (worker_.joinable()) worker_.join();
  if (!connected()) connect();
  // The CSP speed limit is constant for this controller. Set and verify it
  // once per start, instead of sending six redundant parameter writes per tick.
  if (family_ == Family::RS) {
    for (int i = 0; i < kArmDof; ++i) {
      if (!position_mode(i)) continue;
      const auto speed = float_bits(joint_speed_limits_[i]);
      write_setting(i, 0x7017, speed);
      if (read_setting(i, 0x7017) != speed)
        throw std::runtime_error("RS motor " + std::to_string(i + 1) + " speed limit readback mismatch");
    }
  }
  const auto refresh_start = Clock::now();
  for (int i = 0; i < kMotorCount; ++i)
    if (i < kArmDof || has_gripper_) request_feedback(i);
  const auto deadline = refresh_start + std::chrono::milliseconds(250);
  while (Clock::now() < deadline) {
    read_feedback_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(5)));
    bool complete = true;
    for (int i = 0; i < kMotorCount; ++i)
      if (i < kArmDof || has_gripper_)
        complete &= motors_[i].last_feedback >= refresh_start;
    if (complete) break;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.fault) throw std::runtime_error(state_.fault_reason);
    for (int i = 0; i < kMotorCount; ++i) {
      if (i == kArmDof && !has_gripper_) continue;
      if (Clock::now() - motors_[i].last_feedback > kFeedbackLimit)
        throw std::runtime_error("stale feedback from motor " + std::to_string(i + 1));
    }
    target_ = state_.q;
    sent_ = target_;
    cancel_cartesian_locked();
    target_speed_limits_ = joint_speed_limits_;
    last_command_ = Clock::now();
    state_.command_velocity.fill(0.0);
    state_.mit_command_velocity.fill(0.0);
  }
  // Establish a measured hold target before enabling position controllers.
  for (int i = 0; i < kArmDof; ++i)
    if (position_mode(i)) send_position(i, sent_[i]);
  enable_all();
  running_ = true;
  worker_ = std::thread([this] { run(); });
}

void Robot::stop() {
  running_ = false;
  if (worker_.joinable()) worker_.join();
  disable_all();
}

void Robot::close() {
  stop();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

State Robot::get_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  State copy = state_;
  copy.target = target_;
  copy.command = sent_;
  copy.joint_speed_limits = target_speed_limits_;
  const auto now = Clock::now();
  copy.running = running();
  copy.age_sec = 0.0;
  for (int i = 0; i < kMotorCount; ++i) {
    if (i == kArmDof && !has_gripper_) { copy.fresh[i] = false; continue; }
    const auto age = std::chrono::duration<double>(now - motors_[i].last_feedback).count();
    copy.age_sec = std::max(copy.age_sec, age);
    copy.fresh[i] = age <= std::chrono::duration<double>(kFeedbackLimit).count();
    copy.feedback_age_sec[i] = age;
  }
  return copy;
}

JointArray Robot::get_target_joints() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return target_;
}

void Robot::set_target_joints(const JointArray& q, std::optional<double> max_joint_speed) {
  const double largest_limit = *std::max_element(joint_speed_limits_.begin(), joint_speed_limits_.begin() + kArmDof);
  const double speed = max_joint_speed.value_or(largest_limit);
  if (!std::isfinite(speed) || speed <= 0 || speed > largest_limit)
    throw std::invalid_argument("target max_joint_speed must be finite, positive and at most the robot limit");
  for (int i = 0; i < kMotorCount; ++i) {
    if (!std::isfinite(q[i])) throw std::invalid_argument("non-finite joint target");
    if ((i < kArmDof || has_gripper_) && (q[i] < motors_[i].lower || q[i] > motors_[i].upper))
      throw std::invalid_argument("joint " + std::to_string(i + 1) + " exceeds model limits");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.fault) throw std::runtime_error("control fault: " + state_.fault_reason);
  cancel_cartesian_locked();
  target_ = q;
  target_speed_limits_ = joint_speed_limits_;
  for (int i = 0; i < kArmDof; ++i)
    target_speed_limits_[i] = std::min(joint_speed_limits_[i], speed);
  last_command_ = Clock::now();
}

void Robot::hold_position() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.fault) throw std::runtime_error("control fault: " + state_.fault_reason);
  cancel_cartesian_locked();
  target_ = sent_;
  last_command_ = Clock::now();
}

Eigen::VectorXd Robot::model_q(const JointArray& q) const {
  Eigen::VectorXd out = model_.neutral();
  for (int i = 0; i < kArmDof; ++i) out[i] = q[i];
  return out;
}

IKStatus Robot::set_target_pose(const Eigen::Matrix4d& pose, double gripper) {
  if (!pose.allFinite() || !valid_rotation(pose.block<3, 3>(0, 0)) ||
      (pose.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6)
    throw std::invalid_argument("target must be a finite rigid 4x4 transform");
  JointArray seed = get_target_joints();
  IKStatus status;
  rebotarm::IKParams params;
  params.max_iter = 150;
  params.tolerance = 1e-4;
  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    const auto result = model_.solve_ik(pose, model_q(seed), frame_id_, params);
    status.success = result.success;
    status.error = result.error;
    status.iterations = result.iterations;
    if (result.success) {
      for (int i = 0; i < kArmDof; ++i) status.target[i] = result.q[i];
      status.target[kArmDof] = has_gripper_ ? gripper : seed[kArmDof];
    }
  }
  if (status.success) set_target_joints(status.target);
  return status;
}

void Robot::cancel_cartesian_locked() {
  cartesian_enabled_ = false;
  cartesian_endpoint_ = false;
  cartesian_last_update_ = {};
  state_.cartesian_active = false;
  state_.cartesian_ik_valid = false;
  state_.cartesian_tracking_limited = false;
  state_.cartesian_tracking_progress = false;
  state_.cartesian_coordinated = false;
  state_.cartesian_step_scale = 0.0;
  state_.cartesian_limiting_joint = -1;
  // Invalidate an IK solve already in flight before a reset/hold/new session.
  ++state_.cartesian_sequence;
}

uint64_t Robot::queue_target_pose(const Eigen::Matrix4d& pose, double gripper) {
  if (!pose.allFinite() || !valid_rotation(pose.block<3, 3>(0, 0)) ||
      (pose.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6)
    throw std::invalid_argument("target must be a finite rigid 4x4 transform");
  if (!std::isfinite(gripper) || (has_gripper_ &&
      (gripper < motors_[kArmDof].lower || gripper > motors_[kArmDof].upper)))
    throw std::invalid_argument("invalid gripper target");
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.fault) throw std::runtime_error("control fault: " + state_.fault_reason);
  const auto now = Clock::now();
  const double interval = cartesian_last_update_ == Clock::time_point{}
      ? 0.01 : std::chrono::duration<double>(now - cartesian_last_update_).count();
  // A producer restart must not inherit a multi-second interpolation delay.
  state_.cartesian_duration_sec = interval > command_timeout_sec_ ? 0.01 : std::max(0.01, interval);
  cartesian_goal_ = pose;
  cartesian_gripper_ = gripper;
  cartesian_last_update_ = now;
  cartesian_enabled_ = true;
  cartesian_endpoint_ = false;
  state_.cartesian_active = true;
  last_command_ = now;  // Only producer calls refresh the watchdog, never IK substeps.
  return ++state_.cartesian_sequence;
}

void Robot::update_cartesian_target() {
  JointArray seed;
  Eigen::Matrix4d start, goal;
  Clock::time_point started;
  double duration, gripper;
  uint64_t sequence;
  bool new_segment;
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cartesian_enabled_ || !state_.cartesian_active || state_.fault) return;
    if (std::chrono::duration<double>(now - last_command_).count() > command_timeout_sec_) return;
    seed = sent_;  // Warm-start from the command actually reached by the limiter.
    sequence = state_.cartesian_sequence;
    new_segment = sequence != state_.cartesian_consumed_sequence;
    start = cartesian_start_;
    goal = cartesian_goal_;
    started = new_segment ? now : cartesian_started_;
    duration = state_.cartesian_duration_sec;
    gripper = cartesian_gripper_;
  }
  const double alpha = std::clamp(
      (std::chrono::duration<double>(now - started).count() + 0.01) / duration, 0.0, 1.0);
  rebotarm::IKResult result;
  {
    // Never hold the command mutex during FK/IK: A/reset/hold can invalidate
    // this generation while it is being solved, and get_state stays responsive.
    std::lock_guard<std::mutex> model_lock(model_mutex_);
    if (new_segment) start = std::get<2>(model_.fk(model_q(seed), end_frame_));
    Eigen::Matrix4d sample = Eigen::Matrix4d::Identity();
    sample.block<3, 1>(0, 3) = start.block<3, 1>(0, 3) +
        alpha * (goal.block<3, 1>(0, 3) - start.block<3, 1>(0, 3));
    Eigen::Quaterniond from(start.block<3, 3>(0, 0)), to(goal.block<3, 3>(0, 0));
    from.normalize();
    to.normalize();
    if (from.dot(to) < 0.0) to.coeffs() *= -1.0;
    sample.block<3, 3>(0, 0) = from.slerp(alpha, to).normalized().toRotationMatrix();
    rebotarm::IKParams params;
    params.max_iter = 40;
    params.tolerance = 1e-4;
    params.damping = 1e-4;
    params.max_time_sec = 0.003;  // Reserve the rest of the 10 ms cycle for CAN and feedback.
    const Eigen::VectorXd q_seed = model_q(seed);
    Eigen::VectorXd lower = q_seed, upper = q_seed;
    for (int i = 0; i < kArmDof; ++i) {
      const double step = joint_speed_limits_[i] * 0.01;
      lower[i] = std::max(motors_[i].lower, seed[i] - step);
      upper[i] = std::min(motors_[i].upper, seed[i] + step);
    }
    result = coordinate_cartesian_speed_
        ? model_.solve_coordinated_tracking_ik(sample, q_seed, frame_id_, lower, upper, params)
        : model_.solve_tracking_ik(sample, q_seed, frame_id_, lower, upper, params);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cartesian_enabled_ || state_.cartesian_sequence != sequence || state_.fault ||
      std::chrono::duration<double>(Clock::now() - last_command_).count() > command_timeout_sec_) return;
  state_.cartesian_consumed_sequence = sequence;
  state_.cartesian_ik_valid = true;
  state_.cartesian_ik_success = result.success;
  state_.cartesian_tracking_limited = !result.success;
  state_.cartesian_tracking_progress = result.progress;
  state_.cartesian_coordinated = result.coordinated;
  state_.cartesian_step_scale = result.step_scale;
  state_.cartesian_limiting_joint = result.limiting_joint;
  state_.cartesian_ik_error = result.error;
  ++state_.cartesian_ik_solves;
  cartesian_start_ = start;
  cartesian_started_ = started;
  if (has_gripper_) target_[kArmDof] = gripper;
  if (!result.success && !result.progress) {
    // No feasible descent this tick: hold the arm, but keep the segment alive
    // until its endpoint. A later sample may move inward or along a boundary.
    for (int i = 0; i < kArmDof; ++i) target_[i] = sent_[i];
    cartesian_endpoint_ = alpha >= 1.0;
    return;
  }
  if (!result.q.allFinite()) {
    target_ = sent_;
    throw std::runtime_error("Cartesian tracking IK produced a non-finite joint target");
  }
  for (int i = 0; i < kArmDof; ++i) {
    if (!std::isfinite(result.q[i]) || result.q[i] < motors_[i].lower || result.q[i] > motors_[i].upper)
      throw std::runtime_error("Cartesian IK produced an invalid joint target");
    target_[i] = result.q[i];
  }
  target_speed_limits_ = joint_speed_limits_;
  // A partial local solution is a step toward the endpoint, not completion.
  // Continue with the same waypoint until convergence, no progress, or timeout.
  cartesian_endpoint_ = alpha >= 1.0 && result.success;
}

Eigen::Matrix4d Robot::forward_kinematics() const { return forward_kinematics(get_state().q); }

Eigen::Matrix4d Robot::forward_kinematics(const JointArray& q) const {
  std::lock_guard<std::mutex> lock(model_mutex_);
  return std::get<2>(model_.fk(model_q(q), end_frame_));
}

void Robot::send_frame(uint32_t id, const std::array<uint8_t, 8>& data, bool extended) {
  if (fd_ < 0) throw std::runtime_error("transport is not connected");
  if (serial_ && extended) throw std::runtime_error("DM serial bridge only supports standard CAN frames");
  if (serial_) {
    // Damiao's USB serial bridge wraps one standard CAN frame in a 30-byte
    // packet. The bridge returns 16-byte packets beginning with AA 11 ... 55.
    std::array<uint8_t, 30> raw{};
    raw[0] = 0x55; raw[1] = 0xAA; raw[2] = 0x1E; raw[3] = 0x03;
    raw[4] = 1; // sendTimes, little endian
    raw[8] = 10; // timeInterval, little endian, in milliseconds
    raw[12] = 0; // standard ID
    std::memcpy(raw.data() + 13, &id, sizeof(uint32_t));
    raw[17] = 0; // data frame
    raw[18] = 8; // DLC
    std::memcpy(raw.data() + 21, data.data(), data.size());
    std::size_t written = 0;
    while (written < raw.size()) {
      const auto n = ::write(fd_, raw.data() + written, raw.size() - written);
      if (n > 0) { written += static_cast<std::size_t>(n); continue; }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        pollfd pfd{fd_, POLLOUT, 0};
        if (::poll(&pfd, 1, 20) < 0 && errno != EINTR)
          throw std::system_error(errno, std::generic_category(), "poll serial write");
        continue;
      }
      throw std::system_error(errno, std::generic_category(), "serial write");
    }
    return;
  }
  can_frame frame{};
  frame.can_id = extended ? (id | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
  frame.can_dlc = 8;
  std::memcpy(frame.data, data.data(), 8);
  const auto deadline = Clock::now() + kCanWriteTimeout;
  while (true) {
    const auto written = ::write(fd_, &frame, sizeof(frame));
    if (written == sizeof(frame)) return;
    const int error = written < 0 ? errno : EIO;
    if (error != ENOBUFS && error != EAGAIN && error != EWOULDBLOCK && error != EINTR)
      throw std::system_error(error, std::generic_category(), "CAN write on " + channel_);
    if (Clock::now() >= deadline)
      throw std::system_error(error, std::generic_category(),
                              "CAN transmit queue on " + channel_ + " did not drain within 20 ms");
    // Socket POLLOUT can remain ready when the CAN device's small TX queue is
    // full. Back off instead of spinning or dropping this frame. A permanently
    // blocked bus still fails within the deadline and follows normal teardown.
    std::this_thread::sleep_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(1)));
  }
}

void Robot::send_special(int index, uint8_t command) {
  if (family_ == Family::RS) {
    std::array<uint8_t, 8> data{};
    if (command == 0xFB) {
      data[0] = 1; // RobStride clear-fault is DISABLE with byte 0 set.
      send_rs_command(index, kRsDisable, static_cast<uint16_t>(motors_[index].feedback_id), data);
    } else if (command == 0xFC) {
      send_rs_command(index, kRsEnable, static_cast<uint16_t>(motors_[index].feedback_id), data);
    } else if (command == 0xFD) {
      send_rs_command(index, kRsDisable, static_cast<uint16_t>(motors_[index].feedback_id), data);
    } else {
      throw std::invalid_argument("unsupported RS special command");
    }
    return;
  }
  std::array<uint8_t, 8> data{};
  data.fill(0xFF);
  data[7] = command;
  send_frame(motors_[index].send_id, data);
}

void Robot::send_rs_command(int index, uint32_t communication_type, uint16_t extra_data,
                            const std::array<uint8_t, 8>& data) {
  send_frame(rs_ext_id(communication_type, extra_data,
                       static_cast<uint8_t>(motors_[index].send_id)), data, true);
}

void Robot::send_rs_mit(int index, const std::array<uint8_t, 8>& data, uint16_t torque) {
  send_frame(rs_ext_id(kRsOperationControl, torque,
                       static_cast<uint8_t>(motors_[index].send_id)), data, true);
}

void Robot::request_feedback(int index) {
  if (family_ == Family::DM) {
    send_frame(0x7FF, {static_cast<uint8_t>(motors_[index].send_id), 0, 0xCC, 0, 0, 0, 0, 0});
  } else {
    std::array<uint8_t, 8> report{1, 2, 3, 4, 5, 6, 1, 0};
    send_rs_command(index, kRsActiveReport, static_cast<uint16_t>(motors_[index].feedback_id), report);
    send_special(index, 0xFD); // status query while disabled; do not clear fault latches
  }
}

void Robot::recover_rs_startup_faults() {
  // Only connect() may recover faults. First disable the whole arm and drain
  // pre-clear feedback, which can still carry the previous session's fault.
  for (int i = 0; i < kMotorCount; ++i)
    if (i < kArmDof || has_gripper_) send_special(i, 0xFD);
  for (int i = 0; i < kMotorCount; ++i)
    if (i < kArmDof || has_gripper_) request_feedback(i);
  read_feedback_until(Clock::now() + std::chrono::milliseconds(60));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.fault = false;
    state_.fault_reason.clear();
    for (auto& motor : motors_) {
      motor.last_feedback = {};
      motor.rs_status_flags = 0x3F;
      motor.rs_run_state = 0xFF;
    }
  }
  rs_startup_recovery_ = true;
  try {
    // One clear attempt, never an enable or a mechanical-zero command.
    for (int i = 0; i < kMotorCount; ++i)
      if (i < kArmDof || has_gripper_) send_special(i, 0xFB);
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    Clock::time_point healthy_since{};
    while (Clock::now() < deadline) {
      read_feedback_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(5)));
      const auto now = Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      bool healthy = !state_.fault; // A new type-21 report remains latched.
      for (int i = 0; i < kMotorCount; ++i) {
        if (i == kArmDof && !has_gripper_) continue;
        const auto& motor = motors_[i];
        healthy &= now - motor.last_feedback <= kFeedbackLimit &&
                   motor.rs_status_flags == 0 && motor.rs_run_state == 0;
      }
      if (!healthy) healthy_since = {};
      else if (healthy_since == Clock::time_point{}) healthy_since = now;
      else if (now - std::max(healthy_since, rs_unhealthy_at_) >= std::chrono::milliseconds(100)) {
        rs_startup_recovery_ = false;
        return;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.fault) {
      state_.fault = true;
      state_.fault_reason = "RS startup did not receive stable, fault-free disabled feedback";
      for (int i = 0; i < kMotorCount; ++i) {
        if (i == kArmDof && !has_gripper_) continue;
        const auto& motor = motors_[i];
        const auto prefix = "RS motor " + std::to_string(i + 1);
        if (Clock::now() - motor.last_feedback > kFeedbackLimit) {
          state_.fault_reason = prefix + " missing fresh feedback after fault clear";
          break;
        }
        if (motor.rs_status_flags != 0) {
          state_.fault_reason = prefix + " reported status flags " + std::to_string(motor.rs_status_flags) +
                                " (" + rs_status_description(motor.rs_status_flags) + ")";
          break;
        }
        if (motor.rs_run_state != 0) {
          state_.fault_reason = prefix + " did not confirm disabled state after fault clear";
          break;
        }
      }
    }
    throw std::runtime_error(state_.fault_reason + "; startup fault clear failed; motors remain disabled");
  } catch (...) {
    rs_startup_recovery_ = false;
    throw;
  }
}

uint32_t Robot::read_setting(int index, uint16_t setting) {
  awaited_motor_ = index;
  awaited_setting_ = setting;
  setting_received_ = false;
  const auto& m = motors_[index];
  if (family_ == Family::RS) {
    send_rs_command(index, kRsReadParameter, static_cast<uint16_t>(m.feedback_id),
                    {static_cast<uint8_t>(setting), static_cast<uint8_t>(setting >> 8), 0, 0, 0, 0, 0, 0});
  } else {
    send_frame(0x7FF, {static_cast<uint8_t>(m.send_id), static_cast<uint8_t>(m.send_id >> 8),
                       0x33, static_cast<uint8_t>(setting), 0, 0, 0, 0});
  }
  const auto deadline = Clock::now() + std::chrono::milliseconds(150);
  while (!setting_received_ && Clock::now() < deadline)
    read_feedback_until(std::min(deadline, Clock::now() + std::chrono::milliseconds(5)));
  awaited_motor_ = -1;
  if (!setting_received_)
    throw std::runtime_error("motor " + std::to_string(index + 1) +
                             " did not return setting " + std::to_string(setting));
  return setting_value_;
}

void Robot::write_setting(int index, uint16_t setting, uint32_t value) {
  const auto& m = motors_[index];
  std::array<uint8_t, 8> data{};
  if (family_ == Family::RS) {
    data[0] = static_cast<uint8_t>(setting);
    data[1] = static_cast<uint8_t>(setting >> 8);
    put_u32(data, 4, value);
    send_rs_command(index, kRsWriteParameter, static_cast<uint16_t>(m.feedback_id), data);
  } else {
    data[0] = static_cast<uint8_t>(m.send_id);
    data[1] = static_cast<uint8_t>(m.send_id >> 8);
    data[2] = 0x55;
    data[3] = static_cast<uint8_t>(setting);
    put_u32(data, 4, value);
    send_frame(0x7FF, data);
  }
}

void Robot::configure_motor(int index) {
  // RS streaming uses CSP (5): limit_spd belongs to CSP, whereas PP (1)
  // uses vel_max/acc_set and replans each goal. DM uses POS_VEL (2).
  const uint16_t mode_setting = family_ == Family::RS ? 0x7005 : 10;
  const uint32_t desired = position_mode(index) ? (family_ == Family::RS ? 5 : 2)
                                               : (family_ == Family::RS ? 0 : 1);
  bool confirmed = false;
  for (int attempt = 0; attempt < 3 && !confirmed; ++attempt) {
    try { confirmed = read_setting(index, mode_setting) == desired; }
    catch (const std::runtime_error&) { if (attempt == 2) throw; }
    if (confirmed) break;
    write_setting(index, mode_setting, desired);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    confirmed = read_setting(index, mode_setting) == desired;
  }
  if (!confirmed) throw std::runtime_error("motor " + std::to_string(index + 1) + " control mode readback mismatch");
  if (!position_mode(index)) return;
  const auto& m = motors_[index];
  const std::array<uint16_t, 4> settings = family_ == Family::RS
      ? std::array<uint16_t, 4>{0x701F, 0x7020, 0x701E, 0}
      : std::array<uint16_t, 4>{25, 26, 27, 28};
  const std::array<double, 4> gains{m.vel_kp, m.vel_ki, m.pos_kp, m.pos_ki};
  for (int j = 0; j < 4; ++j) {
    if (settings[j] == 0 || gains[j] <= 0) continue;
    const auto value = float_bits(gains[j]);
    write_setting(index, settings[j], value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (read_setting(index, settings[j]) != value)
      throw std::runtime_error("motor " + std::to_string(index + 1) + " position/velocity gain readback mismatch");
  }
}

void Robot::send_position(int index, double q) {
  const auto& m = motors_[index];
  const auto position = float_bits(q * m.direction);
  if (family_ == Family::RS) {
    write_setting(index, 0x7016, position);
  } else {
    std::array<uint8_t, 8> data{};
    put_u32(data, 0, position);
    put_u32(data, 4, float_bits(joint_speed_limits_[index]));
    send_frame(0x100 + m.send_id, data);
  }
}

void Robot::decode_frame(uint32_t id, const std::array<uint8_t, 8>& data, Clock::time_point received_at) {
  int index = -1;
  if (family_ == Family::DM) {
    for (int i = 0; i < kMotorCount; ++i)
      if (id == motors_[i].feedback_id) { index = i; break; }
    // Register replies are not encoder feedback. In particular a mode reply
    // must not be decoded as a large, fictitious joint displacement.
    if (index >= 0 && (data[2] == 0x33 || data[2] == 0x55) &&
        (static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8)) == motors_[index].send_id) {
      if (data[2] == 0x33 && index == awaited_motor_ && data[3] == awaited_setting_) {
        setting_value_ = get_u32(data, 4);
        setting_received_ = true;
      }
      return;
    }
  } else {
    const auto communication_type = (id >> 24) & 0x1F;
    if (communication_type == kRsReadParameter) {
      const auto node_id = (id >> 8) & 0xFF;
      const auto host_id = id & 0xFF;
      const uint16_t setting = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
      if (awaited_motor_ >= 0 && node_id == motors_[awaited_motor_].send_id &&
          host_id == motors_[awaited_motor_].feedback_id && setting == awaited_setting_) {
        setting_value_ = setting == 0x7005 ? data[4] : get_u32(data, 4);
        setting_received_ = true;
      }
      return;
    }
    if (communication_type == kRsFaultReport) {
      const auto node_id = (id >> 8) & 0xFF;
      const auto host_id = id & 0xFF;
      const uint32_t faults = static_cast<uint32_t>(data[0]) |
                              (static_cast<uint32_t>(data[1]) << 8) |
                              (static_cast<uint32_t>(data[2]) << 16) |
                              (static_cast<uint32_t>(data[3]) << 24);
      if (faults != 0) {
        for (int i = 0; i < kMotorCount; ++i)
          if ((i < kArmDof || has_gripper_) &&
              node_id == motors_[i].send_id && host_id == motors_[i].feedback_id) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.fault = true;
            state_.fault_reason = "RS motor " + std::to_string(i + 1) +
                                  " reported fault mask " + std::to_string(faults);
            if (faults & 2u) state_.fault_reason += " (driver chip fault)";
            break;
          }
      }
      return;
    }
    // Command replies use type 2; active telemetry on RS firmware uses type
    // 24 with the same status flags and position/velocity/torque payload.
    if (communication_type != kRsOperationStatus && communication_type != kRsActiveReport) return;
    const auto node_id = (id >> 8) & 0xFF;
    const auto host_id = id & 0xFF;
    for (int i = 0; i < kMotorCount; ++i)
      if (node_id == motors_[i].send_id && host_id == motors_[i].feedback_id) {
        index = i; break;
      }
  }
  if (index < 0 || (index == kArmDof && !has_gripper_)) return;
  const auto& m = motors_[index];
  const uint16_t p = family_ == Family::DM
      ? static_cast<uint16_t>((data[1] << 8) | data[2])
      : static_cast<uint16_t>((data[0] << 8) | data[1]);
  const uint16_t v = family_ == Family::DM
      ? static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4))
      : static_cast<uint16_t>((data[2] << 8) | data[3]);
  const uint16_t t = family_ == Family::DM
      ? static_cast<uint16_t>(((data[4] & 0xF) << 8) | data[5])
      : static_cast<uint16_t>((data[4] << 8) | data[5]);
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (family_ == Family::DM) {
    const auto status = data[0] >> 4;
    if (status >= 8) {
      state_.fault = true;
      state_.fault_reason = "DM motor " + std::to_string(index + 1) +
                            " reported status " + std::to_string(status);
    }
  } else {
    const auto status_flags = (id >> 16) & 0x3F;
    motors_[index].rs_status_flags = status_flags;
    motors_[index].rs_run_state = (id >> 22) & 3;
    if (status_flags != 0 || motors_[index].rs_run_state != 0) rs_unhealthy_at_ = now;
    // During the one startup clear, delayed status flags may settle to zero.
    // Runtime faults remain latched even if a later frame looks healthy.
    if (status_flags != 0 && !rs_startup_recovery_) {
      state_.fault = true;
      state_.fault_reason = "RS motor " + std::to_string(index + 1) +
                            " reported status flags " + std::to_string(status_flags) +
                            " (" + rs_status_description(status_flags) + ")";
    }
  }
  state_.q[index] = m.direction * decode(p, -m.pmax, m.pmax, 16);
  const int feedback_bits = family_ == Family::RS ? 16 : 12;
  state_.dq[index] = m.direction * decode(v, -m.vmax, m.vmax, feedback_bits);
  state_.tau[index] = m.direction * decode(t, -m.tmax, m.tmax, feedback_bits);
  motors_[index].last_feedback = std::max(motors_[index].last_feedback, received_at);
  ++state_.feedback_count[index];
}

void Robot::read_feedback_until(Clock::time_point deadline, bool wait) {
  if (serial_) read_serial_feedback_until(deadline, wait);
  else read_can_feedback_until(deadline, wait);
}

void Robot::read_can_feedback_until(Clock::time_point deadline, bool wait) {
  while (Clock::now() < deadline) {
    can_frame frame{};
    iovec io{&frame, sizeof(frame)};
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(__kernel_timespec)) + CMSG_SPACE(sizeof(uint32_t))]{};
    msghdr message{};
    message.msg_iov = &io;
    message.msg_iovlen = 1;
    message.msg_control = control;
    message.msg_controllen = sizeof(control);
    const auto size = ::recvmsg(fd_, &message, MSG_DONTWAIT);
    if (size == sizeof(frame)) {
      if ((frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) || frame.can_dlc < 8) continue;
      const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
      if (extended != (family_ == Family::RS)) continue;
      std::array<uint8_t, 8> data{};
      std::memcpy(data.data(), frame.data, 8);
      auto received_at = Clock::now();
      for (auto* cmsg = CMSG_FIRSTHDR(&message); cmsg; cmsg = CMSG_NXTHDR(&message, cmsg)) {
        if (cmsg->cmsg_level != SOL_SOCKET) continue;
        if (cmsg->cmsg_type == SO_TIMESTAMPNS_NEW && cmsg->cmsg_len >= CMSG_LEN(sizeof(__kernel_timespec))) {
          __kernel_timespec stamp{};
          std::memcpy(&stamp, CMSG_DATA(cmsg), sizeof(stamp));
          // Kernel RX time is CLOCK_REALTIME; subtract its age from the
          // monotonic clock. Decoding queued old packets must not renew safety
          // freshness as if they had just arrived from the motor.
          const auto arrival = std::chrono::seconds(stamp.tv_sec) + std::chrono::nanoseconds(stamp.tv_nsec);
          const auto age = std::max(std::chrono::system_clock::now().time_since_epoch() - arrival,
                                    std::chrono::system_clock::duration::zero());
          received_at -= std::chrono::duration_cast<Clock::duration>(age);
        } else if (cmsg->cmsg_type == SO_RXQ_OVFL && cmsg->cmsg_len >= CMSG_LEN(sizeof(uint32_t))) {
          uint32_t dropped = 0;
          std::memcpy(&dropped, CMSG_DATA(cmsg), sizeof(dropped));
          std::lock_guard<std::mutex> lock(mutex_);
          state_.can_rx_dropped = dropped;
        }
      }
      decode_frame(frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK), data, received_at);
      continue;
    }
    if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      throw std::system_error(errno, std::generic_category(), "CAN read");
    if (!wait) break;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining.count() <= 0) break;
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
    if (rc < 0 && errno != EINTR) throw std::system_error(errno, std::generic_category(), "CAN poll");
  }
}

bool Robot::parse_serial_frame(uint32_t& id, std::array<uint8_t, 8>& data, uint8_t& dlc) {
  while (!serial_rx_.empty() && serial_rx_.front() != 0xAA) serial_rx_.erase(serial_rx_.begin());
  while (serial_rx_.size() >= 16) {
    if (serial_rx_[0] != 0xAA) {
      serial_rx_.erase(serial_rx_.begin());
      continue;
    }
    if (serial_rx_[15] != 0x55 || serial_rx_[1] != 0x11) {
      serial_rx_.erase(serial_rx_.begin());
      continue;
    }
    const uint8_t flags = serial_rx_[2];
    dlc = flags & 0x3F;
    if ((flags & 0xC0) || dlc > 8) {
      serial_rx_.erase(serial_rx_.begin(), serial_rx_.begin() + 16);
      continue;
    }
    id = static_cast<uint32_t>(serial_rx_[3]) |
         (static_cast<uint32_t>(serial_rx_[4]) << 8) |
         (static_cast<uint32_t>(serial_rx_[5]) << 16) |
         (static_cast<uint32_t>(serial_rx_[6]) << 24);
    std::copy_n(serial_rx_.begin() + 7, 8, data.begin());
    serial_rx_.erase(serial_rx_.begin(), serial_rx_.begin() + 16);
    return true;
  }
  return false;
}

void Robot::read_serial_feedback_until(Clock::time_point deadline, bool wait) {
  std::array<uint8_t, 256> buffer{};
  while (Clock::now() < deadline) {
    uint32_t id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
    if (parse_serial_frame(id, data, dlc)) {
      if (dlc >= 8) decode_frame(id, data);
      continue;
    }
    const auto n = ::read(fd_, buffer.data(), buffer.size());
    if (n > 0) {
      serial_rx_.insert(serial_rx_.end(), buffer.begin(), buffer.begin() + n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      throw std::system_error(errno, std::generic_category(), "serial read");
    if (!wait) break;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
    if (remaining.count() <= 0) break;
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
    if (rc < 0 && errno != EINTR) throw std::system_error(errno, std::generic_category(), "poll serial read");
  }
  uint32_t id = 0;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
  while (parse_serial_frame(id, data, dlc)) if (dlc >= 8) decode_frame(id, data);
}

void Robot::enable_all() {
  try {
    for (int i = 0; i < kMotorCount; ++i) {
      if (i == kArmDof && !has_gripper_) continue;
      // RS active reporting was already enabled by request_feedback().
      send_special(i, 0xFC);
    }
  } catch (...) { disable_all(); throw; }
}

void Robot::disable_all() noexcept {
  if (fd_ < 0) return;
  for (int i = 0; i < kMotorCount; ++i) {
    if (i == kArmDof && !has_gripper_) continue;
    try { send_special(i, 0xFD); } catch (...) {}
  }
}

void Robot::run() noexcept {
  auto next = Clock::now();
  auto previous_cycle = next;
  try {
    while (running_) {
      next += kPeriod;
      const auto cycle_started = Clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.cycle_interval_sec = std::chrono::duration<double>(cycle_started - previous_cycle).count();
      }
      previous_cycle = cycle_started;
      update_cartesian_target();
      // A late wakeup/IK/send can exhaust the previous tick's read budget.
      // Consume queued feedback before testing freshness or sending anything.
      // Bound draining under continuous traffic, and never wait on an empty RX.
      read_feedback_until(Clock::now() + std::chrono::milliseconds(2), false);
      JointArray command{};
      JointArray velocity{};
      JointArray mit_velocity{};
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = Clock::now();
        if (state_.fault) throw std::runtime_error(state_.fault_reason);
        for (int i = 0; i < kMotorCount; ++i) {
          if (i == kArmDof && !has_gripper_) continue;
          if (now - motors_[i].last_feedback > kFeedbackLimit) {
            std::ostringstream reason;
            reason << std::fixed << std::setprecision(1)
                   << "stale feedback from motor " << i + 1 << " (limit_ms=100, ages_ms=[";
            for (int j = 0; j < kMotorCount; ++j) {
              if (j == kArmDof && !has_gripper_) continue;
              if (j) reason << ", ";
              reason << std::chrono::duration<double, std::milli>(now - motors_[j].last_feedback).count();
            }
            reason << "], cycle_ms=" << state_.cycle_interval_sec * 1000
                   << ", overruns=" << state_.overruns << ", rx_dropped=" << state_.can_rx_dropped << ")";
            throw std::runtime_error(reason.str());
          }
        }
        const bool timed_out = std::chrono::duration<double>(now - last_command_).count() > command_timeout_sec_;
        if (timed_out && cartesian_enabled_) {
          cancel_cartesian_locked();
          target_ = sent_;
        }
        const double gripper_step = max_joint_speed_ * 0.01;
        double arm_scale = 1.0;
        for (int i = 0; i < kArmDof && !timed_out; ++i) {
          const double delta = std::abs(target_[i] - sent_[i]);
          if (delta > 0) arm_scale = std::min(arm_scale, target_speed_limits_[i] * 0.01 / delta);
        }
        state_.joint_step_scale = arm_scale;
        for (int i = 0; i < kMotorCount; ++i) {
          const auto goal = timed_out ? sent_[i] : target_[i];
          // Scale the six arm joints together so they make equal progress
          // toward one IK solution. The optional motor gripper is independent.
          const double delta = i < kArmDof ? (goal - sent_[i]) * arm_scale
                                         : std::clamp(goal - sent_[i], -gripper_step, gripper_step);
          sent_[i] += delta;
          command[i] = sent_[i];
          velocity[i] = delta / 0.01;
          state_.joint_speed_utilization[i] = std::abs(velocity[i]) / target_speed_limits_[i];
          // A finite difference of piecewise Cartesian/IK samples is not a
          // continuous velocity trajectory. In MIT it also enters the Kd
          // term directly, exposing waypoint-boundary ripple as torque kicks.
          // Allow position tracking with damping (dq_des=0), as in libtron2rt.
          // Keep the optional CAN gripper's existing MIT behavior independent.
          if ((i < kArmDof && !position_mode(i) && mit_velocity_feedforward_) ||
              (i == kArmDof && has_gripper_))
            mit_velocity[i] = velocity[i];
        }
        state_.command_velocity = velocity;
        state_.mit_command_velocity = mit_velocity;
        if (cartesian_enabled_ && cartesian_endpoint_) {
          bool reached = true;
          for (int i = 0; i < kArmDof; ++i)
            reached &= std::abs(target_[i] - sent_[i]) < 1e-12;
          if (reached) state_.cartesian_active = false;
        }
        ++state_.cycles;
      }
      for (int i = 0; i < kMotorCount; ++i) {
        if (i == kArmDof && !has_gripper_) continue;
        const auto& m = motors_[i];
        if (position_mode(i)) {
          send_position(i, command[i]);
        } else if (family_ == Family::RS) {
          const auto payload = rs_mit_payload(command[i] * m.direction, mit_velocity[i] * m.direction, m.kp, m.kd,
                                              m.pmax, m.vmax, m.kpmax, m.kdmax);
          const auto torque = encode(0.0, -m.tmax, m.tmax, 16);
          send_rs_mit(i, payload, torque);
        } else {
          send_frame(m.send_id, mit_frame(command[i] * m.direction, mit_velocity[i] * m.direction, m.kp, m.kd,
                                          0.0, m.pmax, m.vmax, m.tmax, m.kpmax, m.kdmax));
        }
      }
      read_feedback_until(std::min(next, Clock::now() + std::chrono::milliseconds(4)));
      if (Clock::now() > next) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++state_.overruns;
        next = Clock::now();
      }
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.fault = true;
    state_.fault_reason = e.what();
  }
  running_ = false;
  disable_all();
}

} // namespace rebot_rt
