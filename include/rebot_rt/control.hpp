#pragma once

#include "rebot_rt/robot_model.hpp"

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace rebot_rt {

constexpr int kArmDof = 6;
constexpr int kMotorCount = 7;
using JointArray = std::array<double, kMotorCount>;
using Clock = std::chrono::steady_clock;

enum class Family { DM, RS };

struct MotorConfig {
  uint32_t send_id = 0;
  uint32_t feedback_id = 0;
  std::string model;
  double pmax = 0.0;
  double vmax = 0.0;
  double tmax = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double vel_kp = 0.0;
  double vel_ki = 0.0;
  double pos_kp = 0.0;
  double pos_ki = 0.0;
  double kpmax = 500.0;
  double kdmax = 5.0;
  double direction = 1.0;

  void set_model(Family family, const std::string& new_model);
};

std::array<MotorConfig, kMotorCount> default_motor_configs(Family family);

struct State {
  JointArray q{};            // public joint frame, radians
  JointArray dq{};           // radians/s
  JointArray tau{};          // N m
  JointArray target{};       // latest requested joint goal
  JointArray command{};      // position actually sent by the 100 Hz limiter
  JointArray command_velocity{};      // position-ramp slope, regardless of motor mode
  JointArray mit_command_velocity{};  // MIT dq field before direction/quantization; zero for pos_vel
  std::array<bool, kMotorCount> fresh{};
  double age_sec = 0.0;
  uint64_t cycles = 0;
  uint64_t overruns = 0;
  bool running = false;
  bool fault = false;
  std::string fault_reason;
  bool cartesian_active = false;
  bool cartesian_ik_valid = false;
  bool cartesian_ik_success = false;
  double cartesian_ik_error = 0.0;
  double cartesian_duration_sec = 0.01;
  uint64_t cartesian_sequence = 0;
  uint64_t cartesian_consumed_sequence = 0;
  uint64_t cartesian_ik_solves = 0;
};

struct IKStatus {
  bool success = false;
  double error = 0.0;
  int iterations = 0;
  JointArray target{};
};

class Robot {
public:
  Robot(Family family, std::string channel, std::string urdf_path,
        std::string end_frame = "", bool has_gripper = true,
        double max_joint_speed = 0.5, double command_timeout_sec = 0.5,
        uint32_t dm_serial_baud = 921600,
        std::vector<MotorConfig> motor_configs = {},
        std::string control_mode = "mit", bool mit_velocity_feedforward = true);
  ~Robot();
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  void connect();
  void start();
  void stop();
  void close();
  bool connected() const { return fd_ >= 0; }
  bool running() const { return running_.load(); }
  State get_state() const;
  JointArray get_target_joints() const;
  void set_target_joints(const JointArray& q, std::optional<double> max_joint_speed = std::nullopt);
  void hold_position();
  IKStatus set_target_pose(const Eigen::Matrix4d& pose, double gripper = 0.0);
  uint64_t queue_target_pose(const Eigen::Matrix4d& pose, double gripper = 0.0);
  Eigen::Matrix4d forward_kinematics() const;
  Eigen::Matrix4d forward_kinematics(const JointArray& q) const;

private:
  struct Motor {
    uint32_t send_id = 0;
    uint32_t feedback_id = 0;
    double pmax = 12.5;
    double vmax = 0.0;
    double tmax = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double vel_kp = 0.0;
    double vel_ki = 0.0;
    double pos_kp = 0.0;
    double pos_ki = 0.0;
    double kpmax = 500.0;
    double kdmax = 5.0;
    double direction = 1.0;
    double lower = -12.5;
    double upper = 12.5;
    Clock::time_point last_feedback{};
    uint8_t rs_status_flags = 0;
    uint8_t rs_run_state = 0;
  };

  Family family_;
  std::string control_mode_;
  bool mit_velocity_feedforward_;
  std::string channel_;
  std::string end_frame_;
  bool has_gripper_;
  double max_joint_speed_;
  double target_joint_speed_;
  double command_timeout_sec_;
  uint32_t dm_serial_baud_;
  rebotarm::RobotModel model_;
  int frame_id_;
  std::array<Motor, kMotorCount> motors_;
  int awaited_motor_ = -1;
  uint16_t awaited_setting_ = 0;
  uint32_t setting_value_ = 0;
  bool setting_received_ = false;
  bool rs_startup_recovery_ = false;
  Clock::time_point rs_unhealthy_at_{};
  int fd_ = -1;
  bool serial_ = false;
  std::vector<uint8_t> serial_rx_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  mutable std::mutex mutex_;
  mutable std::mutex model_mutex_;
  State state_;
  JointArray target_{};
  JointArray sent_{};
  Clock::time_point last_command_{};
  bool cartesian_enabled_ = false;
  bool cartesian_endpoint_ = false;
  Eigen::Matrix4d cartesian_start_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d cartesian_goal_ = Eigen::Matrix4d::Identity();
  Clock::time_point cartesian_last_update_{};
  Clock::time_point cartesian_started_{};
  double cartesian_gripper_ = 0.0;

  void cancel_cartesian_locked();
  void update_cartesian_target();

  void send_frame(uint32_t id, const std::array<uint8_t, 8>& data, bool extended = false);
  void send_special(int index, uint8_t command);
  void send_rs_command(int index, uint32_t communication_type, uint16_t extra_data,
                       const std::array<uint8_t, 8>& data);
  void send_rs_mit(int index, const std::array<uint8_t, 8>& data, uint16_t torque);
  void request_feedback(int index);
  void recover_rs_startup_faults();
  bool position_mode(int index) const { return index < kArmDof && control_mode_ == "pos_vel"; }
  uint32_t read_setting(int index, uint16_t setting);
  void write_setting(int index, uint16_t setting, uint32_t value);
  void configure_motor(int index);
  void send_position(int index, double q);
  void read_feedback_until(Clock::time_point deadline);
  void read_can_feedback_until(Clock::time_point deadline);
  void read_serial_feedback_until(Clock::time_point deadline);
  bool parse_serial_frame(uint32_t& id, std::array<uint8_t, 8>& data, uint8_t& dlc);
  void decode_frame(uint32_t id, const std::array<uint8_t, 8>& data);
  void enable_all();
  void disable_all() noexcept;
  void run() noexcept;
  Eigen::VectorXd model_q(const JointArray& q) const;
};

} // namespace rebot_rt
