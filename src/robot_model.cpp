#include "rebot_rt/robot_model.hpp"
#include "rebot_rt/se3_conv.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <cmath>
#include <chrono>
#include <random>
#include <stdexcept>

namespace rebotarm {

namespace {

bool has_frame(const pinocchio::Model& model, const std::string& name) {
  for (const auto& frame : model.frames) {
    if (frame.name == name) return true;
  }
  return false;
}

void reduce_to_arm_dof_if_needed(pinocchio::Model& model) {
  if (model.nq <= 6 ||
      !(has_frame(model, "gripper_end") || has_frame(model, "taccap_eef_link"))) return;

  std::vector<pinocchio::JointIndex> joints_to_lock;
  int movable_seen = 0;
  for (pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id) {
    if (model.joints[joint_id].idx_q() < 0) continue;
    ++movable_seen;
    if (movable_seen > 6) joints_to_lock.push_back(joint_id);
  }
  if (joints_to_lock.empty()) return;

  model = pinocchio::buildReducedModel(model, joints_to_lock, pinocchio::neutral(model));
}

}  // namespace

RobotModel::RobotModel(const std::string& urdf_path) : urdf_path_(urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, model_);
  reduce_to_arm_dof_if_needed(model_);
  data_ = pinocchio::Data(model_);
}

std::vector<std::string> RobotModel::joint_names() const {
  std::vector<std::string> out;
  for (std::size_t i = 1; i < model_.names.size(); ++i) {
    if (model_.joints[i].idx_q() >= 0) out.push_back(model_.names[i]);
  }
  return out;
}

std::vector<std::pair<double, double>> RobotModel::joint_limits() const {
  std::vector<std::pair<double, double>> out;
  for (std::size_t i = 1; i < model_.joints.size(); ++i) {
    const int idx_q = model_.joints[i].idx_q();
    if (idx_q < 0) continue;
    double lo = model_.lowerPositionLimit[idx_q];
    double hi = model_.upperPositionLimit[idx_q];
    if (!std::isfinite(lo) && !std::isfinite(hi)) {
      out.emplace_back(-std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity());
    } else {
      out.emplace_back(lo, hi);
    }
  }
  return out;
}

std::vector<std::string> RobotModel::all_frame_names() const {
  std::vector<std::string> out;
  for (const auto& f : model_.frames) out.push_back(f.name);
  return out;
}

int RobotModel::frame_id(const std::string& name) const {
  if (!has_frame(model_, name))
    throw std::invalid_argument("unknown URDF frame: " + name);
  return static_cast<int>(model_.getFrameId(name));
}

std::string RobotModel::default_end_effector_frame_name() const {
  for (const char* name : {"taccap_eef_link", "end_link", "gripper_end"}) {
    for (const auto& frame : model_.frames) {
      if (frame.name == name) return name;
    }
  }
  throw std::runtime_error(
      "RobotModel has no default end-effector frame (expected taccap_eef_link, end_link, or gripper_end)");
}

int RobotModel::end_effector_frame_id() const {
  return static_cast<int>(model_.getFrameId(default_end_effector_frame_name()));
}

Eigen::VectorXd RobotModel::neutral() const { return pinocchio::neutral(model_); }

void RobotModel::set_gravity(const Eigen::Vector3d& g) { model_.gravity.linear(g); }
Eigen::Vector3d RobotModel::get_gravity() const { return model_.gravity.linear(); }

Eigen::VectorXd RobotModel::random_configuration() const {
  // 与 inverse_kinematics.py 的随机重试一致：limit 无穷时退化为 [-pi, pi]。
  static thread_local std::mt19937 rng{std::random_device{}()};
  Eigen::VectorXd q(model_.nq);
  for (int j = 0; j < model_.nq; ++j) {
    double lo = model_.lowerPositionLimit[j];
    double hi = model_.upperPositionLimit[j];
    if (!std::isfinite(lo)) lo = -M_PI;
    if (!std::isfinite(hi)) hi = M_PI;
    std::uniform_real_distribution<double> d(lo, hi);
    q[j] = d(rng);
  }
  return q;
}

Eigen::VectorXd RobotModel::clamp_config(const Eigen::VectorXd& q) const {
  Eigen::VectorXd out = q;
  for (int j = 0; j < model_.nq; ++j) {
    double lo = std::isfinite(model_.lowerPositionLimit[j]) ? model_.lowerPositionLimit[j] : 0.0;
    double hi = std::isfinite(model_.upperPositionLimit[j]) ? model_.upperPositionLimit[j] : 0.0;
    out[j] = std::min(std::max(out[j], lo), hi);
  }
  return out;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d>
RobotModel::fk(const Eigen::VectorXd& q, const std::string& frame_name) const {
  if (q.size() != model_.nq)
    throw std::invalid_argument("q 维度必须为 nq");
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  if (!frame_name.empty() && !has_frame(model_, frame_name))
    throw std::invalid_argument("unknown URDF frame: " + frame_name);
  const auto fid = frame_name.empty() ? model_.getFrameId(default_end_effector_frame_name())
                                      : model_.getFrameId(frame_name);
  const pinocchio::SE3& oMf = data_.oMf[fid];
  return {oMf.translation(), oMf.rotation(), oMf.toHomogeneousMatrix()};
}

Eigen::MatrixXd RobotModel::frame_jacobian_local(const Eigen::VectorXd& q, int frame_id) const {
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, data_, static_cast<pinocchio::FrameIndex>(frame_id),
                              pinocchio::LOCAL, J);
  return J;
}

std::pair<double, Eigen::Matrix<double, 6, 1>>
RobotModel::compute_error(const Eigen::VectorXd& q, const pinocchio::SE3& target,
                          int end_frame_id) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3& T_cur = data_.oMf[static_cast<pinocchio::FrameIndex>(end_frame_id)];
  Eigen::Matrix<double, 6, 1> err = pinocchio::log6(T_cur.inverse() * target).toVector();
  return {err.norm(), err};
}

IKResult RobotModel::solve_ik(const Eigen::Matrix4d& target_h, const Eigen::VectorXd& q_init,
                              int end_frame_id, const IKParams& params) const {
  const auto started = std::chrono::steady_clock::now();
  const pinocchio::SE3 target = se3_from_homogeneous(target_h);
  // Feedback around a calibrated mechanical zero can be slightly outside the
  // URDF bounds. Project the seed before the convergence check so an exact FK
  // round trip cannot accept an out-of-limit solution and fail at command validation.
  Eigen::VectorXd q = clamp_config(q_init);
  auto [prev_err, err] = compute_error(q, target, end_frame_id);

  for (int iteration = 0; iteration < params.max_iter; ++iteration) {
    if (prev_err < params.tolerance)
      return IKResult{q, true, prev_err, iteration};
    if (params.max_time_sec > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= params.max_time_sec)
      return IKResult{q, false, prev_err, iteration};

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();
    pinocchio::getFrameJacobian(model_, data_, static_cast<pinocchio::FrameIndex>(end_frame_id),
                                pinocchio::LOCAL, J);

    // B601 has six driven arm joints. The two finger joints in its URDF are
    // passive for arm IK; never move them to improve a Cartesian solution.
    if (model_.nv > 6) J.rightCols(model_.nv - 6).setZero();

    const double lam = params.damping * std::max(1.0, prev_err * 10.0);
    Eigen::MatrixXd JJT = J * J.transpose();
    JJT.diagonal().array() += lam;
    Eigen::VectorXd dq = params.step_size * J.transpose() * JJT.ldlt().solve(err);

    double alpha = 1.0;
    bool improved = false;
    for (int k = 0; k < 4; ++k) {
      Eigen::VectorXd q_new = clamp_config(pinocchio::integrate(model_, q, alpha * dq));
      auto [new_err, err_new] = compute_error(q_new, target, end_frame_id);
      if (new_err < prev_err) {
        q = q_new;
        err = err_new;
        prev_err = new_err;
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    (void)improved;  // 线搜索全失败时保持当前 q 继续（与 Python 一致）
  }
  return IKResult{q, false, prev_err, params.max_iter};
}

IKResult RobotModel::solve_tracking_ik(const Eigen::Matrix4d& target_h, const Eigen::VectorXd& q_init,
                                      int end_frame_id, const Eigen::VectorXd& lower,
                                      const Eigen::VectorXd& upper, const IKParams& params) const {
  const auto started = std::chrono::steady_clock::now();
  const auto expired = [&] {
    return params.max_time_sec > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() >= params.max_time_sec;
  };
  const pinocchio::SE3 target = se3_from_homogeneous(target_h);
  const double initial_error = compute_error(q_init, target, end_frame_id).first;
  if (lower.size() != model_.nq || upper.size() != model_.nq || model_.nq != model_.nv)
    throw std::invalid_argument("tracking IK requires scalar joints and matching joint bounds");
  if ((lower.array() > upper.array()).any())
    return IKResult{q_init, false, initial_error, 0, false};

  Eigen::VectorXd q = q_init.cwiseMax(lower).cwiseMin(upper);
  auto [error, residual] = compute_error(q, target, end_frame_id);
  int iteration = 0;
  for (; iteration < params.max_iter && error >= params.tolerance && !expired(); ++iteration) {
    Eigen::MatrixXd J = frame_jacobian_local(q, end_frame_id);
    const pinocchio::SE3 delta = data_.oMf[static_cast<pinocchio::FrameIndex>(end_frame_id)].inverse() * target;
    // Negative derivative of log(current^-1 * target), for a LOCAL Jacobian.
    J = (pinocchio::Jlog6(delta.inverse()) * J).eval();
    if (model_.nv > 6) J.rightCols(model_.nv - 6).setZero();

    Eigen::MatrixXd free_J = J;
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    // Freeze joints whose proposed step points out of an active bound, then
    // resolve with the remaining joints so tangential motion stays possible.
    for (int pass = 0; pass <= 6; ++pass) {
      Eigen::Matrix<double, 6, 6> normal = free_J * free_J.transpose();
      normal.diagonal().array() += params.damping;
      dq = free_J.transpose() * normal.ldlt().solve(residual);
      bool blocked = false;
      for (int j = 0; j < std::min(6, model_.nv); ++j) {
        if ((q[j] <= lower[j] + 1e-12 && dq[j] < 0) ||
            (q[j] >= upper[j] - 1e-12 && dq[j] > 0)) {
          free_J.col(j).setZero();
          blocked = true;
        }
      }
      if (!blocked) break;
    }

    bool improved = false;
    // A projected gradient is a descent fallback if clipping the DLS step
    // loses its descent direction. Every accepted step is checked using FK.
    for (int method = 0; method < 2 && !improved && !expired(); ++method) {
      if (method == 1) dq = J.transpose() * residual;
      if (!dq.allFinite()) continue;
      // Keep the search local even for a target many metres outside the arm.
      const double radius = (upper - lower).maxCoeff();
      const double largest = dq.cwiseAbs().maxCoeff();
      if (largest > radius && largest > 0) dq *= radius / largest;
      double alpha = 1.0;
      for (int backtrack = 0; backtrack < 10 && !expired(); ++backtrack) {
        Eigen::VectorXd candidate = (q + alpha * dq).cwiseMax(lower).cwiseMin(upper);
        const auto [candidate_error, candidate_residual] = compute_error(candidate, target, end_frame_id);
        if (std::isfinite(candidate_error) && candidate_error < error - 1e-10) {
          q = candidate;
          error = candidate_error;
          residual = candidate_residual;
          improved = true;
          break;
        }
        alpha *= 0.5;
      }
    }
    if (!improved) break;
  }
  // Never pass a failed iterate to the controller merely because it exists.
  const bool progress = q.allFinite() && std::isfinite(error) && error < initial_error - 1e-9;
  const bool success = error < params.tolerance && error <= initial_error + 1e-12;
  return IKResult{q, success, error, iteration, progress};
}

IKResult RobotModel::solve_coordinated_tracking_ik(
    const Eigen::Matrix4d& target_h, const Eigen::VectorXd& q_init, int end_frame_id,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, const IKParams& params) const {
  const auto started = std::chrono::steady_clock::now();
  const double budget = params.max_time_sec > 0 ? params.max_time_sec : 0.003;
  const auto elapsed = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  };
  const pinocchio::SE3 target = se3_from_homogeneous(target_h);
  const double initial_error = compute_error(q_init, target, end_frame_id).first;
  if (lower.size() != model_.nq || upper.size() != model_.nq || model_.nq != model_.nv)
    throw std::invalid_argument("tracking IK requires scalar joints and matching joint bounds");
  if (initial_error < params.tolerance && (q_init.array() >= lower.array()).all() &&
      (q_init.array() <= upper.array()).all())
    return IKResult{q_init, true, initial_error, 0, false, true, 1.0, -1};

  const pinocchio::SE3 current = data_.oMf[static_cast<pinocchio::FrameIndex>(end_frame_id)];
  Eigen::Matrix<double, 6, 1> twist;
  // Derivative of linear translation + shortest-path rotation, in LOCAL coordinates.
  twist.head<3>() = current.rotation().transpose() * (target.translation() - current.translation());
  twist.tail<3>() = pinocchio::log3(current.rotation().transpose() * target.rotation());
  Eigen::MatrixXd J = frame_jacobian_local(q_init, end_frame_id).leftCols(6);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-6);
  const Eigen::VectorXd dq = svd.solve(twist);
  double scale = 1.0;
  int limiting_joint = -1;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(dq[i]) < 1e-12) continue;
    const double room = dq[i] > 0 ? upper[i] - q_init[i] : q_init[i] - lower[i];
    const double ratio = std::max(0.0, room) / std::abs(dq[i]);
    if (ratio < scale) { scale = ratio; limiting_joint = i; }
  }
  if (scale < 1.0) scale *= 0.95;  // leave room for nonlinear IK correction
  const Eigen::Quaterniond from(current.rotation()), to(target.rotation());
  int iterations = 0;
  // Reserve part of the same budget for the boundary-approach fallback. Near a
  // singularity a pseudoinverse need not preserve the requested twist; FK, not
  // the linearization, decides whether a scaled Cartesian sample is feasible.
  for (int attempt = 0; attempt < 8 && scale > 1e-5 && elapsed() < budget * 0.65; ++attempt) {
    Eigen::Matrix4d sample = Eigen::Matrix4d::Identity();
    sample.block<3, 1>(0, 3) = current.translation() + scale * (target.translation() - current.translation());
    sample.block<3, 3>(0, 0) = from.slerp(scale, to).normalized().toRotationMatrix();
    IKParams local = params;
    local.damping = 1e-8;
    local.tolerance = std::min(1e-6, initial_error * scale * 0.01);
    local.max_time_sec = budget * 0.65 - elapsed();
    if (local.max_time_sec <= 0) break;
    auto result = solve_tracking_ik(sample, q_init, end_frame_id, lower, upper, local);
    iterations += result.iterations;
    if (result.success) {
      const double error = compute_error(result.q, target, end_frame_id).first;
      if (std::isfinite(error) && error < initial_error - 1e-9)
        return IKResult{result.q, error < params.tolerance, error, iterations, true, true, scale, limiting_joint};
    }
    scale *= 0.5;
  }
  IKParams fallback = params;
  fallback.max_time_sec = budget - elapsed();
  if (fallback.max_time_sec <= 0)
    return IKResult{q_init, false, initial_error, iterations, false, false, 0.0, limiting_joint};
  auto result = solve_tracking_ik(target_h, q_init, end_frame_id, lower, upper, fallback);
  result.iterations += iterations;
  result.limiting_joint = limiting_joint;
  return result;
}

IKResult RobotModel::solve_ik_with_retry(const Eigen::Matrix4d& target, const Eigen::VectorXd& q_seed,
                                         int end_frame_id, const IKParams& params,
                                         int max_retries) const {
  IKResult best = solve_ik(target, q_seed, end_frame_id, params);
  if (best.success) return best;
  for (int r = 0; r < max_retries; ++r) {
    IKResult res = solve_ik(target, random_configuration(), end_frame_id, params);
    if (res.error < best.error) best = res;
    if (best.success) break;
  }
  return best;
}

}  // namespace rebotarm
