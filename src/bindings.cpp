#include "rebot_rt/control.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace rebot_rt;

PYBIND11_MODULE(_native, m) {
  m.doc() = "C++ 100 Hz reBot B601 DM serial/SocketCAN, RS SocketCAN control and IK";
  py::enum_<Family>(m, "Family")
      .value("DM", Family::DM)
      .value("RS", Family::RS);
  py::class_<MotorConfig>(m, "MotorConfig")
      .def(py::init<>())
      .def_readwrite("send_id", &MotorConfig::send_id)
      .def_readwrite("feedback_id", &MotorConfig::feedback_id)
      .def_readwrite("model", &MotorConfig::model)
      .def_readwrite("pmax", &MotorConfig::pmax)
      .def_readwrite("vmax", &MotorConfig::vmax)
      .def_readwrite("tmax", &MotorConfig::tmax)
      .def_readwrite("kp", &MotorConfig::kp)
      .def_readwrite("kd", &MotorConfig::kd)
      .def_readwrite("vel_kp", &MotorConfig::vel_kp)
      .def_readwrite("vel_ki", &MotorConfig::vel_ki)
      .def_readwrite("pos_kp", &MotorConfig::pos_kp)
      .def_readwrite("pos_ki", &MotorConfig::pos_ki)
      .def_readwrite("kpmax", &MotorConfig::kpmax)
      .def_readwrite("kdmax", &MotorConfig::kdmax)
      .def_readwrite("direction", &MotorConfig::direction)
      .def("set_model", &MotorConfig::set_model, py::arg("family"), py::arg("model"));
  m.def("default_motor_configs", &default_motor_configs, py::arg("family"));
  py::class_<State>(m, "State")
      .def_readonly("q", &State::q)
      .def_readonly("dq", &State::dq)
      .def_readonly("tau", &State::tau)
      .def_readonly("target", &State::target)
      .def_readonly("command", &State::command)
      .def_readonly("command_velocity", &State::command_velocity)
      .def_readonly("mit_command_velocity", &State::mit_command_velocity)
      .def_readonly("gravity_torque", &State::gravity_torque)
      .def_readonly("mit_command_torque", &State::mit_command_torque)
      .def_readonly("gravity_torque_limited", &State::gravity_torque_limited)
      .def_readonly("joint_speed_limits", &State::joint_speed_limits)
      .def_readonly("joint_speed_utilization", &State::joint_speed_utilization)
      .def_readonly("joint_step_scale", &State::joint_step_scale)
      .def_readonly("cartesian_coordinated", &State::cartesian_coordinated)
      .def_readonly("cartesian_step_scale", &State::cartesian_step_scale)
      .def_readonly("cartesian_speed_scale", &State::cartesian_speed_scale)
      .def_readonly("cartesian_limiting_joint", &State::cartesian_limiting_joint)
      .def_readonly("fresh", &State::fresh)
      .def_readonly("feedback_age_sec", &State::feedback_age_sec)
      .def_readonly("feedback_count", &State::feedback_count)
      .def_readonly("can_rx_dropped", &State::can_rx_dropped)
      .def_readonly("cycle_interval_sec", &State::cycle_interval_sec)
      .def_readonly("age_sec", &State::age_sec)
      .def_readonly("cycles", &State::cycles)
      .def_readonly("overruns", &State::overruns)
      .def_readonly("running", &State::running)
      .def_readonly("fault", &State::fault)
      .def_readonly("fault_reason", &State::fault_reason)
      .def_readonly("cartesian_active", &State::cartesian_active)
      .def_readonly("cartesian_ik_valid", &State::cartesian_ik_valid)
      .def_readonly("cartesian_ik_success", &State::cartesian_ik_success)
      .def_readonly("cartesian_tracking_limited", &State::cartesian_tracking_limited)
      .def_readonly("cartesian_tracking_progress", &State::cartesian_tracking_progress)
      .def_readonly("cartesian_ik_error", &State::cartesian_ik_error)
      .def_readonly("cartesian_duration_sec", &State::cartesian_duration_sec)
      .def_readonly("cartesian_sequence", &State::cartesian_sequence)
      .def_readonly("cartesian_consumed_sequence", &State::cartesian_consumed_sequence)
      .def_readonly("cartesian_ik_solves", &State::cartesian_ik_solves);
  py::class_<IKStatus>(m, "IKStatus")
      .def_readonly("success", &IKStatus::success)
      .def_readonly("error", &IKStatus::error)
      .def_readonly("iterations", &IKStatus::iterations)
      .def_readonly("target", &IKStatus::target);
  py::class_<Robot>(m, "Robot")
      .def(py::init<Family, std::string, std::string, std::string, bool, double, double, uint32_t,
                    std::vector<MotorConfig>, std::string, bool, std::vector<double>, bool, double, double>(),
           py::arg("family"), py::arg("channel"), py::arg("urdf_path"),
           py::arg("end_frame") = "", py::arg("has_gripper") = true,
           py::arg("max_joint_speed") = 0.5, py::arg("command_timeout_sec") = 0.5,
           py::arg("dm_serial_baud") = 921600,
           py::arg("motor_configs") = std::vector<MotorConfig>{},
           py::arg("control_mode") = "mit", py::arg("mit_velocity_feedforward") = true,
           py::arg("max_joint_speeds") = std::vector<double>{}, py::arg("coordinate_cartesian_speed") = true,
           py::arg("max_cartesian_linear_speed") = 0.0, py::arg("max_cartesian_angular_speed") = 0.0)
      .def("connect", &Robot::connect, py::call_guard<py::gil_scoped_release>())
      .def("start", &Robot::start, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Robot::stop, py::call_guard<py::gil_scoped_release>())
      .def("close", &Robot::close, py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("connected", &Robot::connected)
      .def_property_readonly("running", &Robot::running)
      .def("get_state", &Robot::get_state)
      .def("get_target_joints", &Robot::get_target_joints)
      .def("set_target_joints", &Robot::set_target_joints, py::arg("q"),
           py::arg("max_joint_speed") = py::none())
      .def("hold_position", &Robot::hold_position)
      .def("configure_gravity_compensation", &Robot::configure_gravity_compensation,
           py::arg("scale"), py::arg("torque_limits_nm"), py::arg("slew_rate_nm_s") = 5.0,
           py::arg("gravity") = Eigen::Vector3d(0, 0, -9.81),
           py::arg("torque_offsets_nm") = std::vector<double>{})
      .def("gravity_torques", &Robot::gravity_torques, py::arg("q"))
      .def("set_target_pose", &Robot::set_target_pose, py::arg("pose"),
           py::arg("gripper") = 0.0, py::call_guard<py::gil_scoped_release>())
      .def("queue_target_pose", &Robot::queue_target_pose, py::arg("pose"), py::arg("gripper") = 0.0)
      .def("forward_kinematics", py::overload_cast<>(&Robot::forward_kinematics, py::const_))
      .def("forward_kinematics", py::overload_cast<const JointArray&>(&Robot::forward_kinematics, py::const_),
           py::arg("q"))
      .def("__enter__", [](Robot& self) -> Robot& { self.connect(); self.start(); return self; },
           py::return_value_policy::reference_internal)
      .def("__exit__", [](Robot& self, py::object, py::object, py::object) {
          py::gil_scoped_release release;
          self.close();
      });
}
