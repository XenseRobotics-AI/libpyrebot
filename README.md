# libpyrebot

Native C++ controller for a six joint reBot B601 arm plus its optional motor gripper. The Python package `rebot_rt` contains only pybind11 bindings and complete bundled robot models. Serial/CAN communication, position/velocity and MIT commands, a 100 Hz control thread, FK, and IK all run in C++.

## Supported hardware

- DM: Damiao USB serial bridge (`/dev/ttyACM*`, 921600 baud) or SocketCAN; 4340P × 3 and 4310 × 4 by default; 11 bit CAN frames.
- RS: SocketCAN (`can0`, etc.); rs-06 × 3 and rs-00 × 4 by default; RobStride private 29 bit extended CAN frames.

The default motor IDs are 1–7. DM feedback IDs are `0x11`–`0x17`; the RS host ID is `0xFD`. The bundled URDF coordinates are the raw zeroed motor coordinates, so the default `direction` is `1` for both families. Check the joint signs on your hardware before moving; the older LeRobot joint follower uses a different RS display convention.

## Build

Build with the Python interpreter that will import the module. Pinocchio and pybind11 must be available in that environment; CMake also needs Eigen and the Pinocchio C++ libraries.

```bash
cmake -S third_party/libpyrebot -B /tmp/libpyrebot-build \
  -DPython3_EXECUTABLE=/home/xense/miniforge3/envs/rebot/bin/python \
  -DPINOCCHIO_PREFIX=/home/xense/miniforge3/envs/rebot
cmake --build /tmp/libpyrebot-build -j2
PYTHONPATH=third_party/libpyrebot /home/xense/miniforge3/envs/rebot/bin/python \
  -m unittest discover -s third_party/libpyrebot/tests -v
```

CMake writes `_native` into `third_party/libpyrebot/rebot_rt`. `pip install -e third_party/libpyrebot --no-build-isolation` then makes the package importable in the selected environment. Rebuild when the Python minor version changes. Packaging includes only the extension matching the active Python and tags the wheel for that interpreter and platform; it does not invoke CMake.

## Use from LeRobot

The new robot types are `rebot_b601_rt` and `bi_rebot_b601_rt`. Example single arm config:

```python
from lerobot.robots.rebot_b601_rt import RebotB601RTConfig

config = RebotB601RTConfig(
    id="rebot_dm_left",
    motor_family="dm",
    channel="/dev/ttyACM0",
    dm_serial_baud=921600,
    max_joint_speed=0.5,  # rad/s
)
```

For RS set `motor_family="rs", channel="can0"` (family names are case insensitive). Without a TacCap gripper, DM uses `assets/DM/urdf/ReBot_Arm_DM.urdf` and its `end_link` frame; RS uses `assets/RS/urdf/ReBot_Arm_RS.urdf` and its `gripper_end` frame. With `gripper.type: taccap_follower`, **either motor family** uses the bundled `assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf` and its `taccap_eef_link` frame for both FK and IK. These are the same model assets as the standalone `libpyrebot/rebot_rt/assets` directory, including meshes and the TacCap offset YAML. Motor communication still follows `motor_family`.

LeRobot defaults to `control_mode="pos_vel"`, following the position/velocity loop used by `rebotarm_control_rt`'s `ArmEndPos`. The six arm motors use the reference package's DM/RS position and velocity gains; the native CAN gripper retains MIT mode and its separate gains. DM uses POS_VEL mode 2 and CAN ID `0x100 + motor_id`. RS uses streaming CSP mode 5 with `limit_spd`/`loc_ref` (`0x7017`/`0x7016`), as specified in the [RobStride protocol](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py). The reference wrapper selected PP mode 1 while writing CSP speed parameters; this controller pairs the streaming mode and speed parameter explicitly. The native cycle remains 100 Hz and the speed limit remains 0.5 rad/s by default, rather than copying the reference's 500 Hz and higher speed limits. Set `control_mode="mit"` to use MIT instead. Direct `rebot_rt.Robot` calls retain their original default of `"mit"`; pass `control_mode="pos_vel"` explicitly when using that API.

```python
from lerobot.grippers import TaccapFollowerConfig

config = RebotB601RTConfig(
    motor_family="dm",  # or "rs"
    channel="can0",
    gripper=TaccapFollowerConfig(side="left"),
)
```

In a YAML recipe, the equivalent block is `gripper: {type: taccap_follower, side: left}`. The separate gripper driver handles TacCap commands, observations, and camera discovery. `has_gripper` refers only to a seventh motor on the arm CAN bus: it defaults to `True` with the original gripper and `False` with an external `gripper` block or an explicitly selected TacCap model. To run a bare six-motor arm, set `has_gripper=False`; that flag alone does not select a TacCap model. Explicit `urdf_path` and `end_frame` settings override the automatic model and tool frame selection. A dual arm config takes `left_arm_config` and `right_arm_config`, each with its own channel and gripper block; each arm selects its model independently, including mixed DM/RS pairs.

RS + TacCap launch recipes (run from the LeRobot repository root, with `rebot_rt` on the Python path):

```bash
# Single arm: can0, right TacCap, right Pico4 controller
lerobot-teleoperate --config_path=recipes/teleop/rebot_b601_rt/rs-taccap.yaml
# Dual arm: left can0 + left TacCap, right can1 + right TacCap, both Pico4 controllers
lerobot-teleoperate --config_path=recipes/teleop/bi_rebot_b601_rt/rs-taccap.yaml
```

Set `channel` and gripper `side` to match the hardware. These recipes enable wrist/tactile camera discovery and Rerun display. The single-arm recipe also supports `--teleop.type=spacemouse`. Startup seeds each teleoperator from the measured TCP and gripper position; it does not move the arm to a preset start pose. Dual-arm TCPs are each expressed in their own base frame; no shared-world mounting transform is applied.

The Pico4 A button returns the arm to `reset_position_rad` (six joint angles in radians). Both RS TacCap recipes set it to `[0, 0, 0, 0, 0, 0]`; a dual-arm reset uses each arm's own setting. These recipes use `max_joint_speed: 1.5` rad/s for teleoperation and `reset_joint_speed: 0.5` rad/s for reset. The reset speed is capped by `max_joint_speed`, so raising the teleoperation limit does not speed up reset. Reset runs through the native speed limiter, holds the gripper command, and keeps the reset goal alive until measured joints settle within `reset_tolerance_rad` (default 0.03 rad) for 0.1 s. `reset_timeout_sec` defaults to 30 s. The Cartesian loop stands aside during reset and re-syncs the VR target on completion. Set `reset_position_rad` explicitly on other recipes before using the reset button.

At the native API, `set_target_joints(q, max_joint_speed=0.5)` applies a lower arm ramp speed to that command. The override must be positive, finite, and no greater than the robot's configured maximum. Each call without an override, including a successful `set_target_pose`, restores the configured maximum. Overrides do not rewrite the RS motor speed registers or change the independent CAN gripper limiter.

Actions are TCP position in metres, the first two rotation matrix columns (`tcp.r1`–`tcp.r6`), and optional `gripper.pos`: radians for the original CAN motor gripper, or normalized [0, 1] (closed to open) for an external gripper. Observations use the same TCP keys unless `use_joint_observation=True`; then they use `joint_1.pos`–`joint_6.pos`. The C++ IK solves the six arm joints; the selected gripper driver controls the gripper separately.

IK projects its initial joint estimate into the URDF limits before checking convergence. A pose that cannot be solved within those limits is rejected: the LeRobot adapter holds the previous arm target, logs the measured joints and IK error at most once per second, and continues accepting subsequent targets. External gripper commands remain independent. A converging target resumes arm tracking; transport and motor faults still raise errors. A measured mechanical zero slightly below a URDF joint's zero lower bound can cause small Cartesian targets near that boundary to be rejected.

`motor_configs` accepts seven dictionaries in joint order. Each overrides the C++ defaults. Supported keys are `send_id`, `feedback_id`, `model`, `pmax`, `vmax`, `tmax`, `kp`, `kd`, `kpmax`, `kdmax`, `vel_kp`, `vel_ki`, `pos_kp`, `pos_ki`, and `direction`. `kp`/`kd` apply to MIT; the four additional gains apply to position/velocity mode (RS has no separate `pos_ki`). Positive position/velocity gains are written and read back during connection; zero leaves that firmware setting unchanged. Setting `model` selects its native protocol ranges; explicit range keys can override those values. `direction` must be `1` or `-1`. For example:

```python
config.motor_configs = [
    {"model": "4340P", "kp": 45.0, "kd": 5.0, "direction": 1.0},
    {}, {}, {}, {}, {}, {},
]
```

For direct pybind use, call `rebot_rt.default_motor_configs(family)` and then edit each `MotorConfig` before constructing `Robot`. Use `config.set_model(family, model_name)` to update the model's protocol ranges together.

## Startup and stop

1. Calibrate motor zero positions with the vendor tool and check the arm is supported mechanically. This package does not write motor zero offsets.
2. For SocketCAN, configure the interface at the motor bus bitrate, typically 1 Mbit/s, then check `ip -details link show can0`.
3. Call `connect()`. It requests all motor feedback and refuses missing or faulted motors. With motors disabled, it sets and reads back the selected control mode and position/velocity gains. It does not save parameters to flash.
4. Call `start()`. It sets and verifies the fixed RS CSP speed limit once, refreshes feedback, captures the current joint positions, seeds each position controller with that measured hold target, enables motors, and begins the 10 ms cycle. The native joint speed limiter caps movement at `max_joint_speed`.
5. Call `stop()` or `close()` to disable all motors. The context manager closes automatically.

The control loop keeps sending its last safe target when upper level commands stop for `command_timeout_sec` (default 0.5 s). It disables motors and records a fault if feedback becomes older than 100 ms, the transport fails, or a motor reports a fault. `State.cycles`, `State.overruns`, `State.fresh`, and `State.fault_reason` expose health. The 100 Hz loop is a userspace timed thread, not a hard real time scheduling guarantee.

SocketCAN sends retry temporary `ENOBUFS`, `EAGAIN`, and interrupted writes with a 1 ms backoff, for at most 20 ms per frame. This handles brief bursts exceeding a small device TX queue without dropping commands. A persistent full queue still raises an error naming the interface; permanent interface errors fail immediately. RS CSP sends only one position frame per arm joint per cycle, since its fixed speed limit and active-report configuration are established before enabling. The driver does not change the system's CAN queue length.

The six arm joints share one progress scale when a joint reaches `max_joint_speed`, preserving their joint-space direction. Position/velocity mode sends this limited position and the motor speed limit. MIT commands contain the position ramp and its matching desired velocity; a hold or watchdog timeout sends zero desired velocity. The optional CAN motor gripper is limited separately. There is no gravity feedforward in either the reference `ArmEndPos` chain or this control loop; gravity and load effects still require physical tracking measurements before gain tuning. RS velocity and torque feedback use the protocol's 16-bit fields.

To inspect remaining tracking differences during normal user-operated teleop, add `--robot.trace_joint_tracking=true` (default log rate 2 Hz). It logs `goal` (IK target), `command` (limited command), `measured` (encoder position), `error` (`command - measured`) and `velocity` (position-ramp slope, sent as desired velocity in MIT), all in joint order. A large goal-to-command difference identifies limiting; a large command-to-measured difference identifies motor tracking error. Native `State.target`, `State.command`, and `State.command_velocity` expose the same snapshots without extra bus traffic.

Hardware validation remains necessary: verify each motor's zero, positive direction, selected mode, and a small movement with the arm supported. The included offline tests use a pseudo terminal, simulated SocketCAN transport, and FK/IK; they do not prove timing or motion on a physical DM or RS arm.
