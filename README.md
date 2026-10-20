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

For RS set `motor_family="rs", channel="can0"` (family names are case insensitive). Without a TacCap gripper, DM uses `assets/DM/urdf/ReBot_Arm_DM.urdf` and its `end_link` frame; RS uses `assets/RS/urdf/ReBot_Arm_RS.urdf` and its `gripper_end` frame. With `gripper.type: taccap_follower`, the existing LeRobot adapter automatically selects `assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf`. For a DM arm with the same C1-mounted TacCap, explicitly set `urdf_path` to `assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf` as below. Both TacCap models use `taccap_eef_link`, but retain their own arm geometry, joint directions, and limits. See the [DM TacCap model notes](rebot_rt/assets/DM_Taccap/README.md) for mounting and the self-contained mesh layout. Motor communication still follows `motor_family`.

LeRobot defaults to `control_mode="pos_vel"`, following the position/velocity loop used by `rebotarm_control_rt`'s `ArmEndPos`. The six arm motors use the reference package's DM/RS position and velocity gains; the native CAN gripper retains MIT mode and its separate gains. DM uses POS_VEL mode 2 and CAN ID `0x100 + motor_id`. RS uses streaming CSP mode 5 with `limit_spd`/`loc_ref` (`0x7017`/`0x7016`), as specified in the [RobStride protocol](https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py). The reference wrapper selected PP mode 1 while writing CSP speed parameters; this controller pairs the streaming mode and speed parameter explicitly. The native cycle remains 100 Hz and the speed limit remains 0.5 rad/s by default, rather than copying the reference's 500 Hz and higher speed limits. Set `control_mode="mit"` to use MIT instead. Direct `rebot_rt.Robot` calls retain their original default of `"mit"`; pass `control_mode="pos_vel"` explicitly when using that API.

```python
from pathlib import Path
import rebot_rt
from lerobot.grippers import TaccapFollowerConfig

config = RebotB601RTConfig(
    motor_family="dm",  # or "rs"
    channel="can0",
    urdf_path=str(Path(rebot_rt.__file__).parent / "assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf"),
    end_frame="taccap_eef_link",
    has_gripper=False,
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

The Pico4 A button returns the arm to `reset_position_rad` (six joint angles in radians). Both RS TacCap recipes set it to `[0, 0, 0, 0, 0, 0]`; a dual-arm reset uses each arm's own setting. These recipes use `max_joint_speed: 1.5` rad/s for teleoperation and `reset_joint_speed: 0.5` rad/s for reset. Each joint's reset speed is also capped by its configured limit, so raising the teleoperation limits does not speed up reset. Reset runs through the native speed limiter, holds the gripper command, and keeps the reset goal alive until measured joints settle within `reset_tolerance_rad` (default 0.03 rad) for 0.1 s. `reset_timeout_sec` defaults to 30 s. The Cartesian loop stands aside during reset and re-syncs the VR target on completion. Set `reset_position_rad` explicitly on other recipes before using the reset button.

At the native API, `set_target_joints(q, max_joint_speed=0.5)` applies a lower arm ramp speed to that command. The override must be positive, finite, and no greater than the robot's configured maximum. Each call without an override, including a successful `set_target_pose`, restores the configured maximum. Overrides do not rewrite the RS motor speed registers or change the independent CAN gripper limiter.

For individual arm limits, set `max_joint_speeds: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]` in the robot config (or separately under `left_arm_config` and `right_arm_config`). Values are `[J1, J2, J3, J4, J5, J6]` in rad/s. The six values **replace** the arm's scalar `max_joint_speed`; omit the list to keep the scalar behavior. The native constructor accepts the same keyword as a six-element list, with an empty list selecting the scalar default. The scalar still controls the optional seventh CAN motor. Per-joint limits apply to the Cartesian IK bounds, final joint-command ramp, RS CSP speed registers, and DM position/velocity packet speed fields. A command-level scalar override uses `min(override, configured_limit[i])` on every arm joint, and cannot exceed the largest configured arm limit. These settings limit commanded trajectory speed; they do not guarantee a physical joint can track it. Select higher values only after hardware validation, and leave `MotorConfig.vmax` as the motor protocol encoding range.

`coordinate_cartesian_speed: true` (default) applies to the native Cartesian queue in both control modes. Before bounded residual minimization, it estimates the joint motion needed for the TCP sample and tries a common scale for linear translation and shortest-path rotation. Each trial is verified by bounded IK and FK. Thus a slow J1 slows the whole TCP step, instead of letting the wrist complete its part first; fixed-tip rotations retain their position within the solver tolerance when a coordinated step is feasible. Near singularities or unreachable boundaries, failed trials fall back to the existing feasible error-reducing approach, so pulling the target back inside the workspace still resumes from the last sent command. The fallback can deviate from the requested Cartesian direction. All attempts share the existing 3 ms IK budget. This is local step coordination, not acceleration/jerk planning or a guarantee that an arbitrary TCP speed is physically achievable. Set the flag to false for the previous bounded tracking behavior; per-joint limits remain active either way. Synchronous `set_target_pose()` does not use this queue feature.


Actions are TCP position in metres, the first two rotation matrix columns (`tcp.r1`–`tcp.r6`), and optional `gripper.pos`: radians for the original CAN motor gripper, or normalized [0, 1] (closed to open) for an external gripper. Observations use the same TCP keys unless `use_joint_observation=True`; then they use `joint_1.pos`–`joint_6.pos`. The C++ IK solves the six arm joints; the selected gripper driver controls the gripper separately.

The synchronous IK API projects its initial joint estimate into the URDF limits before checking convergence. With `interpolate_cartesian_targets: false`, a pose that cannot be solved within those limits is rejected: the LeRobot adapter holds the previous arm target, logs the measured joints and IK error at most once per second, and continues accepting subsequent targets. The default streaming path instead accepts bounded, error-reducing partial steps as described below. External gripper commands remain independent; transport and motor faults still raise errors. A measured mechanical zero slightly below a URDF joint's zero lower bound can make small Cartesian targets near that boundary infeasible.

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
3. Call `connect()`. For RS, it disables all configured motors, drains old feedback, and sends one type-4 clear-fault request per motor. Within 500 ms, every motor must provide fresh, fault-free, disabled status continuously for 100 ms. Delayed status flags from the old fault may settle during this startup window; a new type-21 detailed fault report remains latched and blocks startup. Missing feedback, persistent status faults, or failure to confirm disabled state also block startup. DM still requires fault-free initial feedback. With motors disabled, it sets and reads back the selected control mode and position/velocity gains. It does not save parameters to flash or change mechanical zero positions.
4. Call `start()`. It sets and verifies the fixed RS CSP speed limit once, refreshes feedback, captures the current joint positions, seeds each position controller with that measured hold target, enables motors, and begins the 10 ms cycle. The native joint speed limiter caps movement at `max_joint_speed`.
5. Call `stop()` or `close()` to disable all motors. The context manager closes automatically.

The control loop keeps sending its last safe target when upper level commands stop for `command_timeout_sec` (default 0.5 s). It disables motors and records a fault if feedback becomes older than 100 ms, the transport fails, or a motor reports a fault. `State.cycles`, `State.overruns`, `State.fresh`, and `State.fault_reason` expose health. The 100 Hz loop is a userspace timed thread, not a hard real time scheduling guarantee.

Before checking freshness or sending the next command, the worker drains already queued feedback with a 2 ms budget and no wait on an empty queue. This avoids checking last cycle's decoded timestamps before consuming packets that arrived during a scheduling delay. SocketCAN uses kernel receive timestamps (`SO_TIMESTAMPNS_NEW`, converted to monotonic age), so old queued packets do not falsely refresh a silent motor. The 100 ms cutoff and latched fault behavior remain unchanged. Timeout errors include per-motor `ages_ms`, the latest `cycle_ms`, `overruns`, and socket `rx_dropped`. `State.feedback_age_sec`, `feedback_count`, `cycle_interval_sec`, and `can_rx_dropped` expose the same diagnostics; serial feedback still uses local decode time. The timestamp API is described in the [Linux kernel documentation](https://docs.kernel.org/networking/timestamping.html).

RS feedback accepts both type-2 command replies and type-24 active reports. Both update joint state, freshness, disabled/running state and fault flags using the same payload layout. The installed RS arm was observed streaming type 24 at about 100 Hz while disabled; accepting only type 2 incorrectly made the startup clear check time out even with healthy active reports.

Runtime faults remain latched even when later feedback looks healthy; there is no automatic clear or re-enable during control. After removing the external load or obstruction, restart the application to attempt the bounded startup clear. If that fails, inspect the indicated motor and its power/driver before retrying. The two RS fault encodings differ: type-2 `status flags 2` means three-phase overcurrent, while type-21 `fault mask 2` means driver chip fault, per the [vendor protocol](https://github.com/RobStride/EDULITE_A3/blob/main/el_a3_sdk/docs/%E7%94%B5%E6%9C%BA%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E6%B1%87%E6%80%BB.md). Neither code alone establishes the physical cause.

SocketCAN sends retry temporary `ENOBUFS`, `EAGAIN`, and interrupted writes with a 1 ms backoff, for at most 20 ms per frame. This handles brief bursts exceeding a small device TX queue without dropping commands. A persistent full queue still raises an error naming the interface; permanent interface errors fail immediately. RS CSP sends only one position frame per arm joint per cycle, since its fixed speed limit and active-report configuration are established before enabling. The driver does not change the system's CAN queue length.

LeRobot enables `interpolate_cartesian_targets: true` in both `pos_vel` and `mit`. Its pose calls use `queue_target_pose()` to submit the latest waypoint without running IK on the Python producer thread. Following the `libtron2rt` Cartesian flow, the native worker measures the interval between producer calls, interpolates translation linearly and rotation by shortest-path quaternion SLERP, and solves each 100 Hz sample starting from the last limited joint command. A 30 Hz producer normally gets 3–4 native samples per waypoint; a 60 Hz producer gets 1–2. Each replacement segment begins at FK of the last limited command, so an unsent remote IK goal is not used as its start. The first waypoint and a restart after the watchdog use one 10 ms period. This is time-based interpolation, not acceleration- or jerk-limited motion.

Streaming IK searches inside the intersection of joint limits and a per-joint `speed_limit[i] * 0.01` box around the last command. Damped local steps, active-bound handling and a projected-gradient fallback reduce the sampled SE(3) log-error norm; line search checks each candidate using FK. An incomplete solution is accepted only when it reduces that error. Thus unreachable waypoints can still move the arm toward a local feasible boundary, while returning waypoints start from the last command without queuing a distant joint branch. Position and orientation jointly contribute to this residual. This is local tracking, not a guarantee of the globally nearest pose, a collision planner, or acceleration-limited motion.

Real-time IK has a 3 ms budget checked between iterations and line-search evaluations. Useful partial progress remains valid when that budget expires. With no improving step, the arm holds; unfinished interpolation samples can still be tried, and a new producer waypoint restarts tracking. Internal substeps never refresh the command watchdog. Joint commands (including A reset) and `hold_position()` cancel pending Cartesian work; a generation check discards any in-flight IK result superseded by a newer waypoint or reset. `State.cartesian_active`, `cartesian_duration_sec`, `cartesian_sequence`, `cartesian_consumed_sequence`, `cartesian_ik_valid`, `cartesian_ik_success`, `cartesian_ik_error`, and `cartesian_ik_solves` expose asynchronous progress. `cartesian_tracking_limited` means the latest sampled pose was not fully reached (including speed limits); `cartesian_tracking_progress` means a feasible error-reducing step was found. Queuing does not imply full convergence. The synchronous `set_target_pose()` API still requires full convergence; `interpolate_cartesian_targets: false` selects that legacy path in LeRobot. Builds default to Release so native FK/IK is optimized.

The six arm joints share one progress scale when a joint reaches its configured speed limit, preserving their joint-space direction. This hard per-cycle bound remains after Cartesian interpolation. Position/velocity mode sends this limited position and the motor speed limit. `mit_velocity_feedforward=true` (the native API and general robot-config default for compatibility) also sends the position ramp's finite-difference slope as MIT desired velocity. The RS TacCap single/dual recipes set it to **false**: MIT follows the same position ramp with `dq_des=0`, retaining configured Kp/Kd. This matches `libtron2rt`'s Cartesian dq behavior and removes the direct velocity feedforward pulses at waypoint boundaries; it does not add acceleration limits, gravity compensation, or guarantee hardware smoothness. A hold or watchdog timeout sends zero desired velocity with either setting. The optional CAN motor gripper is independent of this arm setting and keeps its original MIT velocity behavior. Gravity and load effects still require physical tracking measurements before gain tuning. RS velocity and torque feedback use the protocol's 16-bit fields.

To inspect remaining tracking differences during normal user-operated teleop, add `--robot.trace_joint_tracking=true` (default log rate 2 Hz). It logs `goal` (IK target), `command` (limited command), `measured` (encoder position), `error` (`command - measured`), `velocity` (position-ramp slope), and, in MIT mode, `mit_dq` (desired velocity before direction/quantization), all in joint order. With feedforward disabled, `velocity` can be nonzero while `mit_dq` stays zero. A large goal-to-command difference identifies limiting; a large command-to-measured difference identifies motor tracking error. Native `State.target`, `State.command`, `State.command_velocity`, and `State.mit_command_velocity` expose the same snapshots without extra bus traffic.

The trace also reports `speed_limits` (effective rad/s caps, including reset), `speed_utilization` (absolute command ramp velocity divided by its cap), `near_speed_limit` (joints at least 95% utilized), and `measured_velocity` (encoder feedback). `tcp_step_scale` is the accepted fraction of the current Cartesian sample, **not** the ratio to the Pico controller's raw velocity. `tcp_coordinated=false` and scale zero identify the residual-minimizing fallback or inactive/cancelled work. `tcp_bound_joint` identifies the joint bounding the initial linear estimate; either a speed or a position bound can cause it, and a nonlinear retry may reduce the step further. Use utilization and `command - measured` together to distinguish command limiting from servo lag. Native fields are `joint_speed_limits`, `joint_speed_utilization`, `joint_step_scale` (final joint-space scale), `cartesian_coordinated`, `cartesian_step_scale`, and `cartesian_limiting_joint` (zero-based index; -1 for none). They describe the latest consumed sample; check `cartesian_ik_valid` and `cartesian_active` for current status.


Hardware validation remains necessary: verify each motor's zero, positive direction, selected mode, and a small movement with the arm supported. The included offline tests use a pseudo terminal, simulated SocketCAN transport, and FK/IK; they do not prove timing or motion on a physical DM or RS arm.
