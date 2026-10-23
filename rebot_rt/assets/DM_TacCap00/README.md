# B601 DM + TacCap00

This package uses the exported `rebot_b601_dm_taccap_v1_gripper` model
(2026-10-23), including the identified bare-arm inertials, C1 + TacCap00 payload,
and matching joint holding-torque offsets. `source.json` records its source hashes.
The payload in this export is **0.9839121914 kg**, including C1. Mass and COM
are preserved exactly from the export.

The identified payload is distributed over `adapter_link` and the three TacCap
links. Do not add it to `link6` again. The runtime locks finger joints at neutral,
merging their inertia into the six-axis arm model. Static identification determines
gravity terms; the exported inertia tensor still uses the CAD prior.

## Files and selection

- `urdf/rebot_b601_dm_taccap_v1_gripper.urdf`: complete visual/dynamics model.
- `urdf/rebot_b601_dm_taccap_v1_gripper_offset.yaml`: holding torque biases,
  ordered by `joint1` through `joint6` when loaded by the LeRobot adapter.
- `meshes/rebot_b601_dm/`: DM arm geometry from the export.
- `meshes/c1_adapter/`: C1 adapter geometry.
- `meshes/taccap00/`: temporary shared TacCap geometry, currently identical to
  `RS_TacCap05/meshes/taccap05/`. Replace this directory when TacCap00 meshes are ready.
- `LICENSES/`: source notices supplied by the identification export.

The LeRobot adapter selects this model for `motor_family: dm` with
`gripper.type: taccap_follower`, and uses `taccap_eef_link`. No explicit `urdf_path`
or `end_frame` is needed. Direct native API calls still supply the URDF path.

The DM teleop/record recipes use measured joint angles to evaluate
`scale * (g(q) + offset)`, with gravity `[0, 0, -9.80665]`, then apply per-joint
FF caps and a 5 N m/s slew limit. Offsets are loaded only when
`gravity_use_joint_offsets: true`; a missing or malformed matching file fails
before the arm connection. `gravity_torques()` and `State.gravity_torque` return
pure URDF gravity, while `State.mit_command_torque` includes the applied bias.
Startup, user reset, exit reset and watchdog hold all use the same compensation.

The model is specific to the identified arm/tool assembly. Encoder calibration,
tracking residuals and behavior on another arm require hardware verification.
