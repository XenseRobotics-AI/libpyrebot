# B601 RS + TacCap05

This directory retains the existing RS v2 arm and gripper model, including its
inertials and matching `_offset.yaml`. The C1 + TacCap05 payload is about 0.692 kg.
The LeRobot adapter selects it for `motor_family: rs` with
`gripper.type: taccap_follower`, using `taccap_eef_link`.

`meshes/taccap05/` contains the existing TacCap meshes. The DM TacCap00 directory
temporarily uses identical mesh geometry, with separate identified inertials.
These copies can be replaced independently when the respective model meshes
are available. Only relative mesh paths changed during the directory rename.

RS recipe compensation settings retain their existing behavior. Identified
holding offsets remain opt-in through `gravity_use_joint_offsets: true`.
