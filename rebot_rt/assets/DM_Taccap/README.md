# ReBot B601 DM with TacCap

`urdf/rebot_b601_dm_taccap.urdf` combines the DM arm with the existing C1
direct-output adapter and TacCap slave gripper. The default tool frame is
`taccap_eef_link`; camera frames are `taccap_camera_link` and
`taccap_camera_optical_link`.

## Model sources and mounting

- `../DM/urdf/ReBot_Arm_DM.urdf` supplies `base_link`, `link1`–`link5`, and all
  six arm joints, including their origins, axes, limits, efforts and velocities.
  In particular, DM joints 2 and 3 retain their negative-angle ranges.
- The original `end_link` and parallel fingers are removed. As in the RS C1
  direct-output assembly, `link6` is kept as the sixth-axis output frame, with
  its original attachment geometry and inertia removed. The sixth motor body
  remains part of `link5`.
- `../RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf` supplies the C1
  adapter, TacCap body and fingers, mimic joint, tool and camera frames, and
  fixed mounting joints. This model uses the same C1 mounting arrangement:
  - `link6_to_adapter`: `xyz="0 0 0"`, `rpy="pi/2 0 pi/2"`.
  - `adapter_to_taccap`: `xyz="0 0.014 0"`, `rpy="0 pi/2 pi/2"`.
- The resulting TCP offset in `link6` is `[-0.036575, 0, 0.2149217]` metres,
  with a 180-degree rotation about local Z.
- Arm inertials come from the DM model; adapter/gripper inertials come from
  the existing RS TacCap export. This is not a DM dynamics identification.
  The RS arm's identified inertials and torque-offset YAML are not included.

All 31 referenced STL files are bundled in this directory, preserving the
source mesh bytes. URDF mesh paths use `../meshes/...`, so copying the entire
`DM_Taccap/` directory is sufficient to load the model independently.

```text
DM_Taccap/
├── README.md
├── urdf/
│   └── rebot_b601_dm_taccap.urdf
└── meshes/
    ├── visual/           # DM arm visual parts
    ├── collision/        # DM arm collision meshes
    ├── c1_adapter/       # C1 adapter
    └── taccap_g1_slave/  # TacCap body and fingers
```

## Native use

```python
from pathlib import Path
import rebot_rt

urdf = (
    Path(rebot_rt.__file__).parent
    / "assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf"
)
robot = rebot_rt.Robot(
    rebot_rt.Family.DM,
    "can0",  # or a DM serial bridge, e.g. /dev/ttyACM0
    str(urdf),
    end_frame="taccap_eef_link",
    has_gripper=False,
)
```

`has_gripper=False` disables the seventh native CAN motor. TacCap uses its
separate driver. The native arm model locks the two TacCap finger joints at
their neutral positions for six-joint FK/IK; the full URDF retains their
motion and mimic relation for visualization.

For LeRobot, set `motor_family="dm"`, this explicit `urdf_path`,
`end_frame="taccap_eef_link"`, and `gripper=TaccapFollowerConfig(...)`.
The existing adapter's automatic TacCap selection still points to `RS_Taccap`,
so the DM URDF path must be supplied explicitly.
