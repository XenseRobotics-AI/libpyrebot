"""Offline FK/IK example for either bundled B601 model."""

import argparse
from pathlib import Path

import numpy as np
import rebot_rt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--family", choices=("dm", "rs"), default="dm")
    args = parser.parse_args()
    family = rebot_rt.Family.DM if args.family == "dm" else rebot_rt.Family.RS
    assets = Path(rebot_rt.__file__).parent / "assets"
    urdf = (assets / "DM" / "urdf" / "ReBot_Arm_DM.urdf" if args.family == "dm"
            else assets / "RS_Taccap" / "urdf" / "rebot_b601_rs_taccap_v2_gripper.urdf")
    robot = rebot_rt.Robot(family, "can0", str(urdf))
    target = robot.forward_kinematics([0.0] * 7)
    result = robot.set_target_pose(target, 0.0)
    print(f"IK success={result.success}, error={result.error:.6g}, iterations={result.iterations}")
    print(f"FK pose error={np.linalg.norm(robot.forward_kinematics(result.target) - target):.6g}")


if __name__ == "__main__":
    main()
