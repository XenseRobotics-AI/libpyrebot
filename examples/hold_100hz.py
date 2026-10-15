"""Hold the current motor positions with the native 100 Hz loop."""

import argparse
import time
from pathlib import Path

import rebot_rt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--family", choices=("dm", "rs"), required=True)
    parser.add_argument("--channel", required=True, help="DM: /dev/ttyACM*, RS: can0")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    family = rebot_rt.Family.DM if args.family == "dm" else rebot_rt.Family.RS
    assets = Path(rebot_rt.__file__).parent / "assets"
    urdf = (assets / "DM" / "urdf" / "ReBot_Arm_DM.urdf" if args.family == "dm"
            else assets / "RS_Taccap" / "urdf" / "rebot_b601_rs_taccap_v2_gripper.urdf")
    with rebot_rt.Robot(family, args.channel, str(urdf)) as robot:
        time.sleep(args.seconds)
        state = robot.get_state()
        print(f"cycles={state.cycles}, overruns={state.overruns}, fault={state.fault}")
        if state.fault:
            raise RuntimeError(state.fault_reason)


if __name__ == "__main__":
    main()
