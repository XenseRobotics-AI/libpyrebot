"""Offline checks for both bundled arm models and native IK."""

import unittest
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import rebot_rt


class TestKinematics(unittest.TestCase):
    @staticmethod
    def bundled_urdf(model: str) -> Path:
        assets = Path(rebot_rt.__file__).parent / "assets"
        names = {
            "DM": "ReBot_Arm_DM.urdf",
            "RS": "ReBot_Arm_RS.urdf",
            "RS_Taccap": "rebot_b601_rs_taccap_v2_gripper.urdf",
        }
        return assets / model / "urdf" / names[model]

    def test_bundled_urdfs_have_all_meshes(self):
        for model in ("DM", "RS", "RS_Taccap"):
            with self.subTest(model=model):
                urdf = self.bundled_urdf(model)
                self.assertTrue(urdf.is_file(), urdf)
                meshes = ElementTree.parse(urdf).findall(".//mesh")
                self.assertTrue(meshes, urdf)
                for mesh in meshes:
                    filename = mesh.attrib["filename"]
                    self.assertTrue((urdf.parent / filename).is_file(), filename)

    def test_fk_ik_both_families(self):
        for family, name in ((rebot_rt.Family.DM, "DM"), (rebot_rt.Family.RS, "RS")):
            with self.subTest(family=name):
                robot = rebot_rt.Robot(family, "can0", str(self.bundled_urdf(name)))
                target = robot.forward_kinematics([0.0] * 7)
                result = robot.set_target_pose(target, -0.1)
                self.assertTrue(result.success)
                self.assertLess(result.error, 1e-4)
                self.assertLess(np.linalg.norm(robot.forward_kinematics(result.target) - target), 1e-4)

    def test_motor_config_validation(self):
        configs = rebot_rt.default_motor_configs(rebot_rt.Family.DM)
        configs[0].set_model(rebot_rt.Family.DM, "4310")
        self.assertEqual(configs[0].vmax, 30.0)
        configs[1].send_id = configs[0].send_id
        with self.assertRaisesRegex(ValueError, "duplicate"):
            rebot_rt.Robot(rebot_rt.Family.DM, "can0", str(self.bundled_urdf("DM")),
                           motor_configs=configs)

    def test_unknown_end_frame_rejected(self):
        urdf = self.bundled_urdf("DM")
        with self.assertRaisesRegex(ValueError, "unknown URDF frame"):
            rebot_rt.Robot(rebot_rt.Family.DM, "can0", str(urdf), "missing_frame")

    def test_command_speed_override_cannot_bypass_the_robot_limit(self):
        robot = rebot_rt.Robot(rebot_rt.Family.RS, "can0", str(self.bundled_urdf("RS_Taccap")),
                               has_gripper=False, max_joint_speed=1.5)
        previous = robot.get_target_joints()
        goal = [0.2, 0.1, 0.05, 0, 0, 0, 0]
        for speed in (0, -1, float("nan"), float("inf"), 1.6):
            with self.subTest(speed=speed), self.assertRaisesRegex(ValueError, "max_joint_speed"):
                robot.set_target_joints(goal, max_joint_speed=speed)
            self.assertEqual(robot.get_target_joints(), previous)
        robot.set_target_joints(goal, max_joint_speed=0.5)
        self.assertEqual(robot.get_target_joints(), goal)

    def test_taccap_export_uses_eef_frame_by_default(self):
        urdf = self.bundled_urdf("RS_Taccap")
        for family in (rebot_rt.Family.DM, rebot_rt.Family.RS):
            with self.subTest(family=family):
                implicit = rebot_rt.Robot(family, "can0", str(urdf), has_gripper=False)
                explicit = rebot_rt.Robot(family, "can0", str(urdf), "taccap_eef_link", False)
                q = [0.15, 0.2, 0.2, 0.1, 0.1, -0.1, 0.0]
                target = implicit.forward_kinematics(q)
                np.testing.assert_allclose(target, explicit.forward_kinematics(q), atol=1e-12)
                result = implicit.set_target_pose(target)
                self.assertTrue(result.success)
                self.assertLess(np.linalg.norm(implicit.forward_kinematics(result.target) - target), 2e-4)

    def test_pose_below_zero_joint_limits_is_rejected_without_changing_command(self):
        robot = rebot_rt.Robot(rebot_rt.Family.RS, "can0", str(self.bundled_urdf("RS_Taccap")),
                               has_gripper=False)
        # Recorded mechPos values: joints 2/3 slightly below the URDF's zero
        # minima. This is a constrained IK failure, not a rotation conversion bug.
        measured = [0.02117745, -0.00106528, -0.00153399, -0.00038349, -0.00786161, -0.09403303, 0.0]
        boundary = measured.copy()
        boundary[1:3] = [0.0, 0.0]
        robot.set_target_joints(boundary)
        result = robot.set_target_pose(robot.forward_kinematics(measured))
        self.assertFalse(result.success)
        np.testing.assert_allclose(robot.get_target_joints(), boundary, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
