"""Offline gravity checks against URDF potential energy, independently of Pinocchio."""

import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import rebot_rt

URDF = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
LIMITS = [1, 8, 8, 3, 1, 1]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def potential_energy(q, gravity, urdf=URDF):
    root = ET.parse(urdf).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    children = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").attrib["link"], []).append(joint)

    def visit(name, transform):
        inertial = links[name].find("inertial")
        energy = 0.0
        if inertial is not None:
            mass = float(inertial.find("mass").attrib["value"])
            com = np.fromstring(inertial.find("origin").attrib["xyz"], sep=" ")
            energy -= mass * np.dot(gravity, transform[:3, :3] @ com + transform[:3, 3])
        for joint in children.get(name, []):
            origin = joint.find("origin")
            local = np.eye(4)
            if origin is not None:
                local[:3, 3] = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
                roll, pitch, yaw = np.fromstring(origin.attrib.get("rpy", "0 0 0"), sep=" ")
                local[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
            angle = {f"joint{i + 1}": value for i, value in enumerate(q[:6])}.get(joint.attrib["name"], 0.0)
            if joint.attrib["type"] == "revolute":
                axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
                local[:3, :3] = local[:3, :3] @ rotation(axis, angle)
            energy += visit(joint.find("child").attrib["link"], transform @ local)
        return energy

    return visit("world", np.eye(4))


class TestGravity(unittest.TestCase):
    def test_dm_identified_model_gravity_matches_energy_including_payload(self):
        urdf = Path(rebot_rt.__file__).parent / "assets/DM_TacCap00/urdf/rebot_b601_dm_taccap_v1_gripper.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, "unused", str(urdf), has_gripper=False)
        gravity = [0, 0, -9.80665]
        robot.configure_gravity_compensation(1, [1, 22, 12, 4, 2, 1], gravity=gravity,
                                             torque_offsets_nm=[0.08, 0.02, -0.06, 0.058, -0.008, -0.011])
        for q in ([0] * 7, [0, -0.8, -0.8, 0.5, 0, 0, 0], [0.4, -1.2, -0.4, -0.3, 0.5, 0.9, 0]):
            expected = []
            for delta in np.eye(7)[:6] * 1e-6:
                expected.append((potential_energy(q + delta, gravity, urdf) -
                                 potential_energy(q - delta, gravity, urdf)) / 2e-6)
            # gravity_torques stays the pure URDF prediction; offsets apply only to sent FF.
            np.testing.assert_allclose(robot.gravity_torques(q), [*expected, 0], atol=1e-6)
        self.assertFalse(robot.connected)

    def robot(self, **kwargs):
        return rebot_rt.Robot(rebot_rt.Family.RS, "unused", str(URDF), has_gripper=False, **kwargs)

    def test_gravity_matches_potential_gradient_including_fixed_gripper_links(self):
        robot = self.robot()
        for gravity in ([0, 0, -9.81], [9.81, 0, 0], [0, 0, 0]):
            robot.configure_gravity_compensation(0.5, LIMITS, gravity=gravity)
            for q in ([0, 0.8, 0.8, -0.5, 0, 0, 0], [0.3, 0.4, 0.5, -0.2, 0.7, -0.4, 0]):
                expected = []
                for i in range(6):
                    delta = np.eye(7)[i] * 1e-6
                    expected.append((potential_energy(q + delta, gravity) - potential_energy(q - delta, gravity)) / 2e-6)
                np.testing.assert_allclose(robot.gravity_torques(q), [*expected, 0], atol=1e-6)
        self.assertFalse(robot.connected)

    def test_validation_rejects_bad_compensation_before_hardware_access(self):
        robot = self.robot()
        for scale in (-1, 1.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                robot.configure_gravity_compensation(scale, LIMITS)
        for limits in ([], [1] * 5, [float("nan")] * 6, [-1] * 6, [37] * 6):
            with self.assertRaises(ValueError):
                robot.configure_gravity_compensation(1, limits)
        for rate in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                robot.configure_gravity_compensation(1, LIMITS, slew_rate_nm_s=rate)
        for offsets in ([0] * 5, [0] * 7, [float("nan")] * 6, [float("inf")] * 6):
            with self.assertRaisesRegex(ValueError, "offsets"):
                robot.configure_gravity_compensation(1, LIMITS, torque_offsets_nm=offsets)
        with self.assertRaises(ValueError):
            robot.configure_gravity_compensation(1, LIMITS, gravity=[float("nan"), 0, 0])
        with self.assertRaises(ValueError):
            robot.gravity_torques([float("nan")] * 7)
        with self.assertRaisesRegex(ValueError, "MIT"):
            self.robot(control_mode="pos_vel").configure_gravity_compensation(1, LIMITS)
        robot.configure_gravity_compensation(1, [0] * 6)  # A joint's FF can be disabled independently.
        self.assertFalse(robot.connected)
