"""Exercise the native serial transport and 100 Hz loop with a pseudo terminal."""

import os
import pty
import select
import struct
import threading
import time
import unittest
from pathlib import Path

import numpy as np
import rebot_rt


class FakeDamiaoBridge:
    def __init__(self, positions=None):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.frames = []
        self.positions = positions if positions is not None else [0.0] * 7
        self.registers = {(motor, 10): (1).to_bytes(4, "little") for motor in range(1, 8)}
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        self._thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)

    def _run(self):
        pending = bytearray()
        while self._running:
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            try:
                pending.extend(os.read(self.master, 4096))
            except OSError:
                return
            while len(pending) >= 30:
                if pending[:3] != b"\x55\xaa\x1e":
                    del pending[0]
                    continue
                packet = bytes(pending[:30])
                del pending[:30]
                can_id = int.from_bytes(packet[13:17], "little")
                self.frames.append((can_id, packet[21:29]))
                data = packet[21:29]
                motor_id = data[0] if can_id == 0x7FF else can_id & 0xFF
                if not 1 <= motor_id <= 7:
                    continue
                if can_id == 0x7FF and data[2] in (0x33, 0x55):
                    if data[2] == 0x55:
                        self.registers[motor_id, data[3]] = data[4:8]
                    value = self.registers.get((motor_id, data[3]), bytes(4))
                    self._reply(motor_id, bytes([motor_id, 0, data[2], data[3]]) + value)
                    continue
                # Position in the default DM motor's +/-12.5 rad range.
                position = int((self.positions[motor_id - 1] + 12.5) * 65535 / 25.0)
                data = bytes([0x10 | motor_id, position >> 8, position & 0xFF, 0x7F, 0xF7, 0xFF, 25, 25])
                self._reply(motor_id, data)

    def _reply(self, motor_id, data):
        reply = bytearray(16)
        reply[0:3] = b"\xaa\x11\x08"
        reply[3:7] = (0x10 + motor_id).to_bytes(4, "little")
        reply[7:15] = data
        reply[15] = 0x55
        os.write(self.master, reply)


class TestDmSerial(unittest.TestCase):
    def test_position_velocity_mode_matches_reference_gains_and_seeds_before_enable(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False,
                               control_mode="pos_vel")
        try:
            robot.connect()
            robot.start()
            robot.set_target_joints([0.2, 0.1, 0.05, 0, 0, 0, 0])
            time.sleep(0.1)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            for motor in range(1, 7):
                self.assertEqual(int.from_bytes(bridge.registers[motor, 10], "little"), 2)
                self.assertEqual(struct.unpack("<f", bridge.registers[motor, 27])[0],
                                 [150, 200, 200, 50, 50, 50][motor - 1])
                first_target = next(i for i, (mid, _) in enumerate(bridge.frames) if mid == 0x100 + motor)
                enable = next(i for i, (mid, data) in enumerate(bridge.frames)
                              if mid == motor and data == b"\xff" * 7 + b"\xfc")
                self.assertLess(first_target, enable)
                position, speed_limit = struct.unpack("<ff", bridge.frames[first_target][1])
                self.assertAlmostEqual(position, 0, delta=0.001)
                self.assertEqual(speed_limit, 0.5)
            self.assertGreater(state.command[0], 0.01)
        finally:
            robot.close()
            bridge.close()

    def test_joint_progress_is_coordinated_and_mit_velocity_matches_the_ramp(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False)
        try:
            robot.connect()
            robot.start()
            initial = np.array(robot.get_target_joints())
            goal = np.array([0.2, 0.1, 0.05, -0.08, 0.04, -0.02, 0.0])
            robot.set_target_joints(goal.tolist())
            time.sleep(0.1)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            progress = (np.array(state.command[:6]) - initial[:6]) / (goal[:6] - initial[:6])
            np.testing.assert_allclose(progress, progress[0], atol=1e-10)
            velocity = np.array(state.command_velocity[:6])
            self.assertLessEqual(np.max(np.abs(velocity)), 0.5 + 1e-10)
            self.assertGreater(np.max(np.abs(velocity)), 0.4)
            # Verify actual serial packets, not only the internal command state.
            latest = {mid: data for mid, data in bridge.frames
                      if 1 <= mid <= 6 and data[:7] != b'\xff' * 7}
            configs = rebot_rt.default_motor_configs(rebot_rt.Family.DM)
            for joint in range(6):
                data = latest[joint + 1]
                raw_velocity = (data[2] << 4) | (data[3] >> 4)
                vmax = configs[joint].vmax
                decoded_velocity = -vmax + raw_velocity * 2 * vmax / 4095
                self.assertAlmostEqual(decoded_velocity, velocity[joint], delta=2 * vmax / 4095 + 1e-6)
            robot.hold_position()
            time.sleep(0.04)
            held = robot.get_state()
            np.testing.assert_allclose(held.target, held.command, atol=1e-12)
            np.testing.assert_allclose(held.command_velocity, 0, atol=1e-12)
        finally:
            robot.close()
            bridge.close()

    def test_watchdog_holds_position_with_zero_velocity(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False,
                               command_timeout_sec=0.03)
        try:
            robot.connect()
            robot.start()
            robot.set_target_joints([0.2, 0.1, 0.05, 0, 0, 0, 0])
            time.sleep(0.12)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            self.assertLess(state.command[0], 0.2)
            np.testing.assert_allclose(state.command_velocity, 0, atol=1e-12)
            previous = state.command
            time.sleep(0.04)
            np.testing.assert_allclose(robot.get_state().command, previous, atol=1e-12)
        finally:
            robot.close()
            bridge.close()

    def test_feedback_below_zero_limits_does_not_accept_an_invalid_ik_seed(self):
        positions = [0.021, -0.0011, -0.0015, -0.0004, -0.0079, -0.094, 0.0]
        bridge = FakeDamiaoBridge(positions)
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False)
        try:
            robot.connect()  # Only fake feedback queries; no start()/enable.
            q = robot.get_state().q
            self.assertLess(q[1], 0)
            self.assertLess(q[2], 0)
            result = robot.set_target_pose(robot.forward_kinematics(q))
            self.assertFalse(result.success)
            self.assertEqual(robot.get_target_joints(), q)
        finally:
            robot.close()
            bridge.close()

    def test_connect_and_run(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets" / "DM" / "urdf" / "ReBot_Arm_DM.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf))
        try:
            robot.connect()
            self.assertTrue(all(robot.get_state().fresh))
            robot.start()
            time.sleep(0.16)
            state = robot.get_state()
            self.assertTrue(state.running)
            self.assertFalse(state.fault, state.fault_reason)
            self.assertGreaterEqual(state.cycles, 5)
            self.assertTrue(any(can_id == 0x7FF for can_id, _ in bridge.frames))
            self.assertTrue(any(can_id == 1 for can_id, _ in bridge.frames))
            robot.stop()
            self.assertFalse(robot.running)
        finally:
            robot.close()
            bridge.close()


if __name__ == "__main__":
    unittest.main()
