"""Exercise the native serial transport and 100 Hz loop with a pseudo terminal."""

import os
import pty
import select
import struct
import subprocess
import sys
import tempfile
import termios
import threading
import time
import unittest
from pathlib import Path

import numpy as np
import rebot_rt


class FakeDamiaoBridge:
    def __init__(self, positions=None, motor_ids=None, feedback_ids=None):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.frames = []
        self.positions = positions if positions is not None else [0.0] * 7
        self.motor_ids = motor_ids if motor_ids is not None else list(range(1, 8))
        self.feedback_ids = dict(zip(self.motor_ids, feedback_ids or range(0x11, 0x18)))
        self.registers = {(motor, 10): (1).to_bytes(4, "little") for motor in self.motor_ids}
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
                motor_id = (int.from_bytes(data[:2], "little") if can_id == 0x7FF else
                            next((mid for mid in self.motor_ids if can_id in (mid, 0x100 + mid)), None))
                if motor_id not in self.motor_ids:
                    continue
                if can_id == 0x7FF and data[2] in (0x33, 0x55):
                    if data[2] == 0x55:
                        self.registers[motor_id, data[3]] = data[4:8]
                    value = self.registers.get((motor_id, data[3]), bytes(4))
                    self._reply(motor_id, motor_id.to_bytes(2, "little") + data[2:4] + value)
                    continue
                # Position in the default DM motor's +/-12.5 rad range.
                position = int((self.positions[self.motor_ids.index(motor_id)] + 12.5) * 65535 / 25.0)
                data = bytes([0x10 | (motor_id & 0xF), position >> 8, position & 0xFF,
                              0x7F, 0xF7, 0xFF, 25, 25])
                self._reply(motor_id, data)

    def _reply(self, motor_id, data):
        reply = bytearray(16)
        reply[0:3] = b"\xaa\x11\x08"
        reply[3:7] = self.feedback_ids[motor_id].to_bytes(4, "little")
        reply[7:15] = data
        reply[15] = 0x55
        os.write(self.master, reply)


class TestDmSerial(unittest.TestCase):
    def test_serial_connection_resets_inherited_stop_bits(self):
        bridge = FakeDamiaoBridge()
        settings = termios.tcgetattr(bridge.slave)
        settings[2] |= termios.CSTOPB
        termios.tcsetattr(bridge.slave, termios.TCSANOW, settings)
        urdf = Path(rebot_rt.__file__).parent / "assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False)
        try:
            robot.connect()
            actual = termios.tcgetattr(bridge.slave)
            self.assertFalse(actual[2] & (termios.CSTOPB | termios.PARENB | termios.CRTSCTS))
            self.assertEqual(actual[2] & termios.CSIZE, termios.CS8)
            self.assertEqual(actual[4:6], [termios.B921600, termios.B921600])
        finally:
            robot.close()
            bridge.close()

    def test_full_standard_can_ids_are_used_for_feedback_requests(self):
        ids = list(range(0x301, 0x308))
        feedback_ids = list(range(0x51, 0x58))
        bridge = FakeDamiaoBridge(motor_ids=ids, feedback_ids=feedback_ids)
        configs = rebot_rt.default_motor_configs(rebot_rt.Family.DM)
        for config, motor_id, feedback_id in zip(configs, ids, feedback_ids):
            config.send_id = motor_id
            config.feedback_id = feedback_id
        urdf = Path(rebot_rt.__file__).parent / "assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf),
                               has_gripper=False, motor_configs=configs)
        try:
            robot.connect()
            robot.start()
            time.sleep(0.04)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            self.assertTrue(all(state.fresh[:6]))
            requested = {int.from_bytes(data[:2], "little") for mid, data in bridge.frames
                         if mid == 0x7FF and data[2] == 0xCC}
            self.assertEqual(requested, set(ids[:6]))
            self.assertFalse(any(mid == ids[6] for mid, _ in bridge.frames))
        finally:
            robot.close()
            bridge.close()

    def test_serial_write_retries_and_faults_are_bounded(self):
        script = '''
import ctypes
import os
import time
from pathlib import Path
import rebot_rt
from test_dm_serial import FakeDamiaoBridge
try:
    attempts = ctypes.CDLL(None).rebot_test_serial_write_attempts
except AttributeError as exc:
    raise RuntimeError("Serial test shim was not loaded; use Python without setuid/file capabilities") from exc
bridge = FakeDamiaoBridge()
urdf = Path(rebot_rt.__file__).parent / "assets/DM_Taccap/urdf/rebot_b601_dm_taccap.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False)
try:
    robot.connect()
    robot.start()
    before = robot.get_state().cycles
    start = time.monotonic()
    os.environ["REBOT_TEST_SERIAL_DEVICE"] = bridge.port
    mode = os.environ["REBOT_TEST_SERIAL_FAILURE"]
    while time.monotonic() - start < 1.0:
        state = robot.get_state()
        if state.fault or state.cycles >= before + 5:
            break
        time.sleep(0.005)
    if mode == "transient":
        assert attempts() >= 4, "transient/partial writes were not injected"
        assert not state.fault, state.fault_reason
        assert state.cycles >= before + 5, state.cycles
        assert len([1 for mid, _ in bridge.frames if mid == 6]) >= 5
    else:
        assert state.fault, state
        assert "serial write" in state.fault_reason, state.fault_reason
        if mode == "persistent":
            assert "20 ms" in state.fault_reason, state.fault_reason
    robot.close()
    assert time.monotonic() - start < 1.5, "serial teardown did not finish promptly"
finally:
    robot.close()
    bridge.close()
'''
        with tempfile.TemporaryDirectory() as directory:
            shim = str(Path(directory) / "serial_write_shim.so")
            subprocess.run(["cc", "-shared", "-fPIC", str(Path(__file__).with_name("serial_write_shim.c")),
                            "-o", shim, "-ldl"], check=True, capture_output=True)
            for failure in ("transient", "persistent", "zero", "disconnected"):
                with self.subTest(failure=failure):
                    env = {**os.environ, "LD_PRELOAD": shim, "REBOT_TEST_SERIAL_FAILURE": failure}
                    env.pop("REBOT_TEST_SERIAL_DEVICE", None)
                    env["PYTHONPATH"] = os.pathsep.join([
                        str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent),
                        env.get("PYTHONPATH", ""),
                    ])
                    result = subprocess.run([sys.executable, "-c", script], env=env, text=True,
                                            capture_output=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_gravity_reaches_dm_torque_field_with_direction_conversion(self):
        bridge = FakeDamiaoBridge(positions=[0, 0.8, -0.8, -0.5, 0, 0, 0])
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        configs = rebot_rt.default_motor_configs(rebot_rt.Family.DM)
        configs[2].direction = -1
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False,
                               motor_configs=configs, mit_velocity_feedforward=False)
        robot.configure_gravity_compensation(0.5, [1, 8, 1, 3, 1, 1])
        try:
            robot.connect()
            robot.start()
            time.sleep(0.4)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            np.testing.assert_allclose(state.gravity_torque, robot.gravity_torques(state.q))
            self.assertAlmostEqual(state.mit_command_torque[2], 1)
            latest = {mid: data for mid, data in bridge.frames if 1 <= mid <= 6 and data[:7] != b"\xff" * 7}
            for joint in range(6):
                data, config = latest[joint + 1], configs[joint]
                raw = ((data[6] & 0xF) << 8) | data[7]
                torque = raw * 2 * config.tmax / 4095 - config.tmax
                self.assertAlmostEqual(torque, state.mit_command_torque[joint] * config.direction,
                                       delta=2 * config.tmax / 4095)
        finally:
            robot.close()
            bridge.close()

    def test_per_joint_speed_limits_reach_dm_position_packets(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
        limits = [0.2, 0.8, 1.1, 0.9, 0.7, 0.6]
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf), has_gripper=False,
                               control_mode="pos_vel", max_joint_speeds=limits)
        try:
            robot.connect()
            robot.start()
            robot.set_target_joints([0.2] * 6 + [0])
            time.sleep(0.08)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            # All six positions share one progress scale; the slowest is J1.
            np.testing.assert_allclose(state.command_velocity[:6], 0.2, atol=1e-10)
            latest = {mid: data for mid, data in bridge.frames if 0x101 <= mid <= 0x106}
            self.assertEqual(len(latest), 6)
            for i, limit in enumerate(limits):
                _, speed = struct.unpack("<ff", latest[0x101 + i])
                self.assertAlmostEqual(speed, limit)
        finally:
            robot.close()
            bridge.close()

    def test_zero_mit_arm_velocity_keeps_position_ramp_and_gripper_independent(self):
        bridge = FakeDamiaoBridge()
        urdf = Path(rebot_rt.__file__).parent / "assets/DM/urdf/ReBot_Arm_DM.urdf"
        robot = rebot_rt.Robot(rebot_rt.Family.DM, bridge.port, str(urdf),
                               mit_velocity_feedforward=False)
        try:
            robot.connect()
            robot.start()
            robot.set_target_joints([0.2, -0.1, -0.05, 0, 0, 0, -0.4])
            time.sleep(0.1)
            state = robot.get_state()
            self.assertFalse(state.fault, state.fault_reason)
            self.assertGreater(state.command[0], 0.01)
            self.assertGreater(state.command_velocity[0], 0.4)
            np.testing.assert_allclose(state.mit_command_velocity[:6], 0, atol=1e-12)
            self.assertLess(state.mit_command_velocity[6], -0.4)
            latest = {mid: data for mid, data in bridge.frames
                      if 1 <= mid <= 7 and data[:7] != b'\xff' * 7}
            configs = rebot_rt.default_motor_configs(rebot_rt.Family.DM)
            for joint in range(7):
                data = latest[joint + 1]
                raw_velocity = (data[2] << 4) | (data[3] >> 4)
                vmax = configs[joint].vmax
                actual = -vmax + raw_velocity * 2 * vmax / 4095
                self.assertAlmostEqual(actual, state.mit_command_velocity[joint], delta=2 * vmax / 4095)
        finally:
            robot.close()
            bridge.close()

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
