"""Run the actual native RS state machine against simulated motors, without CAN."""

import os
import select
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import rebot_rt

FRAME = struct.Struct("=IB3x8s")


class TestRsSocketCan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="rebot-can-test-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.shim = Path(cls.temp.name) / "can.so"
        subprocess.run(["cc", "-shared", "-fPIC", str(Path(__file__).with_name("socketcan_shim.c")),
                        "-o", str(cls.shim), "-ldl"], check=True, capture_output=True)

    def run_native(self, fail_setting=None, tx_failure=None, script=None):
        path = str(Path(self.temp.name) / "bus")
        if os.path.exists(path):
            os.unlink(path)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.addCleanup(server.close)
        server.bind(path)
        server.listen(1)
        server.settimeout(5)
        frames, registers, errors = [], {}, []

        def motors():
            try:
                connection, _ = server.accept()
                with connection:
                    next_status = time.monotonic()

                    def send(kind, motor, data):
                        connection.sendall(FRAME.pack(0x80000000 | kind << 24 | motor << 8 | 0xFD, 8, data))

                    def status(motor):
                        vmax, tmax = (50, 36) if motor <= 3 else (33, 14)
                        v = round((2 + vmax) * 65535 / (2 * vmax))
                        t = round((3 + tmax) * 65535 / (2 * tmax))
                        send(2, motor, struct.pack(">HHHH", 32767, v, t, 250))

                    while True:
                        if select.select([connection], [], [], 0.002)[0]:
                            packet = connection.recv(16)
                            if not packet:
                                break
                            arb, _, data = FRAME.unpack(packet)
                            kind, motor = (arb >> 24) & 0x1F, arb & 0xFF
                            frames.append((kind, motor, data))
                            if kind in (17, 18):
                                setting = int.from_bytes(data[:2], "little")
                                if kind == 18:
                                    registers[motor, setting] = data[4:8]
                                else:
                                    value = registers.get((motor, setting), bytes(4))
                                    if setting == fail_setting:
                                        value = bytes(4)
                                    # An unrelated register reply must not satisfy the pending read.
                                    send(17, motor, b"\x19\x70\x00\x00" + bytes(4))
                                    send(17, motor, data[:4] + value)
                            status(motor)
                        if time.monotonic() >= next_status:
                            for motor in range(1, 7):
                                status(motor)
                            next_status = time.monotonic() + 0.01
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=motors, daemon=True)
        thread.start()
        env = {**os.environ, "LD_PRELOAD": str(self.shim), "REBOT_TEST_CAN_SOCKET": path}
        env.pop("REBOT_TEST_CAN_FAILURE", None)
        if tx_failure is not None:
            env["REBOT_TEST_CAN_FAILURE"] = tx_failure
        package = Path(rebot_rt.__file__).resolve().parent.parent
        env["PYTHONPATH"] = str(package)
        default_script = '''
import time
import ctypes
from pathlib import Path
import numpy as np
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf),
                       has_gripper=False, control_mode="pos_vel", command_timeout_sec=0.04)
try:
    robot.connect()
    robot.start()
    robot.set_target_joints([0.2, 0.1, 0.05, 0, 0, 0, 0])
    time.sleep(0.16)
    state = robot.get_state()
    assert not state.fault, state.fault_reason
    assert state.cycles >= 5, state.cycles
    np.testing.assert_allclose(state.dq[:6], 2, atol=0.002)
    np.testing.assert_allclose(state.tau[:6], 3, atol=0.002)
    np.testing.assert_allclose(state.command_velocity, 0, atol=1e-12)
    assert 0.005 < state.command[0] < 0.2, state.command
finally:
    robot.close()
    print("TX_FAILURES", ctypes.CDLL(None).rebot_test_failed_writes())
'''
        result = subprocess.run([sys.executable, "-c", script or default_script], env=env, text=True,
                                capture_output=True, timeout=15)
        thread.join(timeout=5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors, errors)
        return result, frames, registers

    def test_streaming_uses_csp_gains_measured_start_and_watchdog_hold(self):
        result, frames, registers = self.run_native()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        first_enable = next(i for i, (kind, _, _) in enumerate(frames) if kind == 3)
        for motor in range(1, 7):
            self.assertEqual(registers[motor, 0x7005][0], 5)
            self.assertEqual(struct.unpack("<f", registers[motor, 0x701E])[0],
                             [13, 16, 14, 20, 10, 10][motor - 1])
            seed = next(data for kind, mid, data in frames[:first_enable]
                        if kind == 18 and mid == motor and data[:2] == b"\x16\x70")
            self.assertAlmostEqual(struct.unpack("<f", seed[4:])[0], 0, delta=0.001)
            self.assertEqual(struct.unpack("<f", registers[motor, 0x7017])[0], 0.5)
            speed_writes = [i for i, (kind, mid, data) in enumerate(frames)
                            if kind == 18 and mid == motor and data[:2] == b"\x17\x70"]
            self.assertEqual(len(speed_writes), 1)
            self.assertLess(speed_writes[0], first_enable)
        self.assertFalse(any(kind == 1 for kind, _, _ in frames))
        self.assertFalse(any(kind == 18 and data[:2] == b"\x05\x70"
                             for kind, _, data in frames[first_enable:]))
        self.assertFalse(any(kind == 24 for kind, _, _ in frames[first_enable:]))

    def test_slow_reset_then_cartesian_teleop_restores_full_speed(self):
        result, frames, registers = self.run_native(script='''
import time
from pathlib import Path
import numpy as np
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_Taccap/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf),
                       has_gripper=False, control_mode="pos_vel", max_joint_speed=1.5)
def wait_for(check):
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        state = robot.get_state()
        assert not state.fault, state.fault_reason
        if check(state):
            return state
        time.sleep(0.005)
    raise AssertionError((state.command, state.command_velocity))
try:
    robot.connect()
    robot.start()
    robot.set_target_joints([1.2, 0.6, 0.3, 0, 0, 0, 0])
    fast = wait_for(lambda s: s.command[0] > 0.2)
    np.testing.assert_allclose(fast.command_velocity[0], 1.5, atol=1e-12)
    # A-button reset and its repeated watchdog refresh stay at 0.5 rad/s.
    for _ in range(3):
        robot.set_target_joints([0] * 7, max_joint_speed=0.5)
        slow = wait_for(lambda s: s.command_velocity[0] < 0)
        np.testing.assert_allclose(slow.command_velocity[0], -0.5, atol=1e-12)
        time.sleep(0.02)
    # A subsequent Cartesian command automatically restores the normal limit.
    result = robot.set_target_pose(robot.forward_kinematics([0] * 7))
    assert result.success, result.error
    resumed = wait_for(lambda s: s.command_velocity[0] < -1)
    np.testing.assert_allclose(resumed.command_velocity[0], -1.5, atol=1e-12)
    assert resumed.command[0] < slow.command[0]
finally:
    robot.close()
''')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for motor in range(1, 7):
            self.assertEqual(struct.unpack("<f", registers[motor, 0x7017])[0], 1.5)
            speed_writes = [data for kind, mid, data in frames
                            if kind == 18 and mid == motor and data[:2] == b"\x17\x70"]
            self.assertEqual(len(speed_writes), 1)
            positions = [struct.unpack("<f", data[4:])[0] for kind, mid, data in frames
                         if kind == 18 and mid == motor and data[:2] == b"\x16\x70"]
            self.assertTrue(all(abs(b - a) <= 0.015001 for a, b in zip(positions, positions[1:])))

    def test_transient_queue_full_recovers_during_startup_and_control(self):
        for stage in ("startup", "runtime"):
            with self.subTest(stage=stage):
                result, frames, _ = self.run_native(tx_failure=stage)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("TX_FAILURES 3", result.stdout)
                first_enable = next(i for i, (kind, _, _) in enumerate(frames) if kind == 3)
                # Retrying the blocked seed must neither drop nor duplicate it.
                for motor in range(1, 7):
                    seeds = [data for kind, mid, data in frames[:first_enable]
                             if kind == 18 and mid == motor and data[:2] == b"\x16\x70"]
                    self.assertEqual(len(seeds), 1)

    def test_persistent_full_queue_fails_with_a_bounded_wait_before_enable(self):
        result, frames, _ = self.run_native(tx_failure="persistent")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("did not drain within 20 ms", result.stderr)
        self.assertFalse(any(kind == 3 for kind, _, _ in frames))
        failed = int(result.stdout.split("TX_FAILURES ")[1].split()[0])
        self.assertGreater(failed, 1)
        self.assertLess(failed, 100)  # A backoff, not a busy retry loop.

    def test_permanent_interface_error_is_not_retried(self):
        result, frames, _ = self.run_native(tx_failure="down")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Network is down", result.stderr)
        self.assertIn("TX_FAILURES 1", result.stdout)
        self.assertFalse(any(kind == 3 for kind, _, _ in frames))

    def test_speed_limit_readback_mismatch_prevents_enable(self):
        result, frames, _ = self.run_native(fail_setting=0x7017)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("speed limit readback mismatch", result.stderr)
        self.assertFalse(any(kind == 3 for kind, _, _ in frames))

    def test_mode_readback_mismatch_prevents_enable(self):
        result, frames, _ = self.run_native(fail_setting=0x7005)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("control mode readback mismatch", result.stderr)
        self.assertFalse(any(kind == 3 for kind, _, _ in frames))

    def test_gain_readback_mismatch_prevents_enable(self):
        result, frames, _ = self.run_native(fail_setting=0x701E)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("gain readback mismatch", result.stderr)
        self.assertFalse(any(kind == 3 for kind, _, _ in frames))


if __name__ == "__main__":
    unittest.main()
