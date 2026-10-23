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
    CARTESIAN_SCRIPT = '''
import os
import time
from pathlib import Path
import numpy as np
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf),
                       has_gripper=False, control_mode=os.environ["REBOT_TEST_CONTROL_MODE"],
                       mit_velocity_feedforward=os.environ.get("REBOT_TEST_MIT_FF", "1") == "1",
                       max_joint_speed=1.5, command_timeout_sec=0.3)
def wait_for(check, timeout=0.6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = robot.get_state()
        assert not state.fault, state.fault_reason
        if check(state):
            return state
        time.sleep(0.001)
    raise AssertionError((state.command, state.cartesian_active, state.cartesian_ik_error))
robot.connect()
robot.start()
try:
    seed = np.array(robot.get_state().command)
    sequence = robot.queue_target_pose(robot.forward_kinematics(seed))
    wait_for(lambda s: s.cartesian_consumed_sequence == sequence and not s.cartesian_active)
    BODY
finally:
    robot.close()
'''

    def run_cartesian(self, body, mode, feedforward=True, speeds=None, initial_q=None, coordinated=True):
        return self.run_native(
            script=self.CARTESIAN_SCRIPT.replace("    BODY", body).replace(
                "command_timeout_sec=0.3)",
                f"command_timeout_sec=0.3, max_joint_speeds={speeds or []!r}, "
                f"coordinate_cartesian_speed={coordinated!r})"), control_mode=mode,
            initial_q=initial_q or [0.15, 0.6, 0.5, -0.25, 0.2, 0.1],
            active_report_type=24, reply_status=False,
            feedforward=feedforward,
        )

    def test_cartesian_mit_dq_can_be_zero_with_the_position_ramp_still_moving(self):
        for mode in ("mit", "pos_vel"):
            for feedforward in (False, True):
                with self.subTest(mode=mode, feedforward=feedforward):
                    result, frames, _ = self.run_cartesian('''
    time.sleep(0.065)
    q_goal = seed.copy()
    q_goal[1] += 0.04
    goal = robot.forward_kinematics(q_goal)
    sequence = robot.queue_target_pose(goal)
    moving = wait_for(lambda s: s.command_velocity[1] > 0.1)
    if os.environ["REBOT_TEST_CONTROL_MODE"] == "pos_vel" or os.environ["REBOT_TEST_MIT_FF"] == "0":
        np.testing.assert_allclose(moving.mit_command_velocity[:6], 0, atol=1e-12)
    else:
        np.testing.assert_allclose(moving.mit_command_velocity[:6], moving.command_velocity[:6])
    done = wait_for(lambda s: s.cartesian_consumed_sequence == sequence and not s.cartesian_active)
    assert done.cartesian_ik_success, done.cartesian_ik_error
    np.testing.assert_allclose(robot.forward_kinematics(done.command), goal, atol=0.0002)
    time.sleep(0.03)
    np.testing.assert_allclose(robot.get_state().mit_command_velocity, 0, atol=1e-12)
''', mode, feedforward=feedforward)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    mit = [data for kind, motor, data in frames if kind == 1 and motor == 2]
                    if mode == "pos_vel":
                        self.assertFalse(mit)
                        self.assertTrue(any(kind == 18 and data[:2] == b'\x16\x70'
                                            for kind, _, data in frames))
                    else:
                        self.assertGreater(len(mit), 5)
                        dq = [-50 + int.from_bytes(data[2:4], "big") * 100 / 65535 for data in mit]
                        if feedforward:
                            self.assertGreater(max(dq), 0.1)
                        else:
                            self.assertTrue(all(abs(v) <= 100 / 65535 for v in dq), dq)
                        # Turning off dq must not remove MIT stiffness or damping.
                        for data in mit:
                            self.assertAlmostEqual(int.from_bytes(data[4:6], "big") * 5000 / 65535,
                                                   150, delta=5000 / 65535)
                            self.assertAlmostEqual(int.from_bytes(data[6:8], "big") * 100 / 65535,
                                                   10, delta=100 / 65535)

    def test_cartesian_time_interpolation_in_mit_and_position_modes(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, _ = self.run_cartesian('''
    # A small reachable pose change used to be sent in one 10 ms burst.
    time.sleep(0.065)
    q_goal = seed.copy()
    q_goal[0] += 0.008
    goal = robot.forward_kinematics(q_goal)
    before = robot.get_state().cartesian_ik_solves
    sequence = robot.queue_target_pose(goal)
    duration = robot.get_state().cartesian_duration_sec
    assert 0.06 <= duration < 0.13, duration
    samples = []
    def finished(s):
        samples.append((s.cycles, s.command, s.command_velocity))
        return s.cartesian_consumed_sequence == sequence and not s.cartesian_active
    done = wait_for(finished)
    assert done.cartesian_ik_success, done.cartesian_ik_error
    assert done.cartesian_ik_solves - before >= 5
    assert len({round(q[0], 5) for _, q, _ in samples}) >= 4
    assert max(abs(v[0]) for _, _, v in samples) < 0.4, samples
    np.testing.assert_allclose(robot.forward_kinematics(done.command), goal, atol=0.0002)
    # Faster producer updates reduce the segment duration without a new config.
    robot.queue_target_pose(goal)
    time.sleep(0.017)
    robot.queue_target_pose(goal)
    fast = robot.get_state().cartesian_duration_sec
    assert 0.01 <= fast < duration, (fast, duration)
''', mode, feedforward=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_cartesian_reset_hold_and_watchdog_cancel_pending_motion(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, _ = self.run_cartesian('''
    far = seed.copy()
    far[0] += 0.8
    robot.queue_target_pose(robot.forward_kinematics(far))
    wait_for(lambda s: s.command[0] > seed[0] + 0.03)
    # An A-style joint reset cancels even an unfinished Cartesian segment.
    robot.set_target_joints([0] * 7, max_joint_speed=0.5)
    state = wait_for(lambda s: s.command_velocity[0] < 0)
    assert not state.cartesian_active
    assert max(abs(v) for v in state.command_velocity[:6]) <= 0.5 + 1e-10
    np.testing.assert_allclose(state.mit_command_velocity[:6], 0, atol=1e-12)
    solves = state.cartesian_ik_solves
    time.sleep(0.04)
    assert robot.get_state().cartesian_ik_solves == solves
    robot.hold_position()
    held = robot.get_state().command
    time.sleep(0.05)
    np.testing.assert_allclose(robot.get_state().command, held, atol=1e-12)
    # Internal IK samples must not refresh the producer's watchdog.
    robot.queue_target_pose(robot.forward_kinematics(far))
    wait_for(lambda s: s.cartesian_ik_valid)
    time.sleep(0.36)
    stopped = robot.get_state()
    assert not stopped.cartesian_active
    np.testing.assert_allclose(stopped.command_velocity, 0, atol=1e-12)
    np.testing.assert_allclose(stopped.mit_command_velocity, 0, atol=1e-12)
    time.sleep(0.04)
    np.testing.assert_allclose(robot.get_state().command, stopped.command, atol=1e-12)
    # No stale arrival interval after a timeout.
    robot.queue_target_pose(robot.forward_kinematics(stopped.command))
    assert robot.get_state().cartesian_duration_sec == 0.01
''', mode, feedforward=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_unreachable_pose_is_approached_and_reentry_remains_continuous(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, _ = self.run_cartesian('''
    import pinocchio as pin
    lower = np.array([-2.8, 0, 0, -1.57, -1.57, -3.14])
    upper = np.array([2.8, 3.14, 3.14, 1.57, 1.57, 3.14])
    previous = robot.get_state()
    saw_partial_progress = False
    def stream(goal, seconds):
        global previous, saw_partial_progress
        deadline = time.monotonic() + seconds
        publish = 0
        errors = []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= publish:
                robot.queue_target_pose(goal)
                publish = now + 1 / 30
            s = robot.get_state()
            assert not s.fault, s.fault_reason
            if s.cycles != previous.cycles:
                q = np.array(s.command[:6])
                assert np.all(q >= lower - 1e-12), q
                assert np.all(q <= upper + 1e-12), q
                assert np.max(np.abs(s.command_velocity[:6])) <= 1.5 + 1e-9
                allowed = (s.cycles - previous.cycles) * 0.015 + 1e-9
                assert np.max(np.abs(q - previous.command[:6])) <= allowed
                # No distant IK branch is queued behind the limited command.
                assert np.max(np.abs(np.array(s.target[:6]) - q)) < 0.016
                saw_partial_progress |= s.cartesian_tracking_limited and s.cartesian_tracking_progress
                actual = robot.forward_kinematics(s.command)
                errors.append(np.linalg.norm(pin.log6(pin.SE3(actual).inverse() * pin.SE3(goal)).vector))
                previous = s
            time.sleep(0.001)
        return errors
    unreachable = robot.forward_kinematics(seed)
    unreachable[0, 3] += 0.8
    outside_errors = stream(unreachable, 3.0)
    assert saw_partial_progress
    assert outside_errors[-1] < outside_errors[0] - 0.02, outside_errors
    assert np.max(np.diff(outside_errors)) < 0.0003, outside_errors
    assert np.max(np.abs(np.array(previous.command[:6]) - seed[:6])) > 0.05
    # Continued outward requests settle at a local feasible boundary instead
    # of oscillating or accumulating an ever more distant joint target.
    assert np.ptp(outside_errors[-20:]) < 0.0003, outside_errors[-20:]
    assert np.max(np.abs(previous.command_velocity[:6])) < 0.03, previous.command_velocity
    # Pull the controller back to a reachable pose; no re-enable, clutch reset,
    # random IK restart, or replay of the outside waypoint is needed.
    inside = seed.copy()
    inside[0] += 0.03
    returning = robot.forward_kinematics(inside)
    return_errors = stream(returning, 3.0)
    assert return_errors[-1] < 0.0003, return_errors[-10:]
    np.testing.assert_allclose(robot.forward_kinematics(previous.command), returning, atol=0.0003)
    assert not previous.cartesian_tracking_limited
    # Partial IK progress must not keep itself alive after the producer stops.
    robot.queue_target_pose(unreachable)
    wait_for(lambda s: s.cartesian_tracking_limited and s.cartesian_tracking_progress)
    time.sleep(0.36)
    stopped = robot.get_state()
    assert not stopped.cartesian_active
    np.testing.assert_allclose(stopped.command_velocity, 0, atol=1e-12)
    time.sleep(0.04)
    np.testing.assert_allclose(robot.get_state().command, stopped.command, atol=1e-12)
''', mode, feedforward=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_tcp_x_rotation_keeps_tip_fixed_with_unequal_joint_limits(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, registers = self.run_cartesian('''
    import pinocchio as pin
    limits = np.array([0.3, 1.5, 1.5, 1.5, 0.6, 1.5])
    origin = robot.forward_kinematics(seed)
    goal = origin.copy()
    goal[:3, :3] = origin[:3, :3] @ pin.exp3(np.array([0.12, 0., 0.]))
    deadline = time.monotonic() + 1.6
    previous = robot.get_state()
    publish = 0
    angles, offsets, off_axis = [], [], []
    limited = set()
    while time.monotonic() < deadline:
        if time.monotonic() >= publish:
            robot.queue_target_pose(goal)
            publish = time.monotonic() + 1 / 30
        s = robot.get_state()
        assert not s.fault, s.fault_reason
        if s.cycles != previous.cycles:
            np.testing.assert_allclose(s.joint_speed_limits[:6], limits)
            assert np.all(np.abs(s.command_velocity[:6]) <= limits + 1e-9)
            assert np.all(np.abs(np.array(s.command[:6]) - previous.command[:6]) <=
                          limits * 0.01 * (s.cycles - previous.cycles) + 1e-9)
            pose = robot.forward_kinematics(s.command)
            rotation = pin.log3(origin[:3, :3].T @ pose[:3, :3])
            angles.append(rotation[0])
            offsets.append(np.linalg.norm(pose[:3, 3] - origin[:3, 3]))
            off_axis.append(np.linalg.norm(rotation[1:]))
            if s.cartesian_ik_valid and s.cartesian_tracking_progress:
                assert s.cartesian_coordinated
                if s.cartesian_step_scale < 0.99:
                    limited.add(s.cartesian_limiting_joint)
            previous = s
        time.sleep(0.001)
    assert 0 in limited, limited  # J1, not the wrist, bounds this motion
    assert max(offsets) < 2e-5, max(offsets)
    assert max(off_axis) < 2e-5, max(off_axis)
    assert min(np.diff(angles)) > -1e-5, angles
    assert abs(angles[-1] - 0.12) < 0.0002, angles[-1]
''', mode, feedforward=False, speeds=[0.3, 1.5, 1.5, 1.5, 0.6, 1.5],
                    initial_q=[0.2193, 0.4063, 0.3952, -0.0846, 0.0668, -0.1274])
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                if mode == "pos_vel":
                    for motor, limit in enumerate([0.3, 1.5, 1.5, 1.5, 0.6, 1.5], 1):
                        self.assertAlmostEqual(struct.unpack("<f", registers[motor, 0x7017])[0], limit)

    def test_per_joint_limits_apply_to_joint_moves_reset_and_resume(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, _ = self.run_cartesian('''
    limits = np.array([0.2, 0.8, 1.1, 0.9, 0.7, 0.6])
    goal = seed.copy()
    goal[:6] += 0.1
    robot.set_target_joints(goal)
    s = wait_for(lambda s: s.command_velocity[0] > 0.19)
    np.testing.assert_allclose(s.command_velocity[:6], 0.2, atol=1e-10)
    np.testing.assert_allclose(s.joint_speed_utilization[:6], 0.2 / limits)
    # A scalar reset cap must not increase an individual joint's lower limit.
    robot.set_target_joints([0] * 7, max_joint_speed=0.5)
    s = wait_for(lambda s: s.command_velocity[0] < 0)
    np.testing.assert_allclose(s.joint_speed_limits[:6], np.minimum(limits, 0.5))
    assert np.all(np.abs(s.command_velocity[:6]) <= np.minimum(limits, 0.5) + 1e-10)
    robot.queue_target_pose(robot.forward_kinematics(goal))
    wait_for(lambda s: s.cartesian_tracking_progress)
    np.testing.assert_allclose(robot.get_state().joint_speed_limits[:6], limits)
    robot.hold_position()
    s = wait_for(lambda s: max(np.abs(s.command_velocity)) == 0)
    assert not s.cartesian_coordinated
    assert s.cartesian_step_scale == 0
    assert s.cartesian_limiting_joint == -1
''', mode, feedforward=False, speeds=[0.2, 0.8, 1.1, 0.9, 0.7, 0.6])
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="rebot-can-test-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.shim = Path(cls.temp.name) / "can.so"
        subprocess.run(["cc", "-shared", "-fPIC", str(Path(__file__).with_name("socketcan_shim.c")),
                        "-o", str(cls.shim), "-ldl"], check=True, capture_output=True)

    def run_native(self, fail_setting=None, tx_failure=None, script=None,
                   fault_case=None, control_mode="pos_vel", active_report_type=2, reply_status=True,
                   initial_q=None, feedforward=True, torque_frames=None):
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
                    cleared_at = None
                    fault_reported = False
                    runtime_fault_sent = False
                    enabled = set()
                    all_enabled_at = None

                    def send(kind, motor, data, status_bits=0):
                        connection.sendall(FRAME.pack(
                            0x80000000 | kind << 24 | status_bits << 16 | motor << 8 | 0xFD, 8, data))

                    def status(motor, kind=2):
                        nonlocal fault_reported
                        if motor == 4 and fault_case == "silent_runtime" and len(enabled) == 6:
                            return
                        if motor == 4 and fault_case == "stale_queue" and all_enabled_at is not None \
                                and time.monotonic() - all_enabled_at > 0.06:
                            return
                        flags = 0
                        mode = 2 if motor in enabled else 0
                        if motor == 6:
                            if fault_case == "silent_after_clear" and cleared_at is not None:
                                return
                            if fault_case in ("latched", "persistent_status"):
                                if fault_case == "persistent_status" or cleared_at is None or \
                                        time.monotonic() - cleared_at < 0.03:
                                    flags = 2
                            if fault_case == "not_disabled" and cleared_at is not None:
                                mode = 2
                            if not fault_reported and (
                                (fault_case == "latched" and cleared_at is None) or
                                (fault_case == "persistent_report" and cleared_at is not None)
                            ):
                                send(21, motor, struct.pack("<II", 2, 0))
                                fault_reported = True
                        vmax, tmax = (50, 36) if motor <= 3 else (33, 14)
                        v = round((2 + vmax) * 65535 / (2 * vmax))
                        t = round((3 + tmax) * 65535 / (2 * tmax))
                        p = round(((initial_q[motor - 1] if initial_q is not None else 0) + 12.5) * 65535 / 25)
                        if motor == 6 and fault_case == "flapping_status" and cleared_at is not None:
                            send(kind, motor, struct.pack(">HHHH", 32767, v, t, 250), 2)
                        send(kind, motor, struct.pack(">HHHH", p, v, t, 250), flags | mode << 6)

                    while True:
                        if select.select([connection], [], [], 0.002)[0]:
                            packet = connection.recv(16)
                            if not packet:
                                break
                            arb, _, data = FRAME.unpack(packet)
                            kind, motor = (arb >> 24) & 0x1F, arb & 0xFF
                            frames.append((kind, motor, data))
                            if kind == 1 and torque_frames is not None:
                                torque_frames.append((motor, (arb >> 8) & 0xFFFF))
                            if kind == 3:
                                enabled.add(motor)
                                if len(enabled) == 6:
                                    all_enabled_at = time.monotonic()
                            elif kind == 4:
                                enabled.discard(motor)
                                if motor == 6 and data[0] == 1:
                                    cleared_at = time.monotonic()
                            is_target = kind == 1 or (kind == 18 and data[:2] == b"\x16\x70")
                            if fault_case == "runtime" and is_target and len(enabled) == 6 \
                                    and not runtime_fault_sent:
                                # A single fault followed immediately by normal frames
                                # must still stop the running controller.
                                send(active_report_type, 6, struct.pack(">HHHH", 32767, 32767, 32767, 250),
                                     2 | 2 << 6)
                                runtime_fault_sent = True
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
                            if reply_status:
                                status(motor)
                        if time.monotonic() >= next_status:
                            for motor in range(1, 7):
                                status(motor, active_report_type)
                            next_status = time.monotonic() + 0.01
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=motors, daemon=True)
        thread.start()
        env = {**os.environ, "LD_PRELOAD": str(self.shim), "REBOT_TEST_CAN_SOCKET": path}
        env["REBOT_TEST_CONTROL_MODE"] = control_mode
        env["REBOT_TEST_MIT_FF"] = "1" if feedforward else "0"
        env.pop("REBOT_TEST_CAN_FAILURE", None)
        if tx_failure is not None:
            env["REBOT_TEST_CAN_FAILURE"] = tx_failure
        package = Path(rebot_rt.__file__).resolve().parent.parent
        env["PYTHONPATH"] = str(package)
        default_script = '''
import time
import ctypes
import os
from pathlib import Path
import numpy as np
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf),
                       has_gripper=False, control_mode=os.environ["REBOT_TEST_CONTROL_MODE"],
                       command_timeout_sec=0.04)
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
        self.assertFalse(errors, f"{errors}\n{result.stdout}\n{result.stderr}")
        return result, frames, registers

    def test_gravity_feedforward_encoding_limits_slew_and_watchdog_hold(self):
        for direction, scale in ((1, 0), (1, 0.5), (-1, 0.5)):
            with self.subTest(direction=direction, scale=scale):
                torques = []
                script = '''
import time
from pathlib import Path
import numpy as np
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
configs = rebot_rt.default_motor_configs(rebot_rt.Family.RS)
configs[2].direction = DIRECTION
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf), has_gripper=False,
                       motor_configs=configs, mit_velocity_feedforward=False, command_timeout_sec=0.1)
scale = SCALE
if scale:
    robot.configure_gravity_compensation(scale, [1, 8, 1, 3, 1, 1], slew_rate_nm_s=5)
try:
    robot.connect()
    try:
        robot.configure_gravity_compensation(1, [1] * 6)
    except RuntimeError:
        pass
    else:
        raise AssertionError("live reconfiguration was accepted")
    robot.start()
    q = np.array(robot.get_state().q)
    robot.set_target_joints(q + [0, 0, 0.2, 0, 0, 0, 0])
    time.sleep(0.5)  # The fake encoders stay fixed; the command moves then times out.
    state = robot.get_state()
    assert not state.fault, state.fault_reason
    assert state.command[2] > state.q[2] + 0.01
    np.testing.assert_allclose(state.command_velocity, 0)
    np.testing.assert_allclose(state.mit_command_velocity, 0)
    raw = robot.gravity_torques(q) if scale else np.zeros(7)
    np.testing.assert_allclose(state.gravity_torque, raw)
    np.testing.assert_allclose(state.mit_command_torque, np.clip(scale * np.array(raw), -np.array([1,8,1,3,1,1,0]), [1,8,1,3,1,1,0]), atol=1e-6)
    assert state.gravity_torque_limited[2] == bool(scale)
    robot.hold_position()
    time.sleep(0.03)
    np.testing.assert_allclose(robot.get_state().mit_command_torque, state.mit_command_torque)
    robot.stop()
    robot.start()
    time.sleep(0.015)
    assert abs(robot.get_state().mit_command_torque[2]) < 0.3  # Restart resets the ramp.
finally:
    robot.close()
'''.replace("DIRECTION", str(direction)).replace("SCALE", str(scale))
                result, frames, _ = self.run_native(
                    script=script, control_mode="mit", torque_frames=torques,
                    initial_q=[0, 0.8, 0.8 * direction, -0.5, 0, 0])
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                first_stop = next(i for i, (kind, _, data) in enumerate(frames)
                                  if i > next(j for j, (k, _, _) in enumerate(frames) if k == 3)
                                  and kind == 4 and data[0] == 0)
                n = sum(kind == 1 for kind, _, _ in frames[:first_stop])
                j3 = [-36 + value * 72 / 65535 for mid, value in torques[:n] if mid == 3]
                self.assertGreater(len(j3), 10)
                self.assertLessEqual(abs(j3[0]), 0.052)
                self.assertTrue(all(abs(b - a) <= 0.052 for a, b in zip(j3, j3[1:])))
                self.assertAlmostEqual(j3[-1], direction if scale else 0, delta=0.002)
                for kind, mid, data in frames:
                    if kind == 1 and mid == 3:
                        self.assertAlmostEqual(int.from_bytes(data[4:6], "big") * 5000 / 65535, 150, delta=0.08)
                        self.assertAlmostEqual(int.from_bytes(data[6:8], "big") * 100 / 65535, 10, delta=0.002)

    def test_gravity_compensation_preserves_fault_and_stale_feedback_shutdown(self):
        for fault in ("runtime", "silent_runtime"):
            with self.subTest(fault=fault):
                result, frames, _ = self.run_native(control_mode="mit", fault_case=fault, script='''
import time
from pathlib import Path
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf), has_gripper=False)
robot.configure_gravity_compensation(0.5, [1, 8, 8, 3, 1, 1])
try:
    robot.connect()
    robot.start()
    time.sleep(0.25)
    state = robot.get_state()
    assert state.fault and not state.running, state.fault_reason
    assert "overcurrent" in state.fault_reason or "stale feedback" in state.fault_reason
finally:
    robot.close()
''')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(sum(kind == 3 for kind, _, _ in frames), 6)
                self.assertTrue(all(kind == 4 for kind, _, _ in frames[-6:]))

    def run_feedback_pause(self, mode, silent=False):
        # Only the child controller is paused; the parent fake motor keeps
        # sending. This reproduces feedback waiting unread after a late wakeup.
        return self.run_native(control_mode=mode, active_report_type=24, reply_status=False,
                               fault_case="stale_queue" if silent else None, script='''
import os, signal, subprocess, sys, time
from pathlib import Path
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf), has_gripper=False,
                       control_mode=os.environ["REBOT_TEST_CONTROL_MODE"], mit_velocity_feedforward=False)
try:
    robot.connect()
    robot.start()
    time.sleep(0.04)
    resume = subprocess.Popen([sys.executable, "-c", "import os, signal, time; time.sleep(0.20); os.kill(" + str(os.getpid()) + ", signal.SIGCONT)"])
    os.kill(os.getpid(), signal.SIGSTOP)
    resume.wait(timeout=2)
    time.sleep(0.04)
    state = robot.get_state()
    EXPECTATION
finally:
    robot.close()
'''.replace('    EXPECTATION', '''
    assert state.fault and not state.running, state.fault_reason
    assert "stale feedback from motor 4" in state.fault_reason, state.fault_reason
    assert "ages_ms=[" in state.fault_reason and "rx_dropped=" in state.fault_reason
    assert state.feedback_age_sec[3] > 0.1, state.feedback_age_sec
''' if silent else '''
    assert not state.fault and state.running, state.fault_reason
    assert all(state.fresh[:6]), state.feedback_age_sec
    assert all(count > 10 for count in state.feedback_count[:6]), state.feedback_count
    assert state.can_rx_dropped == 0
'''))

    def test_queued_recent_feedback_is_read_before_timeout_after_scheduling_pause(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, _, _ = self.run_feedback_pause(mode)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_old_queued_frames_cannot_refresh_a_silent_motor(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, frames, _ = self.run_feedback_pause(mode, silent=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(sum(kind == 3 for kind, _, _ in frames), 6)
                self.assertTrue(all(kind == 4 for kind, _, _ in frames[-6:]))

    def test_missing_motor_feedback_still_disables_without_auto_reenable(self):
        result, frames, _ = self.run_native(fault_case="silent_runtime", control_mode="mit", script='''
import time
from pathlib import Path
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf), has_gripper=False)
try:
    robot.connect()
    robot.start()
    time.sleep(0.16)
    state = robot.get_state()
    assert state.fault and not state.running, state.fault_reason
    assert "stale feedback from motor 4" in state.fault_reason, state.fault_reason
finally:
    robot.close()
''')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(sum(kind == 3 for kind, _, _ in frames), 6)
        self.assertTrue(all(kind == 4 for kind, _, _ in frames[-6:]))

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
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
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

    def test_startup_clears_old_fault_and_waits_for_disabled_healthy_feedback(self):
        for mode in ("mit", "pos_vel"):
            with self.subTest(mode=mode):
                result, frames, _ = self.run_native(fault_case="latched", control_mode=mode)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                first_enable = next(i for i, (kind, _, _) in enumerate(frames) if kind == 3)
                for motor in range(1, 7):
                    clears = [i for i, (kind, mid, data) in enumerate(frames)
                              if kind == 4 and mid == motor and data[0] == 1]
                    self.assertEqual(len(clears), 1)
                    self.assertLess(clears[0], first_enable)
                    self.assertTrue(any(kind == 4 and mid == motor and data[0] == 0
                                        for kind, mid, data in frames[:clears[0]]))
                self.assertFalse(any(kind in (6, 22) for kind, _, _ in frames))

    def test_persistent_status_fault_prevents_enable(self):
        result, frames, _ = self.run_native(fault_case="persistent_status", control_mode="mit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RS motor 6 reported status flags 2 (three-phase overcurrent)", result.stderr)
        self.assertIn("startup fault clear failed", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_type24_reports_keep_feedback_fresh_during_startup_and_control(self):
        # Real hardware replies to commands with type 2, but streams type 24.
        # Also cover firmware providing only the active-report stream.
        for mode in ("mit", "pos_vel"):
            for reply_status in (True, False):
                with self.subTest(mode=mode, reply_status=reply_status):
                    result, _, _ = self.run_native(
                        control_mode=mode, active_report_type=24, reply_status=reply_status)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_type24_persistent_fault_prevents_enable(self):
        result, frames, _ = self.run_native(fault_case="persistent_status", control_mode="mit",
                                            active_report_type=24, reply_status=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RS motor 6 reported status flags 2 (three-phase overcurrent)", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_new_detailed_fault_is_not_erased_by_healthy_status_frames(self):
        result, frames, _ = self.run_native(fault_case="persistent_report", control_mode="mit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RS motor 6 reported fault mask 2 (driver chip fault)", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_intermittent_fault_with_healthy_frames_does_not_pass_startup(self):
        result, frames, _ = self.run_native(fault_case="flapping_status", control_mode="mit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("startup fault clear failed", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_silent_motor_after_clear_prevents_enable(self):
        result, frames, _ = self.run_native(fault_case="silent_after_clear", control_mode="mit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RS motor 6 missing fresh feedback after fault clear", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_motor_must_confirm_disabled_before_enable(self):
        result, frames, _ = self.run_native(fault_case="not_disabled", control_mode="mit")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("RS motor 6 did not confirm disabled state", result.stderr)
        self.assertFalse(any(kind in (1, 3) for kind, _, _ in frames))

    def test_runtime_fault_stays_latched_without_clear_or_automatic_reenable(self):
        for mode, report_type in (("mit", 2), ("pos_vel", 2), ("mit", 24), ("pos_vel", 24)):
            with self.subTest(mode=mode, report_type=report_type):
                result, frames, _ = self.run_native(fault_case="runtime", control_mode=mode,
                                                     active_report_type=report_type, script='''
import os
import time
from pathlib import Path
import rebot_rt
urdf = Path(rebot_rt.__file__).parent / "assets/RS_TacCap05/urdf/rebot_b601_rs_taccap_v2_gripper.urdf"
robot = rebot_rt.Robot(rebot_rt.Family.RS, "_rebot_test_can", str(urdf),
                       has_gripper=False, control_mode=os.environ["REBOT_TEST_CONTROL_MODE"])
try:
    robot.connect()
    robot.start()
    deadline = time.monotonic() + 0.5
    while not robot.get_state().fault and time.monotonic() < deadline:
        time.sleep(0.005)
    time.sleep(0.03)
    state = robot.get_state()
    assert state.fault and not state.running, state.fault_reason
    assert "RS motor 6 reported status flags 2 (three-phase overcurrent)" in state.fault_reason
finally:
    robot.close()
''')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                first_enable = next(i for i, (kind, _, _) in enumerate(frames) if kind == 3)
                self.assertEqual(sum(kind == 3 for kind, _, _ in frames), 6)
                self.assertFalse(any(kind == 4 and data[0] == 1
                                     for kind, _, data in frames[first_enable:]))
                disabled = {mid for kind, mid, data in frames[first_enable:] if kind == 4 and data[0] == 0}
                self.assertEqual(disabled, set(range(1, 7)))

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
