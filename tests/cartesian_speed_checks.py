"""Shared FK-based assertions for the simulated DM and RS control loops."""

import time

import numpy as np
import pinocchio as pin


def check_cartesian_speed_limits(robot, linear_limit, angular_limit):
    initial = robot.get_state()
    origin = robot.forward_kinematics(initial.command)
    displacement = np.array([0.012, -0.006, 0.004])
    rotation = np.array([0.06, -0.04, 0.08])
    goal = origin.copy()
    goal[:3, 3] += displacement
    goal[:3, :3] = origin[:3, :3] @ pin.exp3(rotation)
    previous, previous_pose = initial, origin
    peak_linear = peak_angular = 0.0
    capped_steps = 0
    # Large first target (10 ms interpolation), then repeated 30 Hz targets.
    # This exercises catch-up as well as interpolation, with unequal joint caps.
    robot.queue_target_pose(goal)
    publish = time.monotonic() + 0.1
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if time.monotonic() >= publish:
            robot.queue_target_pose(goal)
            publish = time.monotonic() + 1 / 30
        state = robot.get_state()
        assert not state.fault, state.fault_reason
        if state.cycles != previous.cycles:
            dt = (state.cycles - previous.cycles) * 0.01
            pose = robot.forward_kinematics(state.command)
            linear = np.linalg.norm(pose[:3, 3] - previous_pose[:3, 3]) / dt
            angular = np.linalg.norm(pin.log3(previous_pose[:3, :3].T @ pose[:3, :3])) / dt
            assert linear <= linear_limit + 1e-6, (linear, linear_limit)
            assert angular <= angular_limit + 1e-6, (angular, angular_limit)
            limits = np.array(state.joint_speed_limits[:6])
            assert np.all(np.abs(state.command_velocity[:6]) <= limits + 1e-9)
            assert np.all(np.abs(np.array(state.command[:6]) - previous.command[:6]) <= limits * dt + 1e-9)
            delta = pose[:3, 3] - origin[:3, 3]
            progress = np.dot(delta, displacement) / np.dot(displacement, displacement)
            # Translation and rotation advance by the same fraction, so joints
            # cannot finish the wrist motion ahead of the translation.
            assert np.linalg.norm(delta - progress * displacement) < 2e-5
            assert np.linalg.norm(pin.log3(origin[:3, :3].T @ pose[:3, :3]) - progress * rotation) < 2e-4
            if state.cartesian_speed_scale < 0.99:
                capped_steps += 1
                assert state.cartesian_tracking_limited
                assert state.cartesian_active  # capped sample != final endpoint
            peak_linear, peak_angular = max(peak_linear, linear), max(peak_angular, angular)
            previous, previous_pose = state, pose
            if np.linalg.norm(pose - goal) < 2e-5:
                break
        time.sleep(0.001)
    np.testing.assert_allclose(previous_pose, goal, atol=2e-5)
    assert capped_steps > 10, capped_steps
    # Demonstrate motion, not just a trivial hold that happens to satisfy caps.
    assert max(peak_linear / linear_limit, peak_angular / angular_limit) > 0.8

    # A target that takes longer than the watchdog must stop without a producer.
    robot.queue_target_pose(origin)
    time.sleep(0.4)
    stopped = robot.get_state()
    assert not stopped.fault, stopped.fault_reason
    assert not stopped.cartesian_active
    np.testing.assert_allclose(stopped.command_velocity, 0)
    assert np.linalg.norm(robot.forward_kinematics(stopped.command) - origin) > 1e-4
    time.sleep(0.04)
    np.testing.assert_allclose(robot.get_state().command, stopped.command, atol=1e-12)

    # Joint-space reset remains governed by its own joint speed limit.
    reset = np.array(stopped.command)
    reset[0] += 0.03
    robot.set_target_joints(reset, max_joint_speed=0.2)
    time.sleep(0.22)
    np.testing.assert_allclose(robot.get_state().command, reset, atol=1e-10)
    assert robot.get_state().cartesian_speed_scale == 1.0
