"""Headless regression: Cartesian keyboard pinch and scripted physical transfer."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import glfw
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from environment import ArmEnvironment
from main import KeyboardViewer
from wrist_scene import build_scene
from motion import DOWN_ROTATION
from xle_gripper import OPEN_ANGLE, CLOSE_ANGLE


def verify_motors():
    env = ArmEnvironment()
    assert not np.any(env.model.body_gravcomp), 'Unexpected automatic gravity compensation'
    assert np.all(env.model.actuator_gear[env.arm_actuators, 0] == 1)
    assert not np.any(env.model.actuator_biasprm[env.arm_actuators])
    q = env.data.qpos[env.qids].copy()
    env.data.qvel[env.dofs] = 0.2
    env.set_joint_command(q + 0.1, dq=0.4, tau=0.3, kp=3, kd=0.4)
    expected_tau = (env.motors.tau + env.motors.kp * (env.motors.q - q)
                    + env.motors.kd * (env.motors.dq - 0.2))
    env.step(env.model.opt.timestep)
    np.testing.assert_allclose(env.data.ctrl[env.arm_actuators], expected_tau, atol=1e-12)
    np.testing.assert_allclose(env.data.actuator_force[env.arm_actuators], expected_tau, atol=1e-12)
    # Direct commands must persist across outer-loop calls; pure torque mode
    # must not accidentally retain position-servo feedback.
    env.set_joint_command(q, tau=[1e4, -1e4, 1e4, -1e4, 1e4, -1e4, 1e4], kp=0, kd=0)
    env.step()
    np.testing.assert_allclose(env.data.qfrc_actuator[env.dofs],
                               env.motors.tau_limit * [1, -1, 1, -1, 1, -1, 1])
    env.reset()
    env.set_joint_command(env.home_q, kp=0, kd=0, tau=0)
    env.step(0.2)
    drop = float(np.linalg.norm(env.data.qpos[env.qids] - env.home_q))
    assert drop > 0.01, 'Zero-torque arm should move under gravity'
    np.testing.assert_array_equal(env.data.qfrc_actuator[env.dofs], 0)
    env.reset()
    for _ in range(300):
        env.step()
    hold_error = float(np.linalg.norm(env.tcp_position - env.target_position))
    assert hold_error < 0.0005, f'Gravity feedforward failed to hold target: {hold_error:.6f} m'
    assert np.max(np.abs(env.motors.tau - env.gravity_torque(env.home_q))) < .007
    assert np.isfinite(env.data.qpos).all() and not any(w.number for w in env.data.warning)
    # With the same gains and dynamics, turning off feedforward restores sag.
    base_config = json.loads(Path(__file__).with_name('motor_config.json').read_text())
    if base_config.get('sdk_protocol_calibration'):
        base_config['sdk_protocol_calibration'] = str(Path(__file__).parent / base_config['sdk_protocol_calibration'])
    no_ff_config = base_config | {'gravity_feedforward_scale': 0.0}
    with TemporaryDirectory() as folder:
        path = Path(folder) / 'motors.json'
        path.write_text(json.dumps(no_ff_config))
        no_ff_env = ArmEnvironment(motor_config=path)
    for _ in range(300):
        no_ff_env.step()
    uncorrected_error = float(np.linalg.norm(no_ff_env.tcp_position - no_ff_env.target_position))
    assert uncorrected_error > 0.005 and uncorrected_error > 10 * hold_error
    np.testing.assert_array_equal(no_ff_env.motors.tau, 0)
    # Optional first-order motor response and reset of its state.
    config = base_config.copy()
    config['torque_time_constant_s'] = 0.02
    config['tau_limit'] = [1000] * 7  # MJCF/URDF limits still win.
    with TemporaryDirectory() as folder:
        path = Path(folder) / 'motors.json'
        path.write_text(json.dumps(config))
        lag_env = ArmEnvironment(motor_config=path)
    np.testing.assert_array_equal(lag_env.motors.tau_limit, [60, 60, 33, 33, 33, 10, 10])
    lag_env.set_joint_command(lag_env.home_q, tau=1, kp=0, kd=0)
    lag_env.step(lag_env.model.opt.timestep)
    np.testing.assert_allclose(lag_env.motors.applied_tau, lag_env.motors.tau * -np.expm1(-0.002 / 0.02))
    lag_env.reset()
    np.testing.assert_array_equal(lag_env.motors.applied_tau, 0)
    return {'five_parameter_control': 'passed', 'torque_limits': 'passed',
            'zero_torque_joint_displacement_rad': drop,
            'hold_tcp_error_m': hold_error,
            'hold_without_feedforward_error_m': uncorrected_error,
            'motor_response_lag': 'passed'}


def verify_camera(env):
    viewer = KeyboardViewer.__new__(KeyboardViewer)
    viewer.env = env
    viewer.camera = mujoco.MjvCamera()
    viewer.camera.distance = 1.35
    viewer.mouse_pos = (0., 0.)
    # Mock window input only; run the actual MuJoCo camera API without OpenGL.
    with patch.object(glfw, 'get_window_size', return_value=(1280, 900)):
        for button in [glfw.MOUSE_BUTTON_LEFT, glfw.MOUSE_BUTTON_RIGHT, glfw.MOUSE_BUTTON_MIDDLE]:
            before = np.r_[viewer.camera.lookat.copy(), viewer.camera.azimuth,
                           viewer.camera.elevation, viewer.camera.distance]
            with patch.object(glfw, 'get_mouse_button', side_effect=lambda window, b: glfw.PRESS if b == button else glfw.RELEASE):
                x, y = viewer.mouse_pos
                viewer.on_mouse(None, x + 20, y + 10)
            after = np.r_[viewer.camera.lookat, viewer.camera.azimuth,
                          viewer.camera.elevation, viewer.camera.distance]
            assert not np.array_equal(before, after), f'Camera drag failed for button {button}'
    distance = viewer.camera.distance
    viewer.on_scroll(None, 0, 1)
    assert viewer.camera.distance > distance, 'Positive scroll should increase camera distance'
    distance = viewer.camera.distance
    viewer.on_scroll(None, 0, -1)
    assert 0 < viewer.camera.distance < distance, 'Negative scroll should decrease camera distance'
    assert np.isfinite(viewer.camera.lookat).all()


def verify_stop_settling(scene_path=None):
    """Move at keyboard speed, release, then measure residual TCP oscillation."""
    env = ArmEnvironment(scene_path=scene_path)
    viewer = KeyboardViewer.__new__(KeyboardViewer)
    viewer.env, viewer.keys = env, set()
    excursions = {}
    angles = {}
    motions = [('x+', glfw.KEY_D), ('x-', glfw.KEY_A), ('y+', glfw.KEY_W),
               ('y-', glfw.KEY_S), ('z+', glfw.KEY_Q), ('z-', glfw.KEY_E),
               ('rx+', glfw.KEY_I), ('rx-', glfw.KEY_K), ('ry+', glfw.KEY_J),
               ('ry-', glfw.KEY_L), ('rz+', glfw.KEY_U), ('rz-', glfw.KEY_O)]
    for axis, key in motions:
        env.reset()
        for _ in range(300):
            env.step()
        viewer.on_key(None, key, 0, glfw.PRESS, 0)
        for _ in range(150):
            viewer.controls(.01)
            env.step()
        viewer.on_key(None, key, 0, glfw.RELEASE, 0)
        target = env.q_command.copy()
        positions = []
        rotations = []
        for _ in range(200):
            viewer.controls(.01)
            env.step()
            positions.append(env.tcp_position)
            rotations.append(env.data.site_xmat[env.tcp_id].reshape(3, 3).copy())
        # Allow 0.5 s to brake; reject persistent oscillation over the next 1.5 s.
        excursion = float(np.max(np.ptp(positions[50:], axis=0)))
        assert excursion < 0.0002, f'{axis} stop oscillation: {excursion * 1000:.3f} mm'
        angle = float(np.max(Rotation.from_matrix(np.array(rotations[50:]) @ rotations[-1].T).magnitude()))
        assert np.degrees(angle) < 0.02, f'{axis} angular stop oscillation: {np.degrees(angle):.3f} deg'
        np.testing.assert_allclose(env.q_command, target, atol=1e-10, rtol=0)
        assert not any(w.number for w in env.data.warning)
        excursions[axis] = excursion * 1000
        angles[axis] = float(np.degrees(angle))
    return {'max_axis_excursion_mm_after_0_5s': excursions,
            'max_orientation_error_deg_after_0_5s': angles}


def verify():
    motors = verify_motors()
    scene = build_scene()
    settling = verify_stop_settling(scene)
    env = ArmEnvironment(scene_path=scene)
    verify_camera(env)
    # The shared plant scene starts at the oblique RGB-D observation pose.
    env.move_to([.12, .25, .80], 2., rotation=DOWN_ROTATION)
    # Exercise the same key handlers and held-key controller used by the GUI.
    # No OpenGL context is needed for testing controls and contact dynamics.
    viewer = KeyboardViewer.__new__(KeyboardViewer)
    viewer.env, viewer.keys = env, set()

    def press(key):
        viewer.on_key(None, key, 0, glfw.PRESS, 0)

    def release(key):
        viewer.on_key(None, key, 0, glfw.RELEASE, 0)

    def advance(count):
        for _ in range(count):
            viewer.controls(.01)
            env.step()

    press(glfw.KEY_E)
    advance(178)
    release(glfw.KEY_E)
    press(glfw.KEY_SPACE)
    advance(round((OPEN_ANGLE - CLOSE_ANGLE) / env.gripper_speed / .01) + 50)
    press(glfw.KEY_Q)
    advance(178)
    release(glfw.KEY_Q)
    assert env.success, f'Keyboard pinch/lift failed: {env.cube_position}'
    assert len(env.contact_sides()) == 2, 'Expected physical contact with both fingers'
    keyboard_lift = float(env.cube_position[2])
    press(glfw.KEY_P)
    frozen_q, frozen_time = env.data.qpos.copy(), env.data.time
    advance(10)
    assert np.array_equal(env.data.qpos, frozen_q) and env.data.time == frozen_time
    press(glfw.KEY_R)
    assert env.gripper_open and not env.paused and not viewer.keys
    assert np.linalg.norm(env.tcp_position - env.home_position) < 0.001
    result = env.run_demo()
    assert result['passed'], result
    assert np.isfinite(env.data.qpos).all() and np.isfinite(env.data.qvel).all()
    assert not any(w.number for w in env.data.warning), 'MuJoCo issued simulation warnings'
    print(json.dumps({'motors': motors, 'stop_settling': settling, 'camera_controls': 'passed', 'keyboard_lift_height_m': keyboard_lift, 'pause_reset': 'passed', 'transfer': result}, indent=2))


if __name__ == '__main__':
    verify()
