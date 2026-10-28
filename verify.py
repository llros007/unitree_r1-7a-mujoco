"""Headless regression: Cartesian keyboard pinch and scripted physical transfer."""
import json
from unittest.mock import patch

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment
from main import KeyboardViewer


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


def verify():
    env = ArmEnvironment()
    verify_camera(env)
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
    advance(174)
    release(glfw.KEY_E)
    press(glfw.KEY_SPACE)
    advance(120)
    press(glfw.KEY_Q)
    advance(174)
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
    print(json.dumps({'camera_controls': 'passed', 'keyboard_lift_height_m': keyboard_lift, 'pause_reset': 'passed', 'transfer': result}, indent=2))


if __name__ == '__main__':
    verify()
