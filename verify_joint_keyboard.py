"""Exercise joint keyboard callbacks against real simulation dynamics."""
import glfw
import numpy as np

from environment import ArmEnvironment
from main import KeyboardViewer


def viewer_for(env):
    # No window is needed to exercise the actual input/control path.
    viewer = KeyboardViewer.__new__(KeyboardViewer)
    viewer.env, viewer.keys = env, set()
    viewer.control_mode, viewer.selected_joint = 'joint', 0
    viewer.joint_speed, viewer.joint_status = np.deg2rad(20.), ''
    return viewer


def press(viewer, key):
    viewer.on_key(None, key, 0, glfw.PRESS, 0)


def release(viewer, key):
    viewer.on_key(None, key, 0, glfw.RELEASE, 0)


def main():
    env = ArmEnvironment(initial_pose='zero')
    v = viewer_for(env)
    press(v, glfw.KEY_4)
    assert v.selected_joint == 3
    press(v, glfw.KEY_UP)
    for _ in range(50):
        v.controls(.01)
        env.step()
    assert env.q_command[3] > np.deg2rad(5)
    assert env.data.qpos[env.qids[3]] > np.deg2rad(4)
    assert np.max(np.abs(env.q_command[np.arange(7) != 3])) < 1e-12
    release(v, glfw.KEY_UP)
    target = env.q_command.copy()
    for _ in range(100):
        v.controls(.01)
        env.step()
    assert np.array_equal(target, env.q_command), 'Release must hold the joint target'
    assert env.tool_clearance.violation(env.data) is None
    # Selecting another joint must cancel a still-held jog key.
    press(v, glfw.KEY_UP)
    press(v, glfw.KEY_2)
    assert not (v.keys & v.joint_jog_keys())
    # A negative jog at J2's zero lower limit must never produce a negative target.
    press(v, glfw.KEY_DOWN)
    for _ in range(20):
        v.controls(.01)
        env.step()
    assert env.q_command[1] == 0
    assert v.joint_status == 'Joint limit'
    release(v, glfw.KEY_DOWN)
    # Fine motion is exactly a quarter of the requested angular speed.
    press(v, glfw.KEY_4)
    before = env.q_command[3]
    press(v, glfw.KEY_LEFT_SHIFT)
    press(v, glfw.KEY_KP_ADD)
    v.controls(.01)
    assert np.isclose(env.q_command[3] - before, np.deg2rad(.05))
    press(v, glfw.KEY_P)
    before = env.q_command.copy()
    v.controls(.01)
    env.step()
    assert np.array_equal(env.q_command, before)
    press(v, glfw.KEY_R)
    assert not v.keys and env.paused and env.zero_return is None
    press(v, glfw.KEY_P)
    before_pose = env.data.qpos.copy()
    press(v, glfw.KEY_R)
    np.testing.assert_array_equal(env.data.qpos, before_pose)
    for _ in range(1000):
        v.controls(.01)
        env.step()
        if env.zero_return is None:
            break
    assert env.zero_return is None and np.array_equal(env.q_command, np.zeros(7))
    # The old XYZ keys must not move the endpoint in joint mode.
    press(v, glfw.KEY_W)
    v.controls(.01)
    assert np.array_equal(env.q_command, np.zeros(7))
    # Switch with a pending joint command: neither qpos nor the old held keys
    # may jump/carry into Cartesian control.
    press(v, glfw.KEY_4)
    press(v, glfw.KEY_UP)
    for _ in range(30):
        v.controls(.01)
        env.step()
    measured = env.data.qpos[env.qids].copy()
    position, rotation = env.tcp_position, env.data.site_xmat[env.tcp_id].copy()
    press(v, glfw.KEY_TAB)
    assert v.control_mode == 'cartesian' and not v.keys
    np.testing.assert_array_equal(env.data.qpos[env.qids], measured)
    np.testing.assert_array_equal(env.q_command, measured)
    np.testing.assert_allclose(env.target_position, position)
    np.testing.assert_allclose(env.target_rotation.reshape(-1), rotation)
    assert np.all(env.reference.dq == 0)
    target = env.q_command.copy()
    v.controls(.01)
    np.testing.assert_array_equal(env.q_command, target)
    # GLFW key repeat must not repeatedly flip modes.
    v.on_key(None, glfw.KEY_TAB, 0, glfw.REPEAT, 0)
    assert v.control_mode == 'cartesian'
    press(v, glfw.KEY_P)
    press(v, glfw.KEY_W)
    press(v, glfw.KEY_TAB)
    assert v.control_mode == 'joint' and not v.keys and env.paused
    np.testing.assert_array_equal(env.data.qpos[env.qids], measured)
    print('PASS: joint jog, limits, pause/reset and Tab switching without pose jumps or stale keys')


if __name__ == '__main__':
    main()
