"""Check radial Link7 flange mounting and tool/arm collision regression."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from end_effector_collision import ToolArmClearance
from environment import ArmEnvironment
from xle_gripper import OPEN_ANGLE, CLOSE_ANGLE


def main():
    for camera in (True, False):
        env = ArmEnvironment(initial_pose='zero', include_camera=camera)
        m, d = env.model, env.data
        assert env.tool_clearance.pairs, 'Tool/arm contact pairs must be present'
        assert np.array_equal(d.qpos[env.qids], np.zeros(7))
        assert env.tool_clearance.violation(d) is None
        # Tool forward must align with the radial flange normal, NOT J7 axis.
        hand = m.body('hand').id
        link7 = m.body('Link7').id
        R7 = d.xmat[link7].reshape(3, 3)
        Rh = d.xmat[hand].reshape(3, 3)
        assert np.allclose(R7.T @ Rh[:, 2], [0, -1, 0], atol=1e-7)
        assert np.allclose(R7.T @ (d.xpos[hand] - d.xpos[link7]), [0, -.061, 0], atol=1e-7)
        for jaw in np.linspace(CLOSE_ANGLE, OPEN_ANGLE, 21):
            d.qpos[env.gripper_q] = jaw
            mujoco.mj_forward(m, d)
            assert env.tool_clearance.violation(d) is None
        # Regression: the previous axial +Z attachment really interferes.
        tree = ET.parse(env.scene_path)
        root = tree.getroot()
        root.find('compiler').set('meshdir', str(env.scene_path.parent / 'meshes'))
        for texture in root.findall('asset/texture'):
            if texture.get('file'):
                texture.set('file', str((env.scene_path.parent / texture.get('file')).resolve()))
        old_hand = root.find(".//body[@name='hand']")
        old_hand.attrib.pop('xyaxes')
        old_hand.set('pos', '0 0 0.04')
        legacy = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        legacy_data = mujoco.MjData(legacy)
        legacy_data.qpos[legacy.joint('gripper_hinge').qposadr[0]] = OPEN_ANGLE
        mujoco.mj_forward(legacy, legacy_data)
        guard = ToolArmClearance(legacy)
        assert guard.violation(legacy_data) is not None
        checked = {tuple(sorted(pair)) for pair in guard.pairs}
        assert any(tuple(sorted((c.geom1, c.geom2))) in checked and c.dist < 0 for c in legacy_data.contact)
        env.reset()
        assert np.array_equal(env.q_command, np.zeros(7))
        for _ in range(200):
            env.step()
            assert env.tool_clearance.violation(d) is None
        assert np.max(np.abs(d.qpos[env.qids])) < .002
        def observe(e):
            assert e.tool_clearance.violation(e.data) is None
        env.step_observer = observe
        env.move_to_observation()
        for _ in range(100):
            env.step()
        assert np.linalg.norm(env.tcp_position - env.home_position) < .003
    print('PASS: radial flange/TCP axes, legacy interference, jaw sweep, zero hold/reset and unfolding (camera on/off)')


if __name__ == '__main__':
    main()
