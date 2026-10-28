"""R1-7A dynamics and Cartesian damped-least-squares controller (SI units)."""
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from xle_gripper import OPEN_ANGLE, CLOSE_ANGLE

ROOT = Path(__file__).resolve().parent


class ArmEnvironment:
    def __init__(self):
        scene = ROOT / 'assets' / 'scene.xml'
        if not scene.exists():
            from build_scene import build_scene
            scene = build_scene()
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.joints = np.array([self.model.joint(f'J{i}').id for i in range(1, 8)])
        self.qids = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.limits = self.model.jnt_range[self.joints].copy()
        self.tcp_id = self.model.site('tcp').id
        self.cube_id = self.model.body('red_cube').id
        self.cube_q = self.model.joint('cube_free').qposadr[0]
        self.mocap_id = self.model.body_mocapid[self.model.body('target').id]
        self.gripper_q = self.model.joint('gripper_hinge').qposadr[0]
        self.pad_groups = [{self.model.geom(f'{s}_pad_{i}').id for i in range(4)} for s in ['fixed', 'moving']]
        self.cube_geom = self.model.geom('cube').id
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros_like(self.jacp)
        self.home_position = np.array([0.12, 0.25, 0.80])
        self.home_rotation = np.diag([1., -1., -1.])
        self.target_position = self.home_position.copy()
        self.target_rotation = self.home_rotation.copy()
        self.q_command = np.array([0., 1.0, 1.0, 1.5, 0., 0., 0.])
        # Deterministic multi-start is used only at initialization; live IK is local.
        rng = np.random.default_rng(7)
        best, best_error = None, float('inf')
        for seed in [self.q_command] + [rng.uniform(self.limits[:, 0], self.limits[:, 1]) for _ in range(24)]:
            q, error = self.solve_ik(seed, iterations=180)
            if error < best_error:
                best, best_error = q, error
            if error < 0.0002:
                break
        if best_error > 0.01:
            raise RuntimeError(f'Initial pose is unreachable: IK residual {best_error:.4f}')
        self.home_q = best.copy()
        self.reset()

    @property
    def tcp_position(self):
        return self.data.site_xpos[self.tcp_id].copy()

    @property
    def cube_position(self):
        return self.data.xpos[self.cube_id].copy()

    def solve_ik(self, q, iterations=8):
        q = np.array(q, dtype=float, copy=True)
        error_norm = float('inf')
        for _ in range(iterations):
            self.ik_data.qpos[self.qids] = q
            mujoco.mj_forward(self.model, self.ik_data)
            pos_error = self.target_position - self.ik_data.site_xpos[self.tcp_id]
            current_rot = self.ik_data.site_xmat[self.tcp_id].reshape(3, 3)
            rot_error = Rotation.from_matrix(self.target_rotation @ current_rot.T).as_rotvec()
            error = np.r_[pos_error, 0.15 * rot_error]
            error_norm = np.linalg.norm(error)
            if error_norm < 0.0001:
                break
            mujoco.mj_jacSite(self.model, self.ik_data, self.jacp, self.jacr, self.tcp_id)
            jac = np.vstack([self.jacp[:, self.dofs], 0.15 * self.jacr[:, self.dofs]])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.002**2 * np.eye(6), error)
            dq *= min(1., 0.12 / max(np.max(np.abs(dq)), 1e-9))
            q = np.clip(q + dq, self.limits[:, 0] + 0.0005, self.limits[:, 1] - 0.0005)
        return q, error_norm

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.target_position = self.home_position.copy()
        self.target_rotation = self.home_rotation.copy()
        self.q_command = self.home_q.copy()
        self.data.qpos[self.qids] = self.home_q
        self.data.qpos[self.gripper_q] = OPEN_ANGLE
        self.gripper_open = True
        self.paused = False
        self.ik_error = 0.
        self.success = False
        self.data.ctrl[:7] = self.home_q
        self.data.ctrl[7] = OPEN_ANGLE
        self.update_marker()
        mujoco.mj_forward(self.model, self.data)

    def update_marker(self):
        self.data.mocap_pos[self.mocap_id] = self.target_position
        q = Rotation.from_matrix(self.target_rotation).as_quat()
        self.data.mocap_quat[self.mocap_id] = q[[3, 0, 1, 2]]

    def move_target(self, translation, rotation=None):
        previous_pos = self.target_position.copy()
        previous_rot = self.target_rotation.copy()
        # Keep targets over the table and within the useful workspace.
        self.target_position = np.clip(previous_pos + translation, [-0.40, -0.25, 0.622], [0.40, 0.52, 1.22])
        if rotation is not None:
            self.target_rotation = Rotation.from_rotvec(rotation).as_matrix() @ previous_rot
        q, residual = self.solve_ik(self.q_command, iterations=10)
        # Do not accumulate targets far beyond reachable Cartesian poses.
        if residual < 0.012:
            self.q_command = q
        else:
            self.target_position = previous_pos
            self.target_rotation = previous_rot
        self.ik_error = residual
        self.update_marker()

    def contact_sides(self):
        sides = set()
        for contact in self.data.contact:
            pair = {contact.geom1, contact.geom2}
            if self.cube_geom in pair:
                sides.update(i for i, pads in enumerate(self.pad_groups) if pads & pair)
        return sides

    def step(self, duration=0.01):
        if self.paused:
            return
        self.q_command, self.ik_error = self.solve_ik(self.q_command, iterations=4)
        self.data.ctrl[:7] = self.q_command
        self.data.ctrl[7] = OPEN_ANGLE if self.gripper_open else CLOSE_ANGLE
        for _ in range(max(1, round(duration / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)
        self.success = self.cube_position[2] > 0.69 and len(self.contact_sides()) == 2

    def move_to(self, position, duration=2.0):
        """Scripted Cartesian movement for reproducible physical grasp verification."""
        start = self.target_position.copy()
        for t in np.linspace(0, 1, round(duration / 0.01)):
            point = start + (3 * t*t - 2 * t*t*t) * (np.array(position) - start)
            self.move_target(point - self.target_position)
            self.step()

    def run_demo(self, observer=None):
        """Physical pinch, lift, transfer and release; never welds or teleports cube."""
        self.reset()
        for phase, point in [('approach', [0.12, 0.25, 0.72]), ('descend', [0.12, 0.25, 0.626])]:
            self.move_to(point, 2.0)
            if observer:
                observer(phase)
        self.gripper_open = False
        for _ in range(120):
            self.step()
        if observer:
            observer('close')
        self.move_to([0.12, 0.25, 0.80], 3.0)
        lift_height = float(self.cube_position[2])
        pinched = len(self.contact_sides()) == 2
        if observer:
            observer('lift')
        self.move_to([-0.12, 0.25, 0.80], 3.0)
        self.move_to([-0.12, 0.25, 0.633], 3.0)
        self.gripper_open = True
        for _ in range(80):
            self.step()
        self.move_to([-0.12, 0.25, 0.80], 2.)
        if observer:
            observer('release')
        return {'lift_height_m': lift_height, 'bilateral_contact_at_lift': pinched,
                'final_cube_position': self.cube_position.tolist(),
                'passed': bool(lift_height > 0.72 and pinched and np.linalg.norm(self.cube_position[:2] - [-0.12, 0.25]) < 0.055)}
