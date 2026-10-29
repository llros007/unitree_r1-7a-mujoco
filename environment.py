"""R1-7A dynamics and Cartesian damped-least-squares controller (SI units)."""
from pathlib import Path
import copy
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from xle_gripper import OPEN_ANGLE, CLOSE_ANGLE
from motor_control import ArmMotorController, JointReference
from motion import DOWN_ROTATION, PickPlace, ScriptedGrasp

ROOT = Path(__file__).resolve().parent


class ArmEnvironment:
    def __init__(self, motor_config=None, scene_path=None, object_kind='plant',
                 end_effector_config=None, include_camera=True):
        if scene_path is None:
            from wrist_scene import build_scene as build_shared_scene
            scene_path = build_shared_scene(end_effector_config, object_kind, include_camera)
        scene = Path(scene_path)
        if not scene.exists():
            raise FileNotFoundError(scene)
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.scene_path = scene.resolve()
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.joints = np.array([self.model.joint(f'J{i}').id for i in range(1, 8)])
        self.qids = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.arm_actuators = np.array([self.model.actuator(f'J{i}_motor').id for i in range(1, 8)])
        self.gripper_actuator = self.model.actuator('gripper_servo').id
        self.motors = ArmMotorController(self.model, self.joints, self.arm_actuators, motor_config)
        self.gripper_speed = self.motors.gripper_speed
        # Refresh inertia/contact constants after applying configurable armature.
        mujoco.mj_setConst(self.model, self.data)
        self.limits = self.model.jnt_range[self.joints].copy()
        self.reference = JointReference(self.motors.reference_time_constant, self.limits)
        self.tcp_id = self.model.site('tcp').id
        self.cube_id = self.model.body('red_cube').id
        self.cube_q = self.model.joint('cube_free').qposadr[0]
        self.object_kind = ('plant' if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_MESH,
                                                      'potted_plant_04_pot') >= 0 else 'cube')
        if self.object_kind == 'plant':
            from yolo_grasp.plant_asset import POT_HEIGHT
            self.object_half_height = POT_HEIGHT / 2
        else:
            self.object_half_height = .02
        table = self.model.geom('table')
        self.table_height = float(table.pos[2] + table.size[2])
        self.object_rest_height = self.table_height + self.object_half_height
        self.mocap_id = self.model.body_mocapid[self.model.body('target').id]
        self.gripper_q = self.model.joint('gripper_hinge').qposadr[0]
        # Separate zero-velocity state: qfrc_bias is gravity at this pose,
        # without Coriolis or contact forces from the live simulation.
        self.gravity_model = copy.copy(self.model)
        self.gravity_data = mujoco.MjData(self.gravity_model)
        self.step_observer = None
        self.pad_groups = [{self.model.geom(f'{s}_pad_{i}').id for i in range(4)} for s in ['fixed', 'moving']]
        self.cube_geom = self.model.geom('cube').id
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros_like(self.jacp)
        self.home_position = np.array([.12, .08, .83]) if self.object_kind == 'plant' else np.array([.12, .25, .80])
        self.home_rotation = (Rotation.from_euler('x', 40, degrees=True).as_matrix() @ DOWN_ROTATION
                              if self.object_kind == 'plant' else DOWN_ROTATION.copy())
        self.target_position = self.home_position.copy()
        self.target_rotation = self.home_rotation.copy()
        self.q_command = np.array([0., 1.0, 1.0, 1.5, 0., 0., 0.])
        # Deterministic multi-start is used only at initialization; live IK is local.
        rng = np.random.default_rng(7)
        best, best_error, best_margin = None, float('inf'), -float('inf')
        for seed in [self.q_command] + [rng.uniform(self.limits[:, 0], self.limits[:, 1]) for _ in range(24)]:
            q, error = self.solve_ik(seed, iterations=180)
            margin = float(np.min(np.minimum(q - self.limits[:, 0], self.limits[:, 1] - q)))
            if ((error < .0002 and (best_error >= .0002 or margin > best_margin))
                    or (best_error >= .0002 and error < best_error)):
                best, best_error = q, error
                best_margin = margin
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

    @property
    def object_position(self):
        return self.cube_position

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
        self.gripper_command = OPEN_ANGLE
        self.paused = False
        self.ik_error = 0.
        self.success = False
        self.cartesian_control = True
        self._command_countdown = 0
        self.episode = getattr(self, 'episode', -1) + 1
        self.motors.reset(self.home_q)
        self.reference.reset(self.home_q)
        self.data.ctrl[self.gripper_actuator] = OPEN_ANGLE
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

    def object_supported(self):
        """Tactile simulation signal: pot/cube has contacted the tabletop."""
        table = self.model.geom('table').id
        return any({c.geom1, c.geom2} == {self.cube_geom, table} for c in self.data.contact)

    def set_joint_command(self, q, dq=0, tau=0, kp=None, kd=None):
        """Hold SDK-style joint-side commands until replaced or reset (SI units).

        Switches off Cartesian IK so step() cannot overwrite this command.
        tau is feedforward torque; kp/kd default to motor_config.json.
        """
        self.motors.set_command(q, dq, tau, kp, kd)
        self.cartesian_control = False

    def use_cartesian_control(self):
        """Resume keyboard/IK control from the current arm position."""
        self.q_command = self.data.qpos[self.qids].copy()
        self.target_position = self.tcp_position
        self.target_rotation = self.data.site_xmat[self.tcp_id].reshape(3, 3).copy()
        self.motors.reset(self.q_command)
        self.reference.reset(self.q_command)
        self.cartesian_control = True
        self._command_countdown = 0
        self.update_marker()

    def gravity_torque(self, q):
        """Joint-side gravity torque for a seven-joint target pose (Nm)."""
        self.gravity_data.qpos[self.qids] = q
        self.gravity_data.qpos[self.gripper_q] = self.data.qpos[self.gripper_q]
        mujoco.mj_forward(self.gravity_model, self.gravity_data)
        return self.gravity_data.qfrc_bias[self.dofs].copy()

    def step(self, duration=0.01):
        if self.paused:
            return
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError('duration must be finite and positive')
        # IK is solved only when move_target() changes the desired pose.
        # Re-solving a fixed pose can move a redundant arm after key release.
        gripper_target = OPEN_ANGLE if self.gripper_open else CLOSE_ANGLE
        gripper_speed = getattr(self, 'gripper_speed', None)
        for _ in range(max(1, round(duration / self.model.opt.timestep))):
            if gripper_speed is None:
                self.gripper_command = gripper_target
            else:
                increment = gripper_speed * self.model.opt.timestep
                self.gripper_command += np.clip(gripper_target - self.gripper_command, -increment, increment)
            self.data.ctrl[self.gripper_actuator] = self.gripper_command
            if self.cartesian_control and self._command_countdown == 0:
                q, dq = self.reference.update(self.q_command, self.motors.command_period)
                gravity_tau = (self.motors.gravity_feedforward_scale * self.gravity_torque(q)
                               if self.motors.gravity_feedforward_scale else 0.)
                self.motors.set_command(q, dq, tau=gravity_tau)
                self._command_countdown = self.motors.command_ticks
            # Firmware-like inner PD runs at physics rate, holding the outer target.
            self.motors.update(self.data, self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)
            self._command_countdown = max(0, self._command_countdown - 1)
        mujoco.mj_forward(self.model, self.data)
        self.success = self.object_position[2] > self.object_rest_height + .07 and len(self.contact_sides()) == 2
        if self.step_observer is not None:
            self.step_observer(self)

    def move_to(self, position, duration=2.0, rotation=None):
        """Scripted Cartesian movement for reproducible physical grasp verification."""
        start = self.target_position.copy()
        start_rotation = self.target_rotation.copy()
        rotvec = Rotation.from_matrix(rotation @ start_rotation.T).as_rotvec() if rotation is not None else None
        for t in np.linspace(0, 1, round(duration / 0.01)):
            blend = 3 * t*t - 2 * t*t*t
            point = start + blend * (np.array(position) - start)
            delta = None
            if rotvec is not None:
                desired = Rotation.from_rotvec(blend * rotvec).as_matrix() @ start_rotation
                delta = Rotation.from_matrix(desired @ self.target_rotation.T).as_rotvec()
            self.move_target(point - self.target_position, delta)
            self.step()

    def run_demo(self, observer=None):
        """Known-fixture demo using the same contact controller as YOLO grasping."""
        self.reset()
        surface = np.r_[self.model.qpos0[self.cube_q:self.cube_q + 2],
                        self.table_height + 2 * self.object_half_height]
        tcp = surface.copy()
        tcp[2] = max(.622, surface[2] - (.02 if self.object_kind == 'plant' else .014))
        controller = PickPlace(self)
        controller.start(ScriptedGrasp(surface, tcp, DOWN_ROTATION.copy()))
        budget = round((sum(action[2] for action in controller.actions) + 20) / .01)
        previous = controller.phase
        for _ in range(budget):
            controller.tick()
            self.step()
            if controller.phase != previous:
                if observer:
                    observer(previous)
                previous = controller.phase
            if not controller.active:
                break
        result = dict(controller.report)
        result.setdefault('lift_height_m', float(self.object_position[2]))
        result.setdefault('bilateral_contact_at_lift', False)
        result['final_object_position'] = self.object_position.tolist()
        result['final_cube_position'] = self.object_position.tolist()
        result['passed'] = controller.phase == 'done'
        return result
