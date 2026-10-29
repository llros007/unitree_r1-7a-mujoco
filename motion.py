"""Shared target-pose motion and contact pick/place, independent of perception."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from xle_gripper import OPEN_ANGLE, CLOSE_ANGLE

DOWN_ROTATION = np.diag([1., -1., -1.])


@dataclass
class ScriptedGrasp:
    """Known-fixture target for the headless demo, not a perception result."""
    surface: np.ndarray
    tcp: np.ndarray
    rotation: np.ndarray


class PickPlace:
    def __init__(self, env, speed=.5):
        self.env = env
        if not np.isfinite(speed) or not .1 <= speed <= 1.:
            raise ValueError('Speed must be in [0.1, 1.0]')
        self.speed = speed
        self.phase = 'idle'
        self.error = None
        self.report = {}
        self.actions = []
        self.index = 0

    @property
    def active(self):
        return self.phase not in ('idle', 'done', 'failed')

    def start(self, grasp):
        if self.active:
            raise RuntimeError('A grasp is already running')
        self.env.gripper_open = True
        self.error, self.report = None, {}
        self.grasp = grasp
        # SO101 has one fixed jaw. Descend with clearance on that side, then
        # let the moving jaw bring the pot into contact during physical closure.
        radius_clearance = 0.
        if self.env.object_kind == 'plant':
            from yolo_grasp.plant_asset import POT_RADIUS
            radius_clearance = max(0., POT_RADIUS - .02)
        tcp = grasp.tcp - grasp.rotation[:, 0] * (self.env.motors.grasp_approach_clearance + radius_clearance)
        self.report['approach_tcp_world_m'] = tcp.tolist()
        high = max(.80, grasp.surface[2] + .15)
        self.actions = [
            ('align', np.r_[tcp[:2], high], 3.),
            ('approach', np.r_[tcp[:2], grasp.surface[2] + .08], 2.),
            ('descend', tcp, 2.),
            ('close', None, 2.),
            ('lift', np.r_[tcp[:2], high], 3.),
            ('transfer', np.array([-.12, .25, high]), 3.),
            ('place', np.array([-.12, .25, grasp.tcp[2] + .007]), 3.),
            ('release', None, 2.),
            ('retreat', np.array([-.12, .25, high]), 2.),
        ]
        self.actions = [(phase, point, seconds / self.speed) for phase, point, seconds in self.actions]
        jaw_wait = abs(OPEN_ANGLE - CLOSE_ANGLE) / self.env.gripper_speed + .5
        self.actions = [(phase, point, max(seconds, jaw_wait) if phase in ('close', 'release') else seconds)
                        for phase, point, seconds in self.actions]
        self.index = 0
        self._enter()

    def _enter(self):
        self.phase, self.destination, self.duration = self.actions[self.index]
        self.elapsed = 0.
        self.start_position = self.env.target_position.copy()
        self.start_rotation = self.env.target_rotation.copy()
        self.rotation_delta = Rotation.from_matrix(self.grasp.rotation @ self.start_rotation.T).as_rotvec()
        if self.phase == 'close':
            self.env.gripper_open = False
        elif self.phase == 'release':
            self.env.gripper_open = True
        print(f'[grasp] {self.phase}', flush=True)

    def fail(self, reason):
        self.phase, self.error = 'failed', reason
        self.report.update(passed=False, error=reason)
        print(f'[grasp] FAILED: {reason}', flush=True)

    def cancel(self):
        self.phase = 'idle'
        self.actions = []
        self.error = None
        self.report = {}

    def tick(self, dt=.01):
        if not self.active or self.env.paused:
            return
        self.elapsed += dt
        if self.destination is not None:
            t = min(1., self.elapsed / self.duration)
            blend = t * t * (3 - 2 * t)
            position = self.start_position + blend * (self.destination - self.start_position)
            desired_rotation = Rotation.from_rotvec(blend * self.rotation_delta).as_matrix() @ self.start_rotation
            delta = Rotation.from_matrix(desired_rotation @ self.env.target_rotation.T).as_rotvec()
            self.env.move_target(position - self.env.target_position, delta)
        # A round tapered pot can settle lower between the jaws during lift.
        # Release on measured tabletop contact instead of pushing it down to
        # the original open-loop height. No object pose is used for planning.
        supported = self.phase == 'place' and self.env.object_supported()
        if supported:
            self.env.use_cartesian_control()
            self.destination = None
            self.elapsed = self.duration
            self.report['placement_table_contact'] = True
        if self.elapsed < self.duration:
            return
        if self.destination is not None:
            position_error = np.linalg.norm(self.env.tcp_position - self.destination)
            rotation_error = np.linalg.norm(Rotation.from_matrix(
                self.grasp.rotation @ self.env.data.site_xmat[self.env.tcp_id].reshape(3, 3).T).as_rotvec())
            if position_error > .008 or rotation_error > .12:
                if self.elapsed < self.duration + 1.:
                    return
                self.fail(f'{self.phase}: pose unreachable or blocked (position {position_error:.4f} m, angle {rotation_error:.3f} rad)')
                return
        # Ground-truth object state is used only to report/validate physical success,
        # never to set the detected grasp position or to move the cube.
        if self.phase == 'close' and len(self.env.contact_sides()) != 2:
            self.fail('No bilateral finger contact after closure')
            return
        if self.phase == 'lift':
            lifted = self.env.success
            self.report.update(lift_height_m=float(self.env.cube_position[2]),
                               bilateral_contact_at_lift=len(self.env.contact_sides()) == 2)
            if not lifted:
                self.fail('Object slipped or did not lift')
                return
        if self.phase in ('transfer', 'place') and not supported and len(self.env.contact_sides()) != 2:
            self.fail('Object lost during transfer')
            return
        self.index += 1
        if self.index == len(self.actions):
            position = self.env.cube_position
            passed = bool(np.linalg.norm(position[:2] - [-.12, .25]) < .055
                          and abs(position[2] - self.env.object_rest_height) < .015
                          and np.isfinite(self.env.data.qpos).all()
                          and not any(w.number for w in self.env.data.warning))
            self.report.update(final_object_position=position.tolist(),
                               final_cube_position=position.tolist(), passed=passed)
            if not passed:
                self.fail('Object did not settle in the drop zone, or simulation warning')
                return
            self.phase = 'done'
            print('[grasp] DONE', flush=True)
        else:
            self._enter()
