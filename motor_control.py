"""Joint-side PD + configurable feedforward torque control."""
import json
from pathlib import Path

import numpy as np
from sdk_protocol import SdkProtocol


def joint_vector(value, name, nonnegative=False):
    vector = np.asarray(value, dtype=float)
    if vector.ndim == 0:
        vector = np.full(7, float(vector))
    if vector.shape != (7,) or not np.isfinite(vector).all():
        raise ValueError(f'{name} must be a finite scalar or seven joint values')
    if nonnegative and np.any(vector < 0):
        raise ValueError(f'{name} must be nonnegative')
    return vector.copy()


class ArmMotorController:
    def __init__(self, model, joints, actuators, config_path=None):
        path = Path(config_path) if config_path is not None else Path(__file__).with_name('motor_config.json')
        config = json.loads(path.read_text())
        expected = {'kp', 'kd', 'tau_limit', 'damping', 'frictionloss', 'armature', 'torque_time_constant_s'}
        optional = {'cartesian_reference_time_constant_s', 'gravity_feedforward_scale',
                    'command_period_s', 'sdk_protocol_calibration', 'gripper_speed_rad_s',
                    'grasp_approach_clearance_m'}
        if not expected <= set(config) or set(config) - expected - optional:
            raise ValueError(f'Motor config requires {sorted(expected)}; optional: {sorted(optional)}')
        self.config = config
        self.gripper_speed = float(config.get('gripper_speed_rad_s', .3))
        if not np.isfinite(self.gripper_speed) or self.gripper_speed <= 0:
            raise ValueError('gripper_speed_rad_s must be finite and positive')
        self.grasp_approach_clearance = float(config.get('grasp_approach_clearance_m', .006))
        if not np.isfinite(self.grasp_approach_clearance) or not 0 <= self.grasp_approach_clearance <= .02:
            raise ValueError('grasp_approach_clearance_m must be between 0 and .02')
        self.command_period = float(config.get('command_period_s', model.opt.timestep))
        if not np.isfinite(self.command_period) or self.command_period < model.opt.timestep:
            raise ValueError('command_period_s must be finite and at least the physics timestep')
        ticks = self.command_period / model.opt.timestep
        if not np.isclose(ticks, round(ticks), rtol=0, atol=1e-9):
            raise ValueError('command_period_s must be a whole number of physics timesteps')
        self.command_ticks = round(ticks)
        calibration = config.get('sdk_protocol_calibration')
        self.protocol = SdkProtocol(path.resolve().parent / calibration) if calibration is not None else None
        self.reference_time_constant = float(config.get('cartesian_reference_time_constant_s', 0.))
        if not np.isfinite(self.reference_time_constant) or self.reference_time_constant < 0:
            raise ValueError('cartesian_reference_time_constant_s must be finite and nonnegative')
        self.gravity_feedforward_scale = float(config.get('gravity_feedforward_scale', 0.))
        if not np.isfinite(self.gravity_feedforward_scale) or not 0 <= self.gravity_feedforward_scale <= 1:
            raise ValueError('gravity_feedforward_scale must be between 0 and 1')
        self.default_kp = joint_vector(config['kp'], 'kp', True)
        self.default_kd = joint_vector(config['kd'], 'kd', True)
        self.tau_limit = joint_vector(config['tau_limit'], 'tau_limit', True)
        if np.any(self.tau_limit <= 0):
            raise ValueError('tau_limit must be positive')
        self.time_constant = float(config['torque_time_constant_s'])
        if not np.isfinite(self.time_constant) or self.time_constant < 0:
            raise ValueError('torque_time_constant_s must be finite and nonnegative')
        self.qids = model.jnt_qposadr[joints]
        self.dofs = model.jnt_dofadr[joints]
        self.actuators = actuators
        # These are joint-side effective parameters; no extra gear conversion.
        for key in ['damping', 'frictionloss', 'armature']:
            getattr(model, f'dof_{key}')[self.dofs] = joint_vector(config[key], key, True)
        # The configured limit may reduce, but never exceed, the MJCF/URDF cap.
        self.tau_limit = np.minimum(self.tau_limit, model.actuator_forcerange[actuators, 1])
        model.actuator_ctrlrange[actuators] = np.c_[-self.tau_limit, self.tau_limit]
        model.actuator_forcerange[actuators] = np.c_[-self.tau_limit, self.tau_limit]
        self.reset(np.zeros(7))

    def reset(self, q):
        self.set_command(q, 0, 0, self.default_kp, self.default_kd)
        self.requested_tau = np.zeros(7)
        self.applied_tau = np.zeros(7)

    def set_command(self, q, dq=0, tau=0, kp=None, kd=None):
        # Validate atomically so a rejected command leaves the previous one intact.
        values = [joint_vector(q, 'q'), joint_vector(dq, 'dq'), joint_vector(tau, 'tau'),
                  joint_vector(self.default_kp if kp is None else kp, 'kp', True),
                  joint_vector(self.default_kd if kd is None else kd, 'kd', True)]
        host_command = tuple(values)
        if self.protocol is not None:
            values = self.protocol.round_trip(*values)
        self.host_command = host_command
        self.q, self.dq, self.tau, self.kp, self.kd = values

    def update(self, data, dt):
        self.requested_tau = (self.tau + self.kp * (self.q - data.qpos[self.qids])
                              + self.kd * (self.dq - data.qvel[self.dofs]))
        target = np.clip(self.requested_tau, -self.tau_limit, self.tau_limit)
        alpha = -np.expm1(-dt / self.time_constant) if self.time_constant else 1.
        self.applied_tau += alpha * (target - self.applied_tau)
        data.ctrl[self.actuators] = self.applied_tau


class JointReference:
    """Exact critically damped target filter for IK commands, not motor physics.

    Its position and velocity remain continuous across target changes. Direct
    SDK-style commands bypass it so their q/dq semantics stay unchanged.
    """
    def __init__(self, time_constant, limits):
        self.time_constant = time_constant
        self.limits = limits

    def reset(self, q):
        self.q = np.array(q, dtype=float, copy=True)
        self.dq = np.zeros_like(self.q)

    def update(self, target, dt):
        if self.time_constant == 0:
            self.q = np.array(target, copy=True)
            self.dq[:] = 0
            return self.q, self.dq
        omega = 1. / self.time_constant
        error = self.q - target
        coefficient = self.dq + omega * error
        decay = np.exp(-omega * dt)
        self.q = target + (error + coefficient * dt) * decay
        self.dq = (self.dq - omega * coefficient * dt) * decay
        clipped = np.clip(self.q, self.limits[:, 0], self.limits[:, 1])
        self.dq[clipped != self.q] = 0
        self.q = clipped
        return self.q, self.dq
