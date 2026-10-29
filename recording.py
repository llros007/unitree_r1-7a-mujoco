"""Command/state CSVs for later comparison with hardware; no hardware I/O."""
import csv
import hashlib
import json
from pathlib import Path

import mujoco


class JointRecorder:
    def __init__(self, env, path=None):
        self.env = env
        self.path = Path(path).expanduser().resolve() if path is not None else None
        self.stream = None

    def __enter__(self):
        if self.path is None:
            return self
        metadata_path = self.path.with_suffix('.metadata.json')
        if self.path.exists() or metadata_path.exists():
            raise FileExistsError(f'Recording already exists: {self.path}')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        metadata = {
            'source': 'simulation, not hardware measurements',
            'units': {'time': 's', 'q': 'rad', 'dq': 'rad/s', 'tau': 'Nm', 'tcp': 'm'},
            'joint_order': [f'J{i}' for i in range(1, 8)],
            'scene': str(self.env.scene_path),
            'object_kind': self.env.object_kind,
            'scene_sha256': hashlib.sha256(self.env.scene_path.read_bytes()).hexdigest(),
            'mujoco_version': mujoco.__version__,
            'physics_timestep_s': self.env.model.opt.timestep,
            'motor_config': self.env.motors.config,
            'note': 'Rows sample the end of env.step(); commands are held between host sends. '
                    'tau_applied is simulated actuator torque, not a measured hardware torque estimate.',
            'end_effector_model': {},
        }
        for name in ['hand', 'moving_jaw', 'd435_camera']:
            body_id = mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                metadata['end_effector_model'][name] = {
                    'mass_kg': float(self.env.model.body_mass[body_id]),
                    'com_in_body_m': self.env.model.body_ipos[body_id].tolist(),
                    'principal_inertia_kg_m2': self.env.model.body_inertia[body_id].tolist(),
                    'status': 'simulation estimate; not weighed or identified',
                }
        protocol = self.env.motors.protocol
        if protocol is not None:
            metadata['sdk_calibration_path'] = str(protocol.path)
            metadata['sdk_calibration_sha256'] = hashlib.sha256(protocol.path.read_bytes()).hexdigest()
            metadata['sdk_calibration_text'] = protocol.path.read_text()
        metadata_path.write_text(json.dumps(metadata, indent=2) + '\n')
        self.stream = self.path.open('x', newline='')
        self.writer = csv.writer(self.stream)
        self.groups = ['host_q', 'host_dq', 'host_tau', 'host_kp', 'host_kd',
                       'q_cmd', 'dq_cmd', 'tau_ff', 'kp', 'kd',
                       'q', 'dq', 'tau_requested', 'tau_applied']
        self.writer.writerow(['episode', 'time_s', 'mode'] +
                             [f'{group}_J{i}' for group in self.groups for i in range(1, 8)] +
                             ['tcp_x_m', 'tcp_y_m', 'tcp_z_m'])
        self.previous = self.env.step_observer
        self.env.step_observer = self.write
        return self

    def write(self, env):
        motor = env.motors
        arrays = list(motor.host_command) + [motor.q, motor.dq, motor.tau, motor.kp, motor.kd,
                                            env.data.qpos[env.qids], env.data.qvel[env.dofs],
                                            motor.requested_tau, motor.applied_tau]
        self.writer.writerow([env.episode, env.data.time, 'cartesian' if env.cartesian_control else 'direct'] +
                             [float(value) for array in arrays for value in array] + env.tcp_position.tolist())
        if self.previous is not None:
            self.previous(env)

    def __exit__(self, *args):
        if self.stream is not None:
            self.env.step_observer = self.previous
            self.stream.close()
