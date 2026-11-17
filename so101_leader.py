"""SO-101 angle stream and joint mapping for R1-7A teleoperation."""
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np


MOTORS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
R1_AXES = np.array([0, 1, 3, 5, 4])
# Positive SO-101 elbow flexion folds the arm; positive R1 J4 opens it.
DEFAULT_SIGNS = (1, 1, -1, -1, 1)
HELD_AXES = np.setdiff1d(np.arange(7), R1_AXES)
DEFAULT_TRAVEL_DEG = 12.
MAX_LEAD_RAD = np.deg2rad(5)
# Gripper encoder count at the recorded middle pose. The low end of its range is closed.
ENCODER_CENTER = 2048
ENCODER_SPAN = 4095
GRIPPER_HALF_DEG = 50.
STALE_SECONDS = .25
# Two angle references per axis. The elbow's straight references are
# approximated from the shoulder-to-elbow and elbow-to-wrist model geometry.
ABSOLUTE_POINTS_DEG = {
    'shoulder_pan': ((-105, 105), (-105, 105)),
    'shoulder_lift': ((-90, 90), (0, 180)),
    'elbow_flex': ((-74, 86), (160, 0)),
    'wrist_flex': ((-95, 95), (30, -30)),
    'wrist_roll': ((-180, 180), (-90, 90)),
}


def load_absolute_points(path=None):
    entries = ABSOLUTE_POINTS_DEG if path is None else json.loads(path.read_text())
    if not isinstance(entries, dict) or set(entries) != set(MOTORS[:5]):
        raise ValueError('绝对映射必须为五个主臂关节分别提供 [[主臂角1, 主臂角2], [从臂角1, 从臂角2]]')
    points = np.asarray([entries[name] for name in MOTORS[:5]], dtype=float)
    if (points.shape != (5, 2, 2) or not np.isfinite(points).all() or
            np.any(points[:, 0, 0] >= points[:, 0, 1])):
        raise ValueError('映射角度必须为有限数值，且各轴主臂角1必须小于主臂角2')
    return np.deg2rad(points)


def stream_angles(port, calibration_file):
    # Only the reader subprocess needs the LeRobot environment.
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    entries = json.loads(Path(calibration_file).read_text())
    if set(entries) != set(MOTORS) or any(entries[name]['id'] != i for i, name in enumerate(MOTORS, 1)):
        raise ValueError('expected an SO-101 calibration with the six standard motor names and IDs')
    calibration = {name: MotorCalibration(**entries[name]) for name in MOTORS}
    bus = FeetechMotorsBus(port=port, motors={
        name: Motor(i, 'sts3215', MotorNormMode.DEGREES) for i, name in enumerate(MOTORS, 1)
    }, calibration=calibration)
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('leader motor calibration differs from the supplied file')
        for name in MOTORS:
            bus.write('Torque_Enable', name, 0)
        while True:
            raw = bus.sync_read('Present_Position', normalize=False, num_retry=2)
            degrees = []
            for name in MOTORS[:5]:
                item = entries[name]
                mid = (int(item['range_min']) + int(item['range_max'])) / 2
                degrees.append((float(raw[name]) - mid) * 360 / ENCODER_SPAN)
            degrees.append((float(raw['gripper']) - ENCODER_CENTER) * 360 / ENCODER_SPAN)
            print(json.dumps(degrees), flush=True)
            time.sleep(.02)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


class LeaderReader:
    def __init__(self, python, port, calibration):
        self.process = subprocess.Popen([python, '-u', str(Path(__file__).resolve()), port, str(calibration)],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, text=True)
        self.latest = None
        self.gripper_deg = None
        self.thread = threading.Thread(target=self._read, daemon=True, name='so101-feedback')
        self.thread.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                q = np.asarray(json.loads(line), dtype=float)
                if q.shape == (6,) and np.isfinite(q).all():
                    self.gripper_deg = float(q[5])
                    q = q[:5]
                if q.shape == (5,) and np.isfinite(q).all():
                    self.latest = time.monotonic(), np.deg2rad(q)
            except (ValueError, TypeError):
                continue

    def sample(self):
        latest = self.latest
        if latest is not None and self.process.poll() is None and time.monotonic() - latest[0] < STALE_SECONDS:
            return latest[1]
        return None

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.thread.join(timeout=1)
        self.process.stdout.close()


def joint_target(env, base, leader_delta, scale, speed_rad_s, signs=DEFAULT_SIGNS, travel_deg=DEFAULT_TRAVEL_DEG):
    """Map five leader rotations, holding R1 J3/J7 at the clutch pose."""
    requested = base.copy()
    travel = np.deg2rad(travel_deg)
    requested[R1_AXES] += np.clip(scale * np.asarray(signs) * leader_delta, -travel, travel)
    return guard_joint_target(env, requested, speed_rad_s)


def _travel_deg(item):
    low = (int(item['range_min']) - ENCODER_CENTER) * 360 / ENCODER_SPAN
    high = (int(item['range_max']) - ENCODER_CENTER) * 360 / ENCODER_SPAN
    return low, high


def gripper_sides_deg(calibration_file):
    """Leader degrees at the recorded gripper ends. The low count is closed."""
    gripper = json.loads(Path(calibration_file).read_text())['gripper']
    low, high = _travel_deg(gripper)
    if high - low < 2:
        return -GRIPPER_HALF_DEG, GRIPPER_HALF_DEG
    return low, high


def gripper_target(gripper_deg, open_angle, close_angle, low_deg=-GRIPPER_HALF_DEG, high_deg=GRIPPER_HALF_DEG):
    """Map the recorded gripper travel onto the sim jaw. The low count is closed."""
    opened = np.clip((float(gripper_deg) - float(low_deg)) / max(float(high_deg) - float(low_deg), 1.), 0, 1)
    return float(close_angle + opened * (open_angle - close_angle))


def absolute_joint_target(env, base, angles, points, speed_rad_s):
    """Map the leader's current pose, using the clutch pose only for J3/J7."""
    leader, follower = points[:, 0], points[:, 1]
    fraction = np.clip((angles - leader[:, 0]) / (leader[:, 1] - leader[:, 0]), 0, 1)
    requested = base.copy()
    requested[R1_AXES] = follower[:, 0] + fraction * (follower[:, 1] - follower[:, 0])
    return guard_joint_target(env, requested, speed_rad_s)


def guard_joint_target(env, requested, speed_rad_s):
    measured = env.data.qpos[env.qids]
    lo = np.maximum(env.limits[:, 0], measured - MAX_LEAD_RAD)
    hi = np.minimum(env.limits[:, 1], measured + MAX_LEAD_RAD)
    step = speed_rad_s * env.motors.command_period
    # A startup reading can be slightly outside a model limit. The reference
    # filter already clips it before sending, so rate-limit from the legal target.
    previous = np.clip(env.q_command, env.limits[:, 0], env.limits[:, 1])
    lo = np.maximum(lo, previous - step)
    hi = np.minimum(hi, previous + step)
    if np.any(lo > hi):
        return None
    if np.any((requested < lo)[HELD_AXES] | (requested > hi)[HELD_AXES]):
        return None
    target = np.clip(requested, lo, hi)
    if (env.tool_motion_collision(previous, target) is not None or
            env.tool_motion_collision(measured, target) is not None):
        return None
    return target


if __name__ == '__main__':
    stream_angles(*sys.argv[1:])
