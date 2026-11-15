"""Small Cartesian excursions using the keyboard controller's IK solver."""
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from environment import tcp_pose_at as pose


def plan_ik(env, start, distance_mm, duration):
    env.ik_data.qpos[env.gripper_q] = env.data.qpos[env.gripper_q]
    origin, rotation = pose(env, env.ik_data, start)
    env.target_rotation = rotation
    steps = np.linspace(0., 1., 21)
    segments, skipped = [], []
    for axis, name in enumerate('XYZ'):
        for sign in (1, -1):
            offset = np.zeros(3)
            offset[axis] = sign * distance_mm / 1000.
            waypoints = [start.copy()]
            for fraction in steps[1:]:
                env.target_position = origin + fraction * offset
                if np.any(env.target_position < [-.40, -.25, .622]) or np.any(env.target_position > [.40, .52, 1.22]):
                    break
                q, _ = env.solve_ik(waypoints[-1], iterations=30)
                xyz, rot = pose(env, env.ik_data, q)
                if (np.linalg.norm(xyz - env.target_position) > .0005 or
                        Rotation.from_matrix(rot @ rotation.T).magnitude() > np.deg2rad(.3)):
                    break
                waypoints.append(q)
            if len(waypoints) == len(steps):
                curve = CubicSpline(steps, waypoints)
                samples = np.linspace(0., 1., 101)
                path = curve(samples)
                if (np.max(np.abs(path - start)) <= np.deg2rad(5) and
                        np.max(np.abs(curve(samples, 1))) * 1.875 / duration <= np.deg2rad(5) and
                        np.all(path >= env.limits[:, 0]) and np.all(path <= env.limits[:, 1]) and
                        all(env.tool_motion_collision(a, b) is None for a, b in zip(path, path[1:]))):
                    for kind, reverse in [('excursion', False), ('return', True)]:
                        segments.append(dict(axis=name, sign=sign, cycle=1, kind=kind, curve=curve,
                                             reverse=reverse, origin_m=origin.tolist(),
                                             offset_m=offset.tolist(),
                                             target=path[0 if reverse else -1].copy()))
                    break
            skipped.append(f'{name}{sign:+}: IK residual, workspace, joint travel or clearance constraint')
    return segments, skipped


def plan_highest_ik(env, start):
    """Follow the vertical line from the initial TCP while keeping its orientation."""
    env.ik_data.qpos[env.gripper_q] = env.data.qpos[env.gripper_q]
    origin, rotation = pose(env, env.ik_data, start)
    env.target_rotation = rotation
    waypoints = [start.copy()]
    step = .0005
    reason = 'Cartesian height limit (1.22 m)'
    for index in range(1, int(np.floor((1.22-origin[2])/step))+1):
        env.target_position = origin + [0., 0., index*step]
        q, _ = env.solve_ik(waypoints[-1], iterations=30)
        xyz, rot = pose(env, env.ik_data, q)
        if (np.linalg.norm(xyz-env.target_position) > .0005 or
                Rotation.from_matrix(rot @ rotation.T).magnitude() > np.deg2rad(.3)):
            reason = 'IK position/orientation residual'
            break
        if np.max(np.abs(q-start)) > np.deg2rad(60):
            reason = '60 deg joint excursion limit'
            break
        if env.tool_motion_collision(waypoints[-1], q) is not None:
            reason = 'tool/arm clearance'
            break
        waypoints.append(q)
    # Validate interpolated commands as well as the solver waypoints.
    while len(waypoints) > 1:
        curve = CubicSpline(np.linspace(0., 1., len(waypoints)), waypoints)
        samples = np.linspace(0., 1., 4*len(waypoints)+1)
        path = curve(samples)
        if (np.all(path >= env.limits[:, 0]) and np.all(path <= env.limits[:, 1]) and
                np.max(np.abs(path - start)) <= np.deg2rad(60) and
                all(env.tool_motion_collision(a, b) is None for a, b in zip(path, path[1:]))):
            break
        waypoints.pop()
        reason = 'interpolated joint limit or clearance'
    rise = (len(waypoints)-1)*step
    if rise < .001:
        return [], [reason]
    speed = np.max(np.abs(curve(samples, 1)))
    acceleration = np.max(np.abs(curve(samples, 2))) * 1.875**2 + speed * 5.774
    duration = max(3., rise/.003*1.875, speed*1.875/np.deg2rad(3),
                   np.sqrt(acceleration/np.deg2rad(10)))
    return [dict(axis='Z', sign=1, cycle=1, kind=kind, curve=curve, reverse=reverse,
                 origin_m=origin.tolist(), offset_m=[0., 0., rise], move_seconds=duration,
                 target=waypoints[0 if reverse else -1].copy())
            for kind, reverse in [('excursion', False), ('return', True)]], [reason]


def ik_reference(segment, elapsed, duration):
    u = np.clip(elapsed / duration, 0., 1.)
    s = u**3 * (10 + u * (-15 + 6*u))
    ds = 30*u**2 * (1-u)**2 / duration
    if segment['reverse']:
        s, ds = 1-s, -ds
    curve = segment['curve']
    target = np.asarray(segment['origin_m']) + s*np.asarray(segment['offset_m'])
    return curve(s), curve(s, 1)*ds, target
