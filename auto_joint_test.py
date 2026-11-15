#!/usr/bin/env python3
"""Automatic joint feedback benchmark with optional independent simulation and viewer."""
import argparse
import csv
import hashlib
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

# Match the desktop teleop's driver-library setup before importing OpenGL bindings.
_SYSTEM_LIBCXX = '/usr/lib/x86_64-linux-gnu/libstdc++.so.6'
if (__name__ == '__main__' and '--view' in sys.argv and os.path.isfile(_SYSTEM_LIBCXX)
        and os.environ.get('_R1_GL_READY') != '1'):
    os.environ['_R1_GL_READY'] = '1'
    os.environ['LIBGL_DRIVERS_PATH'] = '/usr/lib/x86_64-linux-gnu/dri'
    preload = os.environ.get('LD_PRELOAD', '')
    if _SYSTEM_LIBCXX not in preload.split():
        os.environ['LD_PRELOAD'] = (_SYSTEM_LIBCXX + ' ' + preload).strip()
    os.execv(sys.executable, [sys.executable, *sys.argv])

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

from environment import ArmEnvironment
from ik_test import ik_reference, plan_highest_ik, plan_ik, pose
from real_keyboard import Bus, SDK_BIN, SDK_CALIB, check_command, check_health


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hardware', action='store_true', help='explicitly open serial and move hardware')
    p.add_argument('--sim-compare', action='store_true', help='run an independent plant on the same commands')
    p.add_argument('--view', action='store_true', help='side-by-side feedback/model and independent simulation; implies sim-compare')
    p.add_argument('--close-on-finish', action='store_true', help='automatically close the viewer after reporting')
    p.add_argument('--analyze', type=Path, help='reanalyze an existing output directory; never connects hardware')
    p.add_argument('--port', default='/dev/ttyUSB0')
    p.add_argument('--bridge', type=Path, default=SDK_BIN)
    p.add_argument('--calib', type=Path, default=SDK_CALIB)
    p.add_argument('--motor-config', type=Path)
    p.add_argument('--output', type=Path, default=Path('outputs/joint_test_' + time.strftime('%Y%m%d_%H%M%S')))
    p.add_argument('--joints', nargs='+', type=int, default=[4], help='1-based joint IDs; default J4')
    p.add_argument('--amplitude-deg', type=float, default=1.)
    p.add_argument('--large-angle', action='store_true', help='explicit staged positive J2/J4 test, at most 60 degrees')
    p.add_argument('--stages-deg', nargs='+', type=float, help='increasing positive checkpoints, e.g. 15 30 60')
    p.add_argument('--direction', choices=('both', 'positive', 'negative', 'auto'), default='both',
                   help='auto skips directions blocked by limits/tool clearance; requires one valid direction per joint')
    p.add_argument('--move-seconds', type=float, default=2.)
    p.add_argument('--hold-seconds', type=float, default=2.)
    p.add_argument('--cycles', type=int, default=3)
    p.add_argument('--feedback-timeout', type=float, default=.1)
    p.add_argument('--tolerance-deg', type=float, default=.5, help='engineering acceptance threshold, not manufacturer specification')
    p.add_argument('--velocity-tolerance-deg-s', type=float, default=.5)
    a = p.parse_args(argv)
    for name in ('amplitude_deg', 'move_seconds', 'hold_seconds', 'feedback_timeout',
                 'tolerance_deg', 'velocity_tolerance_deg_s'):
        if not math.isfinite(getattr(a, name)) or getattr(a, name) <= 0:
            p.error(f'{name} must be finite and positive')
    if a.cycles < 1 or len(set(a.joints)) != len(a.joints) or any(j not in range(1, 8) for j in a.joints):
        p.error('cycles must be positive; joints must be unique IDs in 1..7')
    if a.feedback_timeout > .1 or a.amplitude_deg > (60 if a.large_angle else 5):
        p.error('feedback timeout must be <=0.1s; amplitude cap is 5 degrees (60 only with --large-angle)')
    if a.large_angle:
        if a.direction != 'positive' or any(j not in (2,4) for j in a.joints) or not a.stages_deg:
            p.error('--large-angle requires positive direction, only J2/J4, and explicit --stages-deg')
        stages = np.asarray(a.stages_deg)
        if (not np.isfinite(stages).all() or np.any(np.diff(np.r_[0.,stages]) <= 0)
                or not np.isclose(stages[-1], a.amplitude_deg)):
            p.error('stages must increase from zero and end at amplitude-deg')
    elif a.stages_deg:
        p.error('--stages-deg requires --large-angle')
    # Quintic interpolation maximum velocity and acceleration.
    if 1.875 * a.amplitude_deg / a.move_seconds > 5 or 5.774 * a.amplitude_deg / a.move_seconds**2 > 10:
        p.error('trajectory exceeds 5 deg/s or 10 deg/s^2; increase move-seconds')
    a.calib = a.calib.expanduser().resolve()
    a.bridge = a.bridge.expanduser().resolve()
    a.sim_compare = a.sim_compare or a.view
    if a.analyze and (a.hardware or a.view):
        p.error('--analyze cannot be combined with --hardware or --view')
    if a.close_on_finish and not a.view:
        p.error('--close-on-finish requires --view')
    return a


def plan(env, start, args):
    """Single-joint excursions; return to the initial measured pose each time."""
    result = []
    if getattr(args, 'large_angle', False):
        for joint in args.joints:
            for cycle in range(args.cycles):
                previous = start.copy()
                for relative in [*args.stages_deg, 0.]:
                    target = start.copy(); target[joint-1] += np.deg2rad(relative)
                    if np.any(target < env.limits[:,0]) or np.any(target > env.limits[:,1]):
                        raise ValueError(f'J{joint} staged target outside limits')
                    collision = env.tool_motion_collision(previous, target)
                    if collision is not None:
                        raise ValueError(f'J{joint} staged tool clearance violation: {collision}')
                    delta = float(np.max(np.abs(np.rad2deg(target-previous))))
                    duration = max(.3, args.move_seconds*delta/args.amplitude_deg,
                                   1.875*delta/5., math.sqrt(5.774*delta/10.))
                    result.append(dict(joint=joint, cycle=cycle+1, sign=1,
                        kind='return' if relative==0 else 'excursion', relative_deg=relative,
                        move_seconds=duration, start=previous.copy(), target=target.copy()))
                    previous = target
        return result
    for joint in args.joints:
        joint_segments = []
        for cycle in range(args.cycles):
            direction = getattr(args, 'direction', 'both')
            signs = (1,) if direction == 'positive' else ((-1,) if direction == 'negative' else (1, -1))
            for sign in signs:
                target = start.copy()
                target[joint - 1] += sign * np.deg2rad(args.amplitude_deg)
                outside = np.any(target < env.limits[:, 0]) or np.any(target > env.limits[:, 1])
                collision = None if outside else env.tool_motion_collision(start, target)
                if direction == 'auto' and (outside or collision is not None):
                    print(f'J{joint} skip direction {sign:+}: ' + ('joint limit' if outside else f'tool clearance {collision}'), flush=True)
                    continue
                for label, q0, q1 in (('excursion', start, target), ('return', target, start)):
                    if np.any(q1 < env.limits[:, 0]) or np.any(q1 > env.limits[:, 1]):
                        raise ValueError(f'J{joint} target outside joint limits; choose another starting pose')
                    collision = env.tool_motion_collision(q0, q1)
                    if collision is not None:
                        raise ValueError(f'J{joint} path blocked by tool/arm clearance: {collision}')
                    joint_segments.append(dict(joint=joint, cycle=cycle + 1, sign=sign, kind=label,
                                       start=q0.copy(), target=q1.copy()))
        if not joint_segments:
            raise ValueError(f'J{joint}: no checked direction is available at the starting pose')
        result.extend(joint_segments)
    return result


def reference(start, target, elapsed, duration):
    u = min(1., max(0., elapsed / duration))
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
    return start + s * (target - start), ds * (target - start)


def command_wait_seconds(sent_at, now, period, hardware):
    """Minimum interval from the actual send; late computation never adds a full extra period."""
    remaining = sent_at + period - now
    if hardware and remaining < -.1:
        raise TimeoutError(f'control deadline exceeded by {-remaining*1000:.1f}ms')
    return max(0., remaining)


def metrics(samples, args, tested_joint=None):
    """Samples contain elapsed seconds, reference q, measured q and dq (rad)."""
    t = np.array([s[0] for s in samples])
    error = np.rad2deg(np.array([s[2] - s[1] for s in samples]))
    velocity = np.rad2deg(np.array([s[3] for s in samples]))
    hold = t >= args.move_seconds
    tail = t >= max(args.move_seconds, t[-1] - min(.5, args.hold_seconds / 2))
    position_good = np.abs(error) <= args.tolerance_deg
    velocity_good = np.abs(velocity) <= args.velocity_tolerance_deg_s

    def settling(good):
        good = hold & good
        bad = np.flatnonzero(~good)
        first = int(bad[-1] + 1) if len(bad) else 0
        return float(max(0., t[first] - args.move_seconds)) if first < len(t) and t[-1] - t[first] >= .2 else None

    per_joint = [settling(position_good[:, j] & velocity_good[:, j]) for j in range(7)]
    position_passed = [settling(position_good[:, j]) is not None for j in range(7)]
    velocity_passed = [settling(velocity_good[:, j]) is not None for j in range(7)]
    selected = list(range(7)) if tested_joint is None else [tested_joint - 1]
    good = np.all(position_good[:, selected] & velocity_good[:, selected], axis=1)
    # The endpoint must remain within tolerance through the end of the hold.
    settle = settling(good)
    return dict(rmse_deg=np.sqrt(np.mean(error**2, axis=0)).tolist(),
                max_abs_error_deg=np.max(np.abs(error), axis=0).tolist(),
                steady_bias_deg=np.mean(error[tail], axis=0).tolist(),
                steady_std_deg=np.std(error[tail], axis=0).tolist(),
                position_passed=position_passed, velocity_passed=velocity_passed,
                settling_seconds_per_joint=per_joint,
                all_joints_passed=all(s is not None for s in per_joint),
                holding_joints_passed=all(per_joint[j] is not None for j in range(7) if j not in selected),
                settling_seconds=settle, passed=settle is not None,
                endpoint_mean_deg=np.rad2deg(np.mean([s[2] for s, keep in zip(samples, tail) if keep], axis=0)).tolist())


class SimulationBus:
    def __init__(self, env):
        self.env = env
        self.q = env.data.qpos[env.qids].copy()
        self.dq = env.data.qvel[env.dofs].copy()
        self.tau = np.zeros(7)
        self.mode = np.ones(7, dtype=int)
        self.err = np.zeros(7, dtype=int)

    def send(self, *command):
        self.env.set_joint_command(*command)
        self.env.step(self.env.motors.command_period)
        self.q = self.env.data.qpos[self.env.qids].copy()
        self.dq = self.env.data.qvel[self.env.dofs].copy()
        self.tau = self.env.motors.applied_tau.copy()

    def stop(self):
        pass


class ShadowSimulation:
    """Independent plant, zero-order command hold on host timestamps, fixed physics steps."""
    def __init__(self, env, q, dq):
        self.env = env
        env.data.qpos[env.qids] = q
        env.data.qvel[env.dofs] = dq
        env.set_joint_command(q, 0, 0)
        self.origin = None
        self.steps = 0

    def advance(self, timestamp):
        if self.origin is None:
            self.origin = timestamp
        target = math.floor((timestamp - self.origin + 1e-9) / self.env.model.opt.timestep)
        ticks = target - self.steps
        if ticks > 0:
            self.env.step(ticks * self.env.model.opt.timestep)
            self.steps = target

    def sample(self, command, sent, received):
        self.advance(sent)  # Previous command persists until the next host send.
        self.env.set_joint_command(*command)
        self.advance(received)
        return (self.env.data.qpos[self.env.qids].copy(), self.env.data.qvel[self.env.dofs].copy())


def run(args, observer=None, stop_event=None, heartbeat=None):
    ik_distance = getattr(args, 'ik_distance_mm', None)
    env = ArmEnvironment(initial_pose='clearance', motor_config=args.motor_config, calibration_path=args.calib)
    shadow_env = (ArmEnvironment(initial_pose='clearance', motor_config=args.motor_config,
                                 calibration_path=args.calib) if args.sim_compare else None)
    stop_event = stop_event or threading.Event()
    period = env.motors.command_period
    if min(args.move_seconds, args.hold_seconds) < max(.3, 2 * period):
        raise ValueError('move/hold time must be at least max(0.3s, two command periods)')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(source='hardware_feedback' if args.hardware else 'simulation', status='running',
                  measures='joint feedback tracking only; no external joint/TCP ground truth',
                  thresholds=dict(position_deg=args.tolerance_deg, velocity_deg_s=args.velocity_tolerance_deg_s),
                  arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                  motor_config=env.motors.config, calibration_sha256=hashlib.sha256(args.calib.read_bytes()).hexdigest(),
                  scene_sha256=hashlib.sha256(env.scene_path.read_bytes()).hexdigest(),
                  calibration_text=args.calib.read_text(), segments=[])
    report['simulation_comparison'] = dict(enabled=args.sim_compare,
        note='Independent dynamics initialized once from feedback; no feedback injection. Host send/receive timestamps, not synchronized motor sampling.')
    bus = None
    began = time.monotonic()
    deadline = began
    count = 0
    failure = None
    previous_send = None
    fk_data = mujoco.MjData(env.model) if ik_distance else None
    if ik_distance:
        report['test_type'] = 'cartesian_ik'
        report['measures'] = 'TCP from encoder FK and independent simulation, with position and orientation errors'

    def validate_feedback():
        if not np.isfinite(np.r_[bus.q, bus.dq, bus.tau]).all():
            raise RuntimeError('nonfinite feedback')
        # Feedback can cross a nominal boundary slightly (servo error / quantization).
        margin = np.deg2rad(.5)
        if np.any(bus.q < env.limits[:, 0] - margin) or np.any(bus.q > env.limits[:, 1] + margin):
            raise RuntimeError('measured joint outside model limits')
        if env.tool_collision(bus.q) is not None:
            raise RuntimeError('measured pose violates tool/arm clearance')

    def exchange(q, dq, allow_enabling=False, integrate=False, unwind=False):
        nonlocal deadline, count, previous_send
        check_health(bus, require_foc=not allow_enabling)
        validate_feedback()
        tau = env.gravity_feedforward(q)
        command = (q, dq, tau, env.motors.default_kp, env.motors.default_kd)
        now = time.monotonic() - began
        dt = now - previous_send if args.hardware and previous_send is not None else period
        command = env.motors.compensate_hold(command, bus.q, dt,
            integrate=integrate and not allow_enabling, unwind=unwind and not allow_enabling)
        check_command(env, bus, command)
        if (args.large_angle or ik_distance) and np.max(np.abs(q-bus.q)) > np.deg2rad(5):
            raise RuntimeError('tracking error exceeds 5 degrees')
        sent = time.monotonic() - began
        if args.hardware and previous_send is not None and sent - previous_send - period > .1:
            raise TimeoutError('inter-command gap exceeded the period by 100ms')
        previous_send = sent
        bus.send(*command)
        received = time.monotonic() - began
        check_health(bus, require_foc=not allow_enabling)
        validate_feedback()
        check_command(env, bus, command)
        if (args.large_angle or ik_distance) and np.max(np.abs(q-bus.q)) > np.deg2rad(5):
            raise RuntimeError('tracking error exceeds 5 degrees')
        count += 1
        return command, sent, received

    try:
        with (args.output / 'trace.csv').open('x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['segment', 'phase', 'reference_time_s', 'send_time_s', 'receive_time_s'] +
                            [f'{field}_J{j}' for field in ('q_cmd_rad', 'dq_cmd_rad_s', 'tau_ff_Nm', 'kp', 'kd',
                                                         'q_feedback_rad', 'dq_feedback_rad_s', 'tau_feedback_Nm', 'mode', 'err', 'tau_bias_Nm')
                             for j in range(1, 8)] +
                            ([f'{field}_J{j}' for field in ('q_sim_rad', 'dq_sim_rad_s') for j in range(1, 8)] +
                             ['sim_time_s'] if args.sim_compare else []) +
                            ([f'tcp_{kind}_{axis}_m' for kind in ('target', 'command', 'feedback', 'sim')
                              for axis in 'xyz'] + ['orientation_command_deg', 'orientation_feedback_deg',
                              'orientation_sim_deg', 'orientation_feedback_minus_sim_deg'] if ik_distance else []))

            def sample(segment, phase, t, q, dq, enabling=False, tcp_target=None):
                nonlocal deadline
                if stop_event.is_set():
                    raise InterruptedError('test cancelled by user')
                if heartbeat is not None and time.monotonic() - heartbeat() > .5:
                    raise TimeoutError('viewer heartbeat lost for 0.5s; cancelling test')
                command, sent, received = exchange(q, dq, enabling, integrate=phase=='hold', unwind=phase=='move')
                arrays = [*command, bus.q, bus.dq, bus.tau, bus.mode, bus.err, env.motors.position_bias.bias]
                extra = []
                if shadow is not None:
                    if args.hardware:
                        sim_q, sim_dq = shadow.sample(command, sent, received)
                    else:
                        # Offline plant and shadow use the same fixed physical duration per command.
                        shadow.env.set_joint_command(*command)
                        shadow.env.step(period)
                        shadow.steps += env.motors.command_ticks
                        sim_q = shadow.env.data.qpos[shadow.env.qids].copy()
                        sim_dq = shadow.env.data.qvel[shadow.env.dofs].copy()
                    extra = np.r_[sim_q, sim_dq, shadow.steps * shadow.env.model.opt.timestep].tolist()
                else:
                    sim_q, sim_dq = bus.q.copy(), bus.dq.copy()
                if ik_distance:
                    command_xyz, command_rot = pose(env, fk_data, q)
                    feedback_xyz, feedback_rot = pose(env, fk_data, bus.q)
                    sim_xyz, sim_rot = pose(env, fk_data, sim_q)
                    rotations = [command_rot @ env.target_rotation.T, feedback_rot @ env.target_rotation.T,
                                 sim_rot @ env.target_rotation.T, feedback_rot @ sim_rot.T]
                    angles = np.rad2deg(Rotation.from_matrix(rotations).magnitude())
                    target_xyz = origin if tcp_target is None else tcp_target
                    extra += np.r_[target_xyz, command_xyz, feedback_xyz, sim_xyz, angles].tolist()
                writer.writerow([segment, phase, t, sent, received] + np.concatenate(arrays).tolist() + extra)
                if observer is not None:
                    snapshot = dict(q=bus.q.copy(), dq=bus.dq.copy(), sim_q=sim_q.copy(),
                                    sim_dq=sim_dq.copy(), target=q.copy(), segment=segment, phase=phase,
                                    source=report['source'], time_s=received)
                    if ik_distance:
                        snapshot['tcp_target'] = target_xyz.copy()
                        snapshot['tcp_error_mm'] = 1000 * np.linalg.norm(feedback_xyz - sim_xyz)
                    observer(snapshot)
                if args.hardware or args.view:
                    now = time.monotonic()
                    stop_event.wait(command_wait_seconds(began + sent, now, period, args.hardware))
                return (t, q.copy(), bus.q.copy(), bus.dq.copy())

            if stop_event.is_set():
                raise InterruptedError('test cancelled before connecting')
            bus = Bus(args.bridge, args.port, args.calib, args.feedback_timeout) if args.hardware else SimulationBus(env)
            report['initial_feedback_info'] = getattr(bus, 'info', {})
            # Connection setup may take seconds; scheduling starts only after feedback arrives.
            deadline = time.monotonic()
            measured_start = bus.q.copy()
            start = measured_start.copy()
            report['initial_q_deg'] = np.rad2deg(measured_start).tolist()
            report['initial_dq_deg_s'] = np.rad2deg(bus.dq).tolist()
            margin = np.deg2rad(.5) if ik_distance and args.hardware else 0.
            if (not np.isfinite(start).all() or np.any(start < env.limits[:, 0] - margin)
                    or np.any(start > env.limits[:, 1] + margin)):
                raise ValueError('invalid initial pose; verify calibration')
            if ik_distance:
                start = np.clip(measured_start, env.limits[:, 0], env.limits[:, 1])
                if env.tool_motion_collision(measured_start, start) is not None:
                    raise ValueError('initial pose adjustment violates tool/arm clearance')
                report['ik_reference_start_q_deg'] = np.rad2deg(start).tolist()
                if args.hardware:
                    bus.stop()
                    bus = None
                origin, env.target_rotation = pose(env, fk_data, start)
                if getattr(args, 'ik_highest', False):
                    segments, skipped = plan_highest_ik(env, start)
                    if segments:
                        print(f"Vertical rise {segments[0]['offset_m'][2]*1000:.1f} mm; "
                              f"{segments[0]['move_seconds']:.1f} s each way; endpoint constraint {skipped[0]}", flush=True)
                else:
                    segments, skipped = plan_ik(env, start, ik_distance, args.move_seconds)
                report['skipped_ik_directions'] = skipped
                report['planned_axes'] = [s['axis'] for s in segments if s['kind'] == 'excursion']
                if not segments:
                    raise ValueError('No Cartesian path passed IK and clearance checks')
                print(f"IK axes {report['planned_axes']}; skipped {skipped}", flush=True)
                if args.hardware:
                    if stop_event.is_set():
                        raise InterruptedError('test cancelled during planning')
                    bus = Bus(args.bridge, args.port, args.calib, args.feedback_timeout)
                    report['reconnected_q_deg'] = np.rad2deg(bus.q).tolist()
                    if np.max(np.abs(bus.q-measured_start)) > np.deg2rad(.25):
                        raise RuntimeError('starting pose changed by over 0.25 degrees during IK planning')
                    if env.tool_motion_collision(bus.q, start) is not None:
                        raise ValueError('reconnected pose cannot safely reach IK start')
            else:
                segments = plan(env, start, args)
                report['planned_directions'] = {f'J{j}': sorted({s['sign'] for s in segments if s['joint']==j})
                                                for j in args.joints}
            shadow = ShadowSimulation(shadow_env, bus.q.copy(), bus.dq.copy()) if shadow_env is not None else None
            deadline = time.monotonic()
            stable = 0
            best_stable = 0
            non_foc = np.zeros(7, dtype=int)
            enable_began = time.monotonic()
            report['enable'] = dict(required_consecutive=20, attempts=0)
            for n in range(max(20, math.ceil(2 / period))):
                sample(-1, 'enable', n * period, start, np.zeros(7), True)
                stable = stable + 1 if np.all(bus.mode == 1) else 0
                best_stable = max(best_stable, stable)
                non_foc += bus.mode != 1
                report['enable'].update(attempts=n+1, best_consecutive=best_stable,
                    non_foc_counts=non_foc.tolist(), final_mode=bus.mode.tolist(),
                    final_errors=bus.err.tolist(), wall_seconds=time.monotonic()-enable_began)
                if stable >= 20:
                    break
            if stable < 20:
                bad = [f'J{j+1}:mode={int(bus.mode[j])},nonFOC={int(non_foc[j])}'
                       for j in range(7) if non_foc[j] or bus.mode[j] != 1]
                raise RuntimeError(f'FOC did not remain stable for 20 cycles; best={best_stable}, '
                                   f'attempts={n+1}; ' + '; '.join(bad))
            for index, segment in enumerate(segments):
                label = f"{segment['axis']}{segment['sign']:+}" if ik_distance else f"J{segment['joint']}"
                target_label = ('' if ik_distance else
                                f" target={np.rad2deg(segment['target'][segment['joint']-1]):.2f} deg")
                print(f"{index + 1}/{len(segments)} {label} cycle={segment['cycle']} {segment['kind']}"
                      f"{target_label}", flush=True)
                samples = []
                move_seconds = segment.get('move_seconds', args.move_seconds)
                segment_args = SimpleNamespace(**{**vars(args), 'move_seconds': move_seconds})
                segment_began = time.monotonic()
                step = 0
                while True:
                    t = time.monotonic() - segment_began if args.hardware else step * period
                    if ik_distance:
                        q, dq, tcp_target = ik_reference(segment, t, move_seconds)
                    else:
                        q, dq = reference(segment['start'], segment['target'], t, move_seconds)
                        tcp_target = None
                    samples.append(sample(index, 'hold' if t >= move_seconds else 'move', t, q, dq,
                                          tcp_target=tcp_target))
                    step += 1
                    if t >= move_seconds + args.hold_seconds:
                        break
                result = {k: v for k, v in segment.items() if k not in ('start', 'target', 'curve')}
                result['target_deg'] = np.rad2deg(segment['target']).tolist()
                result.update(metrics(samples, segment_args, segment.get('joint')))
                report['segments'].append(result)
                bad_position = [j+1 for j, ok in enumerate(result['position_passed']) if not ok]
                bad_velocity = [j+1 for j, ok in enumerate(result['velocity_passed']) if not ok]
                print(f"  {label}: {'PASS' if result['passed'] else 'FAIL'}; "
                      f'position failed={bad_position}; raw velocity failed={bad_velocity}', flush=True)
                if args.large_angle and (max(abs(x) for x in result['steady_bias_deg']) > 2.
                                         or max(result['steady_std_deg']) > .1):
                    raise RuntimeError('staged checkpoint not stable within 2 deg / 0.1 deg spread; no further angle increase')
            report['tested_joints_passed'] = all(s['passed'] for s in report['segments'])
            report['holding_joints_passed'] = all(s['holding_joints_passed'] for s in report['segments'])
            report['all_positions_passed'] = all(all(s['position_passed']) for s in report['segments'])
            report['all_raw_velocities_passed'] = all(all(s['velocity_passed']) for s in report['segments'])
            report['status'] = 'passed' if all(s['all_joints_passed'] for s in report['segments']) else 'failed_threshold'
            # Dispersion of return means, based on encoders only, not external repeatability.
            returns = [s['endpoint_mean_deg'] for s in report['segments'] if s['kind'] == 'return']
            report['return_feedback_std_deg'] = np.std(returns, axis=0).tolist()
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'aborted'
        report['error'] = f'{type(exc).__name__}: {exc}'
        failure = exc
    finally:
        if bus is not None:
            report['last_feedback_info'] = getattr(bus, 'info', {})
            try:
                bus.stop()
            except Exception as exc:
                report['stop_error'] = str(exc)
                report['status'] = 'aborted'
                failure = exc
        report['samples'] = count
        report['final_position_bias_nm'] = env.motors.position_bias.bias.tolist()
        report['wall_seconds'] = time.monotonic() - began
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(f"{report['status']}: {args.output.resolve() / 'report.json'}", flush=True)
    if ik_distance and 'all_positions_passed' in report:
        print(f"Joint position conditions: {'PASS' if report['all_positions_passed'] else 'FAIL'}; "
              f"raw joint velocity: {'PASS' if report['all_raw_velocities_passed'] else 'FAIL'}", flush=True)
    elif 'tested_joints_passed' in report:
        print(f"Tested joints: {'PASS' if report['tested_joints_passed'] else 'FAIL'}; "
              f"holding joints: {'PASS' if report['holding_joints_passed'] else 'FAIL'}", flush=True)
    # Offline reporting runs after the driver has stopped, outside the control deadline.
    try:
        from joint_test_report import analyze
        analyze(args.output)
    except Exception as exc:
        print(f'Postprocessing failed (raw trace/report preserved): {exc}', flush=True)
    if failure is not None:
        print(report.get('error', report.get('stop_error')))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    def terminate(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')

    signal.signal(signal.SIGTERM, terminate)
    args = parse_args()
    if args.analyze:
        from joint_test_report import analyze
        analyze(args.analyze)
        raise SystemExit(0)
    if args.view:
        from joint_test_viewer import run_viewer
        raise SystemExit(run_viewer(args))
    raise SystemExit(run(args))
