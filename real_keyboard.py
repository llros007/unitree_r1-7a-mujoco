#!/usr/bin/env python3
"""Shared keyboard teleop with explicit hardware opt-in and isolated control/render loops."""
import os
import sys

_SYSTEM_LIBCXX = '/usr/lib/x86_64-linux-gnu/libstdc++.so.6'
if __name__ == '__main__' and os.path.isfile(_SYSTEM_LIBCXX) and os.environ.get('_R1_GL_READY') != '1':
    os.environ['_R1_GL_READY'] = '1'
    os.environ['LIBGL_DRIVERS_PATH'] = '/usr/lib/x86_64-linux-gnu/dri'
    preload = os.environ.get('LD_PRELOAD', '')
    if _SYSTEM_LIBCXX not in preload.split():
        os.environ['LD_PRELOAD'] = (_SYSTEM_LIBCXX + ' ' + preload).strip()
    os.execv(sys.executable, [sys.executable, *sys.argv])

import argparse
import copy
import json
import queue
import selectors
import subprocess
import threading
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment, tcp_pose_at
from main import HELP as SIM_HELP, KeyboardViewer
from so101_leader import (DEFAULT_SIGNS, DEFAULT_TRAVEL_DEG, LeaderReader, absolute_joint_target,
                          gripper_sides_deg, gripper_target, joint_target, load_absolute_points)
from xle_gripper import CLOSE_ANGLE, OPEN_ANGLE

ROOT = Path(__file__).resolve().parent
# Never change model/calibration just because a sibling checkout exists.
SDK_BIN = ROOT / 'r1-7a_sdk/bin/demo_cmd_stream'
SDK_CALIB = ROOT / 'r1-7a_sdk/config/joint_calib.txt'
TRACK_ABORT_RAD = .40
INPUT_TIMEOUT = .5
DEFAULT_LEADER_HARDWARE_SPEED_DEG = 5.
MAX_LEADER_HARDWARE_SPEED_DEG = 60.
# A moving leader already streams continuous setpoints. Use less smoothing
# when previewing absolute follow with the default simulated motor config.
ABSOLUTE_SIM_REFERENCE_TIME_CONSTANT = .025
REAL_HELP = (SIM_HELP
             .replace('--control-mode joint:', 'Tab切换至关节模式：')
             .replace('默认从七关节全零姿态启动；', '纯仿真从零位启动，真机从实测姿态启动；'))
HARDWARE_NOTE = """
默认只跑仿真，不打开串口。真机必须显式添加 --hardware。
  --mode sync --hardware：控制器使用真机实测，画面同步实测。
  --mode sim：独立仿真；加 --hardware 时将仿真参考目标发给真机。
R：从当前姿态缓慢回到七关节零位，到位后保持；默认峰值10度/秒。
Tab：取消当前运动并保持当前位置，切换 IK / 关节模式。
P：暂停并清零目标速度，继续保持；恢复时不会追赶暂停前的目标。
Space：只控制仿真夹爪/显示标记，不驱动真机夹爪舵机。
默认使用仓库内部标定；--calib 可明确选择其他文件。
退出或故障会停驱动，机械臂可能失去主动支撑。
退出：MuJoCo窗口按Esc或点关闭；终端输入q后回车也可退出。
"""
LEADER_HELP = ('SO-101 主臂：点击窗口，按住 Space 跟随，松开保持；P 暂停，Esc 退出。\n'
               'C：一键把盆栽放回原来的位置，机械臂保持不动。')


def parse_state(line):
    parts = line.split()
    if len(parts) != 36 or parts[0] != 'STATE':
        raise RuntimeError(f'总线状态行无法解析: {line!r}')
    values = np.array([float(x) for x in parts[1:]])
    if not np.isfinite(values).all():
        raise RuntimeError('总线状态包含非有限数值')
    if np.any(values[21:] != np.floor(values[21:])) or np.any(values[21:] < 0):
        raise RuntimeError('总线 mode/err 必须为非负整数')
    return (values[:7], values[7:14], values[14:21],
            values[21:28].astype(int), values[28:].astype(np.uint32))


class Bus:
    """One outstanding request, bounded response wait; only the worker owns this pipe."""
    def __init__(self, binary, port, calib, timeout=.1):
        self.timeout = timeout
        self.buffer = bytearray()
        self.info = {}
        self.selector = selectors.DefaultSelector()
        self.proc = subprocess.Popen(
            [str(binary), '--port', port, '--calib', str(calib)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        os.set_blocking(self.proc.stdout.fileno(), False)
        self.selector.register(self.proc.stdout, selectors.EVENT_READ)
        try:
            self._assign(self._read_state(timeout=5.))
        except Exception:
            self.stop()
            raise

    def _assign(self, state):
        self.q, self.dq, self.tau, self.mode, self.err = state

    def _read_state(self, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            while b'\n' in self.buffer:
                line, _, rest = self.buffer.partition(b'\n')
                self.buffer = bytearray(rest)
                decoded = line.decode('utf-8', errors='replace')
                if decoded.startswith('STATE '):
                    return parse_state(decoded)
                if decoded.startswith('INFO '):
                    info = json.loads(decoded[5:])
                    for key in ('packets', 'age_s', 'driver_temperature_c', 'winding_temperature_c', 'voltage_v'):
                        values = np.asarray(info[key], dtype=float)
                        if values.shape != (7,) or not np.isfinite(values).all():
                            raise RuntimeError('Invalid motor feedback diagnostics')
                    self.info = info
                    continue
                print(decoded, flush=True)
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self.selector.select(remaining):
                raise TimeoutError('总线响应超时，停止发送运动指令')
            chunk = os.read(self.proc.stdout.fileno(), 8192)
            if not chunk:
                raise RuntimeError('总线进程已退出或状态流关闭')
            self.buffer.extend(chunk)
            if len(self.buffer) > 65536:
                raise RuntimeError('总线状态行过长')

    def send(self, q, dq, tau, kp, kd):
        row = np.concatenate([q, dq, tau, kp, kd])
        if row.shape != (35,) or not np.isfinite(row).all():
            raise ValueError('总线指令必须包含35个有限数值')
        self.proc.stdin.write((' '.join(f'{v:.8g}' for v in row) + '\n').encode())
        self._assign(self._read_state())

    def stop(self):
        try:
            if self.proc.poll() is None:
                try:
                    self.proc.stdin.write(b'STOP\n')
                    self.proc.wait(timeout=.5)
                except Exception:
                    self.proc.kill()
                    self.proc.wait(timeout=1)
        finally:
            self.selector.close()
            self.proc.stdin.close()
            self.proc.stdout.close()


def show_measured(env, q, dq):
    env.data.qpos[env.qids] = q
    env.data.qvel[env.dofs] = dq
    env.data.qpos[env.gripper_q] = OPEN_ANGLE if env.gripper_open else CLOSE_ANGLE
    mujoco.mj_forward(env.model, env.data)


def reanchor(env, bus=None):
    if bus is not None:
        show_measured(env, bus.q, bus.dq)
    env.use_cartesian_control()
    env.ik_error = 0.


def hold_command(env):
    """Freeze the reference and its filter state, so resume cannot replay old motion."""
    env.zero_return = None
    env.cartesian_control = True
    env.q_command = env.reference.q.copy()
    env.reference.reset(env.q_command)
    return command_from_env(env)


def command_from_env(env):
    if env.zero_return is not None:
        return env.motors.host_command
    q, dq = env.reference.update(env.q_command, env.motors.command_period)
    tau = env.gravity_feedforward(q)
    env.motors.set_command(q, dq, tau)
    return env.motors.host_command


def check_health(bus, require_foc=True):
    info = getattr(bus, 'info', {})
    if info:
        if np.any(np.asarray(info['age_s']) > .1) or np.any(np.asarray(info['age_s']) < 0):
            raise RuntimeError('Motor feedback older than 100ms')
        if max(info['driver_temperature_c'] + info['winding_temperature_c']) > 60:
            raise RuntimeError('Motor temperature exceeds conservative test threshold 60 C')
    if np.any(bus.err != 0) or (require_foc and np.any(bus.mode != 1)):
        raise RuntimeError(f'电机退出 FOC 或报错 mode={bus.mode}, '
                           f'err={[hex(int(e)) for e in bus.err]}')


def check_command(env, bus, command):
    q, dq, tau, kp, kd = command
    if np.any(q < env.limits[:, 0]) or np.any(q > env.limits[:, 1]):
        raise RuntimeError('目标角超出模型关节范围')
    if np.max(np.abs(q - bus.q)) > TRACK_ABORT_RAD:
        raise RuntimeError(f'跟踪误差超过 {TRACK_ABORT_RAD} rad')
    predicted = tau + kp * (q - bus.q) + kd * (dq - bus.dq)
    if np.any(np.abs(predicted) > env.motors.tau_limit):
        raise RuntimeError('估算总力矩超过配置限制；此检查不替代驱动器硬件限幅')


def make_controller(env, joint_speed):
    # Callbacks and controls need no GL resources; they run only in the worker.
    control = KeyboardViewer.__new__(KeyboardViewer)
    control.env, control.window, control.keys = env, None, set()
    control.control_mode, control.selected_joint = 'cartesian', 0
    control.joint_speed, control.joint_status = np.deg2rad(joint_speed), 'Hold'
    return control


def handle_key(control, bus, key, action, mods):
    if action == glfw.PRESS and key == glfw.KEY_H:
        print(REAL_HELP + HARDWARE_NOTE, flush=True)
        return
    if action == glfw.PRESS and key in (glfw.KEY_R, glfw.KEY_TAB, glfw.KEY_P):
        if key == glfw.KEY_R and control.env.zero_return is not None:
            return
        # With hardware attached, ALL modes start home/switch from real measurements.
        if key in (glfw.KEY_R, glfw.KEY_TAB):
            reanchor(control.env, bus)
        elif not control.env.paused:
            hold_command(control.env)
        control.keys.clear()
        if key == glfw.KEY_R:
            control.joint_status = control.env.start_zero_return(control.home_speed_deg, control.home_accel_deg)
            print(control.joint_status, flush=True)
        elif key == glfw.KEY_TAB:
            control.toggle_control_mode()
        else:
            control.env.paused = not control.env.paused
            print('控制已暂停；按 P 恢复' if control.env.paused else '控制已恢复', flush=True)
        return
    control.on_key(None, key, 0, action, mods)


class Session:
    """One model owned by the worker; UI receives copied snapshots, never shares MjData."""
    def __init__(self, env, args):
        self.env, self.args = env, args
        if (args.mode == 'sim' and not args.hardware and
                getattr(args, 'leader_absolute_points', None) is not None and
                getattr(args, 'motor_config', None) is None):
            env.reference.time_constant = min(env.reference.time_constant, ABSOLUTE_SIM_REFERENCE_TIME_CONSTANT)
        self.control = make_controller(env, args.joint_speed)
        self.control.control_mode = getattr(args, 'control_mode', 'cartesian')
        self.control.home_speed_deg = getattr(args, 'home_speed_deg', 10.)
        self.control.home_accel_deg = getattr(args, 'home_accel_deg', 20.)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.snapshot = None
        self.error = None
        self.tick_timings = {}
        self.home_generation = 0
        self.home_pending = False
        self.home_results = queue.Queue()
        self.last_compensation_at = None
        self.leader = None
        self.gripper_sides = (-50., 50.)
        self.leader_anchor = None
        self.input_at = time.monotonic()
        self.thread = threading.Thread(target=self.run, name='r1-control', daemon=True)

    def update_leader(self, allow_motion):
        e, c = self.env, self.control
        sample = self.leader.sample()
        if not allow_motion or glfw.KEY_SPACE not in c.keys:
            if sample is None:
                c.joint_status = 'Leader feedback unavailable'
            elif not allow_motion:
                c.joint_status = 'Paused'
            elif c.joint_status in ('Leader feedback unavailable', 'Paused'):
                c.joint_status = 'Hold SPACE to follow'
            return
        if sample is None:
            raise TimeoutError('SO-101 主臂反馈超时，停止遥操作')
        if self.leader_anchor is None:
            self.leader_anchor = sample.copy(), np.clip(e.q_command, e.limits[:, 0], e.limits[:, 1])
        leader_origin, robot_origin = self.leader_anchor
        delta = sample - leader_origin
        if np.any(np.abs(delta) > np.deg2rad(180)):
            c.keys.clear()
            c.joint_status = 'Leader angle wrapped; press SPACE again'
            return
        speed_deg_s = self.args.joint_speed
        if self.args.hardware:
            speed_deg_s = min(speed_deg_s, MAX_LEADER_HARDWARE_SPEED_DEG)
        speed = np.deg2rad(speed_deg_s)
        absolute_points = getattr(self.args, 'leader_absolute_points', None)
        if absolute_points is not None:
            # Keep the previous sample for wrap detection. The five mapped
            # targets depend only on the current sample and the fixed table.
            self.leader_anchor = sample.copy(), robot_origin
            target = absolute_joint_target(e, robot_origin, sample, absolute_points, speed)
        else:
            travel_deg = (DEFAULT_TRAVEL_DEG if self.args.hardware else
                          getattr(self.args, 'leader_travel_deg', DEFAULT_TRAVEL_DEG))
            target = joint_target(e, robot_origin, delta, self.args.leader_scale, speed,
                                  signs=getattr(self.args, 'leader_signs', DEFAULT_SIGNS), travel_deg=travel_deg)
        if target is None:
            c.keys.clear()
            c.joint_status = 'Blocked; press SPACE again'
            return
        e.q_command = target
        e.target_position, e.target_rotation = tcp_pose_at(e, e.ik_data, target)
        e.update_marker()
        c.joint_status = 'Absolute follow' if absolute_points is not None else 'Following SO-101'

    def follow_leader_gripper(self):
        if self.leader is None:
            return
        degrees = getattr(self.leader, 'gripper_deg', None)
        if degrees is None:
            return
        goal = gripper_target(degrees, OPEN_ANGLE, CLOSE_ANGLE, *self.gripper_sides)
        self.env.gripper_goal = goal
        self.env.gripper_open = goal > (OPEN_ANGLE + CLOSE_ANGLE) / 2

    def restore_object(self):
        e = self.env
        joint = e.model.joint('cube_free')
        qadr = int(e.cube_q)
        dadr = int(e.model.jnt_dofadr[joint.id])
        e.data.qpos[qadr:qadr + 7] = e.model.qpos0[qadr:qadr + 7]
        e.data.qvel[dadr:dadr + 6] = 0
        mujoco.mj_forward(e.model, e.data)
        self.control.joint_status = '盆栽已放回原位'
        print('盆栽已放回原位', flush=True)

    def cancel_home_planning(self):
        self.home_generation += 1
        self.home_pending = False

    def request_home(self, bus):
        if self.home_pending or self.env.zero_return is not None:
            return
        if self.env.paused:
            self.control.joint_status = 'Paused: press P before returning to zero'
            return
        reanchor(self.env, bus)
        hold_command(self.env)
        self.control.keys.clear()
        self.home_generation += 1
        generation = self.home_generation
        self.home_pending = True
        self.control.joint_status = 'Checking zero-return path; holding current pose'
        # Only immutable model/geometry are shared; worker owns all MjData scratch space.
        planner = copy.copy(self.env)
        qpos = self.env.data.qpos.copy()
        start = qpos[self.env.qids].copy()
        speed, accel = self.control.home_speed_deg, self.control.home_accel_deg

        def compute():
            try:
                planner.data = mujoco.MjData(planner.model)
                planner.data.qpos[:] = qpos
                planner.ik_data = mujoco.MjData(planner.model)
                result = planner.plan_zero_return(start, speed, accel)
            except Exception as exc:
                result = f'Blocked: zero-return planning failed: {exc}'
            self.home_results.put((generation, result, speed, accel))

        threading.Thread(target=compute, name='r1-home-planner', daemon=True).start()

    def accept_home_result(self, bus):
        while True:
            try:
                generation, result, speed, accel = self.home_results.get_nowait()
            except queue.Empty:
                return
            if generation != self.home_generation or not self.home_pending:
                continue
            self.home_pending = False
            if self.stop_event.is_set() or self.env.paused:
                continue
            if not isinstance(result, str):
                measured = self.env.data.qpos[self.env.qids] if bus is None else bus.q
                if np.max(np.abs(measured - result['start'])) > np.deg2rad(.5):
                    result = 'Blocked: pose moved during planning; hold stable and press R again'
            self.control.joint_status = (result if isinstance(result, str) else
                                         self.env.begin_zero_return(result, speed, accel))
            print(self.control.joint_status, flush=True)

    def publish(self):
        e = self.env
        state = {name: getattr(e, name).copy() for name in
                 ('q_command', 'target_position', 'target_rotation')}
        state.update(qpos=e.data.qpos.copy(), qvel=e.data.qvel.copy(),
                     paused=e.paused, gripper_open=e.gripper_open, ik_error=e.ik_error,
                     success=e.success, control_mode=self.control.control_mode,
                     selected_joint=self.control.selected_joint, joint_status=self.control.joint_status,
                     motor=tuple(x.copy() for x in (e.motors.q, e.motors.tau, e.motors.applied_tau)))
        with self.lock:
            self.snapshot = state

    def check_cycle_deadline(self, now, deadline, returning):
        if deadline is None or now - deadline <= .1:
            return
        details = ', '.join(f'{k}={v:.1f}' for k, v in self.tick_timings.items())
        home = returning or self.env.zero_return is not None
        raise TimeoutError(f'控制循环延迟超过100ms：落后 {(now-deadline)*1000:.1f}ms；'
                           f'阶段耗时 {details}；回零中={home}。已中止运动')

    def tick(self, bus, deadline=None):
        e, c = self.env, self.control
        returning = e.zero_return is not None
        began = time.monotonic()
        self.tick_timings = {}
        if bus is not None:
            check_health(bus)
            e.zero_return_feedback = (bus.q.copy(), bus.dq.copy())
            if self.args.mode == 'sync':
                show_measured(e, bus.q, bus.dq)
        measured_at = time.monotonic()
        self.tick_timings['feedback_model_ms'] = 1000 * (measured_at - began)
        while True:
            try:
                key, action, mods = self.events.get_nowait()
            except queue.Empty:
                break
            if action == glfw.PRESS and key == glfw.KEY_C and bus is None:
                self.restore_object()
                continue
            if self.leader is not None and key == glfw.KEY_SPACE:
                if action == glfw.PRESS and not e.paused:
                    c.keys.add(key)
                elif action == glfw.RELEASE:
                    c.keys.discard(key)
                    self.release_leader()
                    c.joint_status = 'Hold SPACE to follow'
            elif key is None:
                c.keys.clear()
                if self.leader is not None:
                    self.release_leader()
                    c.joint_status = 'Hold SPACE to follow'
            elif self.leader is not None and key != glfw.KEY_P:
                continue
            elif key == glfw.KEY_R and action == glfw.PRESS:
                self.request_home(bus)
            else:
                if action == glfw.PRESS and key in (glfw.KEY_P, glfw.KEY_TAB):
                    self.cancel_home_planning()
                if self.home_pending:
                    continue
                handle_key(c, bus, key, action, mods)
        self.accept_home_result(bus)
        events_at = time.monotonic()
        self.tick_timings['keys_and_planning_ms'] = 1000 * (events_at - measured_at)
        if (bus is not None or self.leader is not None) and time.monotonic() - self.input_at > INPUT_TIMEOUT:
            c.keys.clear()
            if not e.paused:
                self.cancel_home_planning()
                hold_command(e)
                e.paused = True
                print('窗口超过0.5秒未处理输入，已暂停保持；窗口恢复后按 P 继续', flush=True)
        if self.leader is not None:
            self.follow_leader_gripper()
            leader_started = time.monotonic()
            self.update_leader(not e.paused)
            if e.paused or glfw.KEY_SPACE not in c.keys:
                self.release_leader()
            self.tick_timings['leader_update_ms'] = 1000 * (time.monotonic() - leader_started)
        elif not self.home_pending:
            c.controls(e.motors.command_period)
        if e.paused or self.home_pending:
            command = hold_command(e)
        elif self.args.mode == 'sim':
            e.step(e.motors.command_period)
            command = e.motors.host_command
        else:
            command = command_from_env(e)
        command_at = time.monotonic()
        self.tick_timings['reference_and_checks_ms'] = 1000 * (command_at - events_at)
        if bus is not None:
            if self.stop_event.is_set():
                return
            now = time.monotonic()
            dt = e.motors.command_period if self.last_compensation_at is None else now-self.last_compensation_at
            command = e.motors.compensate_hold(command, bus.q, dt,
                integrate=not e.paused and not self.home_pending and np.max(np.abs(command[1])) < 1e-6,
                unwind=not e.paused and not self.home_pending and np.max(np.abs(command[1])) >= 1e-6)
            self.last_compensation_at = now
            if e.motors.position_bias.enabled:
                e.motors.set_command(*command)
            check_command(e, bus, command)
            send_at = time.monotonic()
            self.tick_timings['command_guard_ms'] = 1000 * (send_at - command_at)
            self.check_cycle_deadline(send_at, deadline, returning)
            bus.send(*command)
            received_at = time.monotonic()
            self.tick_timings['bus_roundtrip_ms'] = 1000 * (received_at - send_at)
            check_health(bus)
            if self.args.mode == 'sync':
                show_measured(e, bus.q, bus.dq)
            e.motors.applied_tau = bus.tau.copy()
            self.tick_timings['feedback_display_ms'] = 1000 * (time.monotonic() - received_at)

    def release_leader(self):
        if self.leader_anchor is not None:
            self.leader_anchor = None
            hold_command(self.env)
            self.env.target_position, self.env.target_rotation = self.env.tcp_position, (
                self.env.data.site_xmat[self.env.tcp_id].reshape(3, 3).copy())
            self.env.update_marker()

    def run(self):
        bus = None
        try:
            if getattr(self.args, 'leader_port', None):
                self.gripper_sides = gripper_sides_deg(self.args.leader_calib)
                self.leader = LeaderReader(self.args.leader_python, self.args.leader_port, self.args.leader_calib)
                deadline = time.monotonic() + 15.
                while self.leader.sample() is None:
                    if self.stop_event.is_set():
                        return
                    if self.leader.process.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('未收到 SO-101 主臂角度，请检查串口、LeRobot 环境和标定')
                    self.stop_event.wait(.02)
                self.control.joint_status = 'Hold SPACE to follow'
            if self.args.hardware:
                bus = Bus(self.args.bridge, self.args.port, self.args.calib,
                          timeout=self.args.feedback_timeout)
                # Do not let the reference filter silently snap an invalid
                # calibrated measurement onto a model joint limit at startup.
                tolerance = np.deg2rad(.5)
                limits = self.env.limits
                outside = np.flatnonzero((bus.q < limits[:, 0] - tolerance) |
                                         (bus.q > limits[:, 1] + tolerance))
                if outside.size:
                    details = '；'.join(
                        f'J{j + 1} 实测 {np.rad2deg(bus.q[j]):+.3f}°，'
                        f'模型范围 [{np.rad2deg(limits[j, 0]):+.3f}°, '
                        f'{np.rad2deg(limits[j, 1]):+.3f}°]'
                        for j in outside)
                    raise RuntimeError(f'启动实测角超出模型范围（启动读数容差 ±0.5°）：{details}。'
                                       '请核对标定与实物姿态')
                reanchor(self.env, bus)
                self.enable(bus)
            else:
                if self.leader is None:
                    self.env.data.qpos[self.env.qids] = 0.
                self.env.data.qvel[:] = 0.
                mujoco.mj_forward(self.env.model, self.env.data)
                reanchor(self.env)
            self.publish()
            self.ready.set()
            mode_help = LEADER_HELP if self.leader is not None else (
                'Tab切换，关节模式按4再按↑可测试J4；P暂停/恢复。')
            if self.leader is not None and bus is not None:
                mode_help += f' 真机跟随目标限速 {self.args.joint_speed:g}°/秒。'
            print(f'控制已就绪：{"真机" if bus is not None else "纯仿真"} / '
                  f'{self.control.control_mode}；{mode_help}', flush=True)
            period = self.env.motors.command_period
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                returning = self.env.zero_return is not None
                deadline += period
                self.tick(bus, deadline if bus is not None else None)
                publish_at = time.monotonic()
                self.publish()
                self.tick_timings['snapshot_ms'] = 1000 * (time.monotonic() - publish_at)
                now = time.monotonic()
                if now > deadline:
                    # Never send catch-up bursts. A long missed deadline stops hardware.
                    if bus is not None:
                        self.check_cycle_deadline(now, deadline, returning)
                    deadline = now + period
                self.stop_event.wait(max(0., deadline - now))
        except Exception as exc:
            self.error = str(exc)
        finally:
            try:
                if bus is not None:
                    bus.stop()
            except Exception as exc:
                self.error = self.error or f'总线关闭失败：{exc}'
            finally:
                if self.leader is not None:
                    self.leader.close()
                self.ready.set()
                self.stop_event.set()

    def enable(self, bus):
        """Both hardware modes must hold measured pose with stable FOC before motion."""
        start = time.monotonic()
        stable = 0
        best_stable = 0
        attempts = 0
        non_foc = np.zeros(7, dtype=int)
        max_roundtrip = 0.
        command = hold_command(self.env)
        while not self.stop_event.is_set() and time.monotonic() - start < 2.:
            check_health(bus, require_foc=False)
            check_command(self.env, bus, command)
            sent_at = time.monotonic()
            bus.send(*command)
            max_roundtrip = max(max_roundtrip, time.monotonic() - sent_at)
            check_health(bus, require_foc=False)
            attempts += 1
            non_foc += bus.mode != 1
            stable = stable + 1 if np.all(bus.mode == 1) else 0
            best_stable = max(best_stable, stable)
            self.publish()
            if stable >= 20:
                reanchor(self.env, bus)
                return
            self.stop_event.wait(self.env.motors.command_period)
        missing = [f'J{j+1}(mode={int(bus.mode[j])},非FOC={int(non_foc[j])}次)'
                   for j in range(7) if non_foc[j] or bus.mode[j] != 1]
        raise RuntimeError('未能连续20周期保持所有关节 FOC，取消运动；'
                           f'耗时={time.monotonic()-start:.2f}s，采样={attempts}，'
                           f'最长连续FOC={best_stable}，最大总线往返={max_roundtrip*1000:.1f}ms；'
                           f'异常模式关节={missing}，最终mode={bus.mode.tolist()}，'
                           f'err={[hex(int(e)) for e in bus.err]}')


class RealKeyboardViewer(KeyboardViewer):
    reset_label = 'slow zero'

    def __init__(self, env, session):
        self.session = session
        super().__init__(env)
        if getattr(session.args, 'leader_port', None):
            self.control_help = ('SO-101 LEADER\nHold SPACE: follow | Release: hold\n'
                                 'C: put the plant back\n'
                                 'P: pause | ESC: exit\n'
                                 'Mouse: left orbit | right pan | wheel zoom')

    def on_key(self, window, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            self.session.stop_event.set()
            glfw.set_window_should_close(window, True)
            return
        if getattr(self.session.args, 'leader_port', None) and not self.session.ready.is_set():
            return
        self.session.events.put((key, action, mods))

    def display(self, state):
        if state is None:
            return
        e = self.env
        e.data.qpos[:] = state['qpos']
        e.data.qvel[:] = state['qvel']
        for name in ('q_command', 'target_position', 'target_rotation', 'paused',
                     'gripper_open', 'ik_error', 'success'):
            setattr(e, name, state[name])
        for name in ('control_mode', 'selected_joint', 'joint_status'):
            setattr(self, name, state[name])
        e.motors.q, e.motors.tau, e.motors.applied_tau = state['motor']
        e.update_marker()
        mujoco.mj_forward(e.model, e.data)
        mode = 'SO-101 | ' + self.joint_status if self.session.leader is not None else (
            'JOINT' if self.control_mode == 'joint' else 'IK')
        glfw.set_window_title(self.window, f'R1-7A | {self.session.args.mode} | {mode}')

    def close(self):
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()


def terminal_exit(session, stream):
    """Line-based terminal exit, independent of GLFW focus and event processing."""
    try:
        fd = stream.fileno()
    except (OSError, ValueError, AttributeError):
        for line in stream:
            if line.strip().lower() in ('q', 'quit', 'exit'):
                session.stop_event.set()
                break
        return
    try:
        # Avoid a daemon blocked inside buffered sys.stdin at interpreter shutdown.
        with selectors.DefaultSelector() as selector:
            selector.register(fd, selectors.EVENT_READ)
            pending = ''
            while not session.stop_event.is_set():
                if not selector.select(.1):
                    continue
                data = os.read(fd, 4096)
                if not data:
                    return
                pending += data.decode(errors='replace')
                while '\n' in pending:
                    line, pending = pending.split('\n', 1)
                    if line.strip().lower() in ('q', 'quit', 'exit'):
                        session.stop_event.set()
                        return
    except (OSError, ValueError):
        return


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=REAL_HELP + HARDWARE_NOTE,
                                    formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--hardware', action='store_true', help='明确连接并驱动真机')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--mode', choices=['sync', 'sim'], default='sim')
    parser.add_argument('--control-mode', choices=['cartesian', 'joint'], default='cartesian',
                        help='启动时使用 IK 或关节角控制；Tab 可随时切换')
    parser.add_argument('--calib', '--calibration', type=Path, default=SDK_CALIB)
    parser.add_argument('--bridge', type=Path, default=SDK_BIN)
    parser.add_argument('--motor-config', type=Path)
    parser.add_argument('--leader-port', help='SO-101 主臂串口，启用关节遥操作')
    parser.add_argument('--leader-mapping', choices=('relative', 'absolute'), default='relative',
                        help='主臂跟随方式，默认 relative')
    parser.add_argument('--leader-map', type=Path, help='绝对映射的五轴两点参考 JSON，角度单位为度')
    parser.add_argument('--leader-python', default=sys.executable, help='已安装 LeRobot 的 Python')
    parser.add_argument('--leader-calib', type=Path, default=Path.home() /
                        '.cache/huggingface/lerobot/calibration/teleoperators/so_leader/7vb_leader_arm.json')
    parser.add_argument('--leader-scale', type=float, help='相对模式角度比例，0 到 1，默认 0.35')
    parser.add_argument('--leader-signs', type=int, nargs=5, choices=(-1, 1),
                        help='相对模式五轴方向，顺序 J1 J2 J4 J6 J5；默认 1 1 -1 -1 1')
    parser.add_argument('--leader-travel-deg', type=float, help='相对模式纯仿真单次跟随最大角度，默认 12°')
    parser.add_argument('--auto-ik', action='store_true', help='自动测试 XYZ 平移并比较反馈正运动学和独立仿真')
    parser.add_argument('--ik-highest', action='store_true', help='自动 IK 测试仅沿 Z 上升到当前直线路径的最高可达点')
    parser.add_argument('--ik-distance-mm', type=float, default=5., help='自动 IK 单轴位移，1 至 15 mm')
    parser.add_argument('--ik-output', type=Path, help='自动 IK 报告目录，必须是新目录')
    parser.add_argument('--ik-hold-seconds', type=float, default=4., help='自动 IK 到点后保持时间（秒）')
    parser.add_argument('--joint-speed', type=float,
                        help=f'关节目标速度，度/秒；真机主臂遥操作默认 {DEFAULT_LEADER_HARDWARE_SPEED_DEG:g}，'
                             f'最高 {MAX_LEADER_HARDWARE_SPEED_DEG:g}，其他模式默认 20')
    parser.add_argument('--home-speed-deg', type=float, default=10., help='R回零峰值速度，度/秒')
    parser.add_argument('--home-accel-deg', type=float, default=20., help='R回零最大加速度，度/秒²')
    parser.add_argument('--feedback-timeout', type=float, default=.1, help='总线响应超时，秒')
    args = parser.parse_args(argv)
    if args.joint_speed is None:
        args.joint_speed = DEFAULT_LEADER_HARDWARE_SPEED_DEG if args.hardware and args.leader_port else 20.
    for name in ('joint_speed', 'feedback_timeout', 'home_speed_deg', 'home_accel_deg'):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} 必须为正的有限数值')
    if args.feedback_timeout > .5:
        parser.error('--feedback-timeout 不能超过0.5秒')
    if args.mode == 'sync' and not args.hardware:
        parser.error('--mode sync 需要 --hardware；纯仿真使用 --mode sim')
    if args.ik_highest:
        args.auto_ik = True
    args.leader_absolute_points = None
    if args.leader_map is not None and args.leader_mapping != 'absolute':
        parser.error('--leader-map 需要 --leader-mapping absolute')
    if args.leader_mapping == 'absolute':
        if not args.leader_port:
            parser.error('--leader-mapping absolute 需要 --leader-port')
        if any(value is not None for value in (args.leader_scale, args.leader_signs, args.leader_travel_deg)):
            parser.error('绝对映射的方向和比例由映射表定义，请移除 --leader-scale、--leader-signs、--leader-travel-deg')
        try:
            path = args.leader_map.expanduser() if args.leader_map is not None else None
            args.leader_absolute_points = load_absolute_points(path)
        except (OSError, ValueError, TypeError) as exc:
            parser.error(f'无法读取绝对映射：{exc}')
    if args.leader_travel_deg is not None and (
            not args.leader_port or args.hardware or not 0 < args.leader_travel_deg <= 180):
        parser.error('--leader-travel-deg 仅用于主臂纯仿真模式，范围为 (0, 180] 度')
    if args.leader_port:
        if args.hardware and args.joint_speed > MAX_LEADER_HARDWARE_SPEED_DEG:
            parser.error(f'真机主臂遥操作的 --joint-speed 最大为 {MAX_LEADER_HARDWARE_SPEED_DEG:g}°/秒')
        if args.leader_scale is None:
            args.leader_scale = .35
        if args.leader_signs is None:
            args.leader_signs = DEFAULT_SIGNS
        if args.leader_travel_deg is None:
            args.leader_travel_deg = DEFAULT_TRAVEL_DEG
        if args.auto_ik or (args.hardware and args.mode != 'sync'):
            parser.error('主臂模式与自动 IK 互斥，真机主臂模式需要 --mode sync')
        if not 0 < args.leader_scale <= 1 or not args.leader_calib.expanduser().is_file():
            parser.error('主臂比例必须大于 0 且不超过 1，并提供有效标定文件')
        args.leader_calib = args.leader_calib.expanduser().resolve()
        if not os.path.exists(args.leader_port):
            parser.error(f'主臂串口不存在：{args.leader_port}')
        if args.hardware and os.path.exists(args.port) and os.path.samefile(args.leader_port, args.port):
            parser.error('主臂与 R1-7A 必须使用不同串口')
        args.control_mode = 'joint'
    if args.auto_ik:
        if not np.isfinite(args.ik_hold_seconds) or args.ik_hold_seconds < .3:
            parser.error('--ik-hold-seconds 必须为有限值且不少于 0.3 秒')
        if args.control_mode != 'cartesian' or (args.hardware and args.mode != 'sync'):
            parser.error('--auto-ik 需要笛卡尔控制，真机使用 --mode sync')
        if not 1 <= args.ik_distance_mm <= 15:
            parser.error('--ik-distance-mm 必须在 1 到 15 mm 之间')
    args.calib = args.calib.expanduser().resolve()
    args.bridge = args.bridge.expanduser().resolve()
    if not args.calib.is_file():
        parser.error(f'标定文件不存在：{args.calib}')
    if args.hardware:
        if not os.path.exists(args.port):
            parser.error(f'没有串口：{args.port}')
        if not args.bridge.is_file() or not os.access(args.bridge, os.X_OK):
            parser.error(f'桥接程序不可执行：{args.bridge}；请编译仓库内的 demo_cmd_stream')
    return args


def main():
    args = parse_args()
    if args.auto_ik:
        from auto_joint_test import parse_args as parse_test_args
        from joint_test_viewer import run_viewer
        move_seconds = max(3., 3. * args.ik_distance_mm / 5.)
        options = ['--view', '--close-on-finish', '--port', args.port, '--bridge', str(args.bridge),
                   '--calib', str(args.calib), '--feedback-timeout', str(args.feedback_timeout),
                   '--move-seconds', str(move_seconds), '--hold-seconds', str(args.ik_hold_seconds), '--cycles', '1']
        if args.hardware:
            options.append('--hardware')
        if args.motor_config is not None:
            options += ['--motor-config', str(args.motor_config)]
        test = parse_test_args(options)
        test.ik_distance_mm = args.ik_distance_mm
        test.ik_highest = args.ik_highest
        test.output = args.ik_output or Path('outputs') / test.output.name.replace('joint_test_', 'ik_test_', 1)
        raise SystemExit(run_viewer(test))
    initial_pose = ('clearance' if args.leader_port and args.leader_mapping == 'relative' and not args.hardware
                    else 'zero')
    env = ArmEnvironment(motor_config=args.motor_config, calibration_path=args.calib,
                         initial_pose=initial_pose)
    display_env = ArmEnvironment(motor_config=args.motor_config, calibration_path=args.calib,
                                 initial_pose=initial_pose)
    session = Session(env, args)
    viewer = RealKeyboardViewer(display_env, session)
    glfw.set_window_focus_callback(viewer.window, lambda window, focused:
                                   session.events.put((None, None, None)) if not focused else None)
    print(LEADER_HELP if args.leader_port else REAL_HELP + HARDWARE_NOTE, flush=True)
    print('标定文件:', args.calib, '\n硬件连接:', args.hardware, flush=True)
    try:
        session.thread.start()
        if sys.stdin.isatty():
            threading.Thread(target=terminal_exit, args=(session, sys.stdin),
                             name='r1-terminal-exit', daemon=True).start()
        while not glfw.window_should_close(viewer.window) and not session.stop_event.is_set():
            glfw.poll_events()
            session.input_at = time.monotonic()
            with session.lock:
                state = session.snapshot
            viewer.display(state)
            viewer.render()
    finally:
        session.stop_event.set()
        if session.thread.ident is not None:
            session.thread.join(timeout=7.)
        viewer.close()
    if session.error:
        sys.exit(f'控制已停止：{session.error}')


if __name__ == '__main__':
    main()
