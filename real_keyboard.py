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
import queue
import selectors
import subprocess
import threading
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment
from main import HELP as SIM_HELP, KeyboardViewer
from xle_gripper import CLOSE_ANGLE, OPEN_ANGLE

ROOT = Path(__file__).resolve().parent
# Never change model/calibration just because a sibling checkout exists.
SDK_BIN = ROOT / 'r1-7a_sdk/bin/demo_cmd_stream'
SDK_CALIB = ROOT / 'r1-7a_sdk/config/joint_calib.txt'
TRACK_ABORT_RAD = .40
INPUT_TIMEOUT = .5
REAL_HELP = (SIM_HELP
             .replace('--control-mode joint:', 'Tab切换至关节模式：')
             .replace('默认从七关节全零姿态启动；', '纯仿真从零位启动，真机从实测姿态启动；'))
HARDWARE_NOTE = """
默认只跑仿真，不打开串口。真机必须显式添加 --hardware。
  --mode sync --hardware：控制器使用真机实测，画面同步实测。
  --mode sim：独立仿真；加 --hardware 时将仿真参考目标发给真机。
R：从当前姿态缓慢回到七关节零位，到位后保持；默认峰值5度/秒。
Tab：取消当前运动并保持当前位置，切换 IK / 关节模式。
P：暂停并清零目标速度，继续保持；恢复时不会追赶暂停前的目标。
Space：只控制仿真夹爪/显示标记，不驱动真机夹爪舵机。
默认使用仓库内部标定；--calib 可明确选择其他文件。
退出或故障会停驱动，机械臂可能失去主动支撑。
"""


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
    tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q)
           if env.motors.gravity_feedforward_scale else np.zeros(7))
    env.motors.set_command(q, dq, tau)
    return env.motors.host_command


def check_health(bus, require_foc=True):
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
        self.control = make_controller(env, args.joint_speed)
        self.control.control_mode = getattr(args, 'control_mode', 'cartesian')
        self.control.home_speed_deg = getattr(args, 'home_speed_deg', 5.)
        self.control.home_accel_deg = getattr(args, 'home_accel_deg', 10.)
        self.events = queue.Queue()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self.snapshot = None
        self.error = None
        self.input_at = time.monotonic()
        self.thread = threading.Thread(target=self.run, name='r1-control', daemon=True)

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

    def tick(self, bus):
        e, c = self.env, self.control
        if bus is not None:
            check_health(bus)
            e.zero_return_feedback = (bus.q.copy(), bus.dq.copy())
            if self.args.mode == 'sync':
                show_measured(e, bus.q, bus.dq)
        while True:
            try:
                key, action, mods = self.events.get_nowait()
            except queue.Empty:
                break
            if key is None:
                c.keys.clear()
            else:
                handle_key(c, bus, key, action, mods)
        if bus is not None and time.monotonic() - self.input_at > INPUT_TIMEOUT:
            c.keys.clear()
            if not e.paused:
                hold_command(e)
                e.paused = True
                print('窗口超过0.5秒未处理输入，真机已暂停保持；窗口恢复后按 P 继续', flush=True)
        c.controls(e.motors.command_period)
        if e.paused:
            command = hold_command(e)
        elif self.args.mode == 'sim':
            e.step(e.motors.command_period)
            command = e.motors.host_command
        else:
            command = command_from_env(e)
        if bus is not None:
            if self.stop_event.is_set():
                return
            check_command(e, bus, command)
            bus.send(*command)
            check_health(bus)
            if self.args.mode == 'sync':
                show_measured(e, bus.q, bus.dq)
            e.motors.applied_tau = bus.tau.copy()

    def run(self):
        bus = None
        try:
            if self.args.hardware:
                bus = Bus(self.args.bridge, self.args.port, self.args.calib,
                          timeout=self.args.feedback_timeout)
                # Do not let the reference filter silently snap an invalid
                # calibrated measurement onto a model joint limit at startup.
                tolerance = np.deg2rad(.5)
                if (np.any(bus.q < self.env.limits[:, 0] - tolerance)
                        or np.any(bus.q > self.env.limits[:, 1] + tolerance)):
                    raise RuntimeError('启动实测角超出模型范围，请先核对标定与实物姿态')
                reanchor(self.env, bus)
                self.enable(bus)
            else:
                self.env.data.qpos[self.env.qids] = 0.
                self.env.data.qvel[:] = 0.
                mujoco.mj_forward(self.env.model, self.env.data)
                reanchor(self.env)
            self.publish()
            self.ready.set()
            print(f'控制已就绪：{"真机" if bus is not None else "纯仿真"} / '
                  f'{self.control.control_mode}；请点击 MuJoCo 窗口后操作。'
                  'Tab切换，关节模式按4再按↑可测试J4；P暂停/恢复。', flush=True)
            period = self.env.motors.command_period
            deadline = time.monotonic()
            while not self.stop_event.is_set():
                self.tick(bus)
                self.publish()
                deadline += period
                now = time.monotonic()
                if now > deadline:
                    # Never send catch-up bursts. A long missed deadline stops hardware.
                    if bus is not None and now - deadline > .1:
                        raise TimeoutError('控制循环延迟超过100ms')
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
                self.ready.set()
                self.stop_event.set()

    def enable(self, bus):
        """Both hardware modes must hold measured pose with stable FOC before motion."""
        start = time.monotonic()
        stable = 0
        command = hold_command(self.env)
        while not self.stop_event.is_set() and time.monotonic() - start < 2.:
            check_health(bus, require_foc=False)
            check_command(self.env, bus, command)
            bus.send(*command)
            check_health(bus, require_foc=False)
            stable = stable + 1 if np.all(bus.mode == 1) else 0
            self.publish()
            if stable >= 20:
                reanchor(self.env, bus)
                return
            self.stop_event.wait(self.env.motors.command_period)
        raise RuntimeError('未能连续20周期保持所有关节 FOC，取消运动')


class RealKeyboardViewer(KeyboardViewer):
    reset_label = 'slow zero'

    def __init__(self, env, session):
        self.session = session
        super().__init__(env)

    def on_key(self, window, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            self.session.stop_event.set()
            glfw.set_window_should_close(window, True)
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
        mode = 'JOINT' if self.control_mode == 'joint' else 'IK'
        glfw.set_window_title(self.window, f'R1-7A | {self.session.args.mode} | {mode}')

    def close(self):
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()


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
    parser.add_argument('--joint-speed', type=float, default=20., help='关节速度，度/秒')
    parser.add_argument('--home-speed-deg', type=float, default=5., help='R回零峰值速度，度/秒')
    parser.add_argument('--home-accel-deg', type=float, default=10., help='R回零最大加速度，度/秒²')
    parser.add_argument('--feedback-timeout', type=float, default=.1, help='总线响应超时，秒')
    args = parser.parse_args(argv)
    for name in ('joint_speed', 'feedback_timeout', 'home_speed_deg', 'home_accel_deg'):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} 必须为正的有限数值')
    if args.feedback_timeout > .5:
        parser.error('--feedback-timeout 不能超过0.5秒')
    if args.mode == 'sync' and not args.hardware:
        parser.error('--mode sync 需要 --hardware；纯仿真使用 --mode sim')
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
    env = ArmEnvironment(motor_config=args.motor_config, calibration_path=args.calib,
                         initial_pose='zero')
    display_env = ArmEnvironment(motor_config=args.motor_config, calibration_path=args.calib,
                                 initial_pose='zero')
    session = Session(env, args)
    viewer = RealKeyboardViewer(display_env, session)
    glfw.set_window_focus_callback(viewer.window, lambda window, focused:
                                   session.events.put((None, None, None)) if not focused else None)
    print(REAL_HELP + HARDWARE_NOTE, flush=True)
    print('标定文件:', args.calib, '\n硬件连接:', args.hardware, flush=True)
    try:
        session.thread.start()
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
