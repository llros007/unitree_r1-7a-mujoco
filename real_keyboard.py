#!/usr/bin/env python3
"""Real-arm teleop through the same keyboard controller as ./run.sh.

Keys, Cartesian IK, and joint-angle jog live in main.KeyboardViewer.
This process only sends that controller's q/dq/tau/kp/kd to the arm and
draws the measured joints in the same window.
"""
import os
import sys

# Conda's libstdc++ is too old for the system OpenGL driver. Reload once so
# the MuJoCo window can be created.
_SYSTEM_LIBCXX = '/usr/lib/x86_64-linux-gnu/libstdc++.so.6'
_SYSTEM_DRI = '/usr/lib/x86_64-linux-gnu/dri'
if os.path.isfile(_SYSTEM_LIBCXX) and os.environ.get('_R1_GL_READY') != '1':
    os.environ['_R1_GL_READY'] = '1'
    os.environ['LIBGL_DRIVERS_PATH'] = _SYSTEM_DRI
    preload = os.environ.get('LD_PRELOAD', '')
    if _SYSTEM_LIBCXX not in preload.split():
        os.environ['LD_PRELOAD'] = (_SYSTEM_LIBCXX + ' ' + preload).strip()
    os.execv(sys.executable, [sys.executable, *sys.argv])

import argparse
import subprocess
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment
from main import HELP as SIM_HELP
from main import KeyboardViewer
from xle_gripper import CLOSE_ANGLE, OPEN_ANGLE

ROOT = Path(__file__).resolve().parent


def _pick(*candidates):
    for path in candidates:
        if path.is_file():
            return path
    return candidates[-1]


SDK_BIN = _pick(
    ROOT.parent / 'r1-7a_sdk' / 'bin' / 'demo_cmd_stream',
    ROOT / 'r1-7a_sdk' / 'bin' / 'demo_cmd_stream')
SDK_CALIB = _pick(
    ROOT.parent / 'r1-7a_sdk' / 'config' / 'joint_calib.txt',
    ROOT / 'r1-7a_sdk' / 'config' / 'joint_calib.txt')
TRACK_ABORT_RAD = 0.40
HARDWARE_NOTE = """
按键和 ./run.sh 相同，请点在 MuJoCo 窗口里。
  --mode sync  画面每帧改成真机实测，真机执行同一套目标（默认，需要接上机械臂）
  --mode sim   仿真自己解 IK、自己积分。没接机械臂只跑仿真；接上时真机执行这套目标
标定文件与真机相同：旁边的 r1-7a_sdk/config/joint_calib.txt。
Space 在 sync 下只切换画面夹爪，不带动夹爪舵机。
R 回到本次启动姿态。
"""


def parse_state(line):
    parts = line.split()
    if len(parts) != 1 + 35 or parts[0] != 'STATE':
        raise RuntimeError(f'总线状态行无法解析: {line!r}')
    values = np.array([float(x) for x in parts[1:]], dtype=float)
    q, dq, tau = values[0:7], values[7:14], values[14:21]
    mode = values[21:28].astype(int)
    err = values[28:35].astype(np.uint32)
    return q, dq, tau, mode, err


class Bus:
    def __init__(self, binary, port, calib):
        self.proc = subprocess.Popen(
            [str(binary), '--port', port, '--calib', str(calib)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        try:
            self.q, self.dq, self.tau, self.mode, self.err = self._read_state()
        except Exception:
            self.stop()
            raise

    def _read_state(self):
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError('总线进程没有返回状态')
            if line.startswith('STATE '):
                return parse_state(line)
            print(line, end='', flush=True)

    def send(self, q, dq, tau, kp, kd):
        row = np.concatenate([q, dq, tau, kp, kd])
        self.proc.stdin.write(' '.join(f'{v:.8g}' for v in row) + '\n')
        self.proc.stdin.flush()
        self.q, self.dq, self.tau, self.mode, self.err = self._read_state()

    def stop(self):
        if self.proc.poll() is not None:
            return
        try:
            self.proc.stdin.write('STOP\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


class RealKeyboardViewer(KeyboardViewer):
    """Same keys as ./run.sh. R re-anchors to the measured arm instead of resetting."""

    def __init__(self, env):
        super().__init__(env, control_mode='cartesian', joint_speed=20.)
        self.reanchor = None
        self._closed = False
        glfw.set_window_title(self.window, 'R1-7A real | IK (Cartesian)')

    def on_key(self, window, key, scancode, action, mods):
        if action == glfw.PRESS and key == glfw.KEY_R:
            self.keys.clear()
            if self.reanchor is not None:
                self.reanchor()
            self.joint_status = 'Hold measured pose'
            return
        super().on_key(window, key, scancode, action, mods)
        if action == glfw.PRESS and key == glfw.KEY_TAB:
            mode = 'JOINT ANGLES' if self.control_mode == 'joint' else 'IK (Cartesian)'
            glfw.set_window_title(self.window, f'R1-7A real | {mode}')

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()


def adopt_pose(env, q):
    """Put the simulated arm on a joint vector and rebase IK on that pose."""
    q = np.array(q, dtype=float, copy=True)
    env.data.qpos[env.qids] = q
    env.data.qvel[env.dofs] = 0.
    mujoco.mj_forward(env.model, env.data)
    env.use_cartesian_control()
    env.hardware_q = q.copy()


def show_measured(env, q, dq):
    env.data.qpos[env.qids] = q
    env.data.qvel[env.dofs] = dq
    env.data.qpos[env.gripper_q] = OPEN_ANGLE if env.gripper_open else CLOSE_ANGLE
    mujoco.mj_forward(env.model, env.data)


def command_from_env(env):
    """Same outer-loop command ./run.sh holds between physics steps."""
    q, dq = env.reference.update(env.q_command, env.motors.command_period)
    tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q)
           if env.motors.gravity_feedforward_scale else np.zeros(7))
    env.motors.set_command(q, dq, tau)
    return env.motors.host_command


def hold_measured(env, bus, viewer):
    """Hold the measured joints until FOC stays up. Do not move toward another pose."""
    show_measured(env, bus.q, bus.dq)
    env.use_cartesian_control()
    print('停在当前实测姿态。按键在 MuJoCo 窗口里，和 ./run.sh 相同。', flush=True)
    print('真机 / 画面关节角 deg', np.degrees(bus.q).round(1), flush=True)
    stable = 0
    sent = 0
    period = env.motors.command_period
    next_tick = time.monotonic()
    deadline = next_tick + 2.0
    while not glfw.window_should_close(viewer.window):
        now = time.monotonic()
        if now < next_tick:
            time.sleep(max(0., next_tick - now))
        next_tick += period
        glfw.poll_events()
        if glfw.KEY_ESCAPE in viewer.keys:
            return False
        q, dq, tau, kp, kd = env.motors.host_command
        bus.send(q, np.zeros(7), np.zeros(7), kp, kd)
        sent += 1
        show_measured(env, bus.q, bus.dq)
        env.motors.applied_tau = np.array(bus.tau, dtype=float, copy=True)
        viewer.render()
        if np.any(bus.err != 0):
            print('电机报错 mode=', bus.mode, 'err=', [hex(int(e)) for e in bus.err], flush=True)
            return False
        if np.all(bus.mode == 1):
            stable += 1
            if stable >= 20:
                print('7 个关节保持 FOC，停在 deg', np.degrees(bus.q).round(1), flush=True)
                return True
        else:
            stable = 0
        if sent > 200 or now > deadline:
            stopped = [i for i, mode in enumerate(bus.mode) if mode != 1]
            print(f'关节 {stopped} 没有保持 FOC mode={bus.mode} err={[hex(int(e)) for e in bus.err]}', flush=True)
            print('先断电再上电，然后重试。', flush=True)
            return False
    return False


def main():
    parser = argparse.ArgumentParser(description=SIM_HELP + HARDWARE_NOTE,
                                    formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='真机串口。两种模式都会用它读初始角度，默认同 /dev/ttyUSB0')
    parser.add_argument('--mode', choices=['sync', 'sim'], default='sync',
                        help='sync: 画面跟真机实测；sim: 仿真自己解 IK，真机接上时也执行这套目标')
    parser.add_argument('--calib', type=Path, default=SDK_CALIB)
    parser.add_argument('--bridge', type=Path, default=SDK_BIN)
    parser.add_argument('--motor-config', type=Path)
    parser.add_argument('--joint-speed', type=float, default=20.,
                        help='关节角模式速度，度/秒，与 ./run.sh 相同，默认 20')
    args = parser.parse_args()
    if not np.isfinite(args.joint_speed) or args.joint_speed <= 0:
        parser.error('--joint-speed 必须为正的有限数值（度/秒）')
    port_ready = os.path.exists(args.port)
    if args.mode == 'sync' and not port_ready:
        sys.exit(f'没有 {args.port}。只跑仿真请用: .venv/bin/python real_keyboard.py --mode sim')
    if port_ready and not args.bridge.is_file():
        sys.exit(f'缺少 {args.bridge}，请先在 r1-7a_sdk/build 里 cmake && make demo_cmd_stream')

    print('加载和 ./run.sh 相同的模型与键盘控制……', flush=True)
    env = ArmEnvironment(motor_config=args.motor_config, calibration_path=args.calib)
    print('kp=', env.motors.default_kp, 'kd=', env.motors.default_kd,
          'gravity_scale=', env.motors.gravity_feedforward_scale, flush=True)
    print(SIM_HELP, flush=True)
    print(HARDWARE_NOTE, flush=True)
    viewer = RealKeyboardViewer(env)
    viewer.joint_speed = np.deg2rad(args.joint_speed)
    bus = Bus(args.bridge, args.port, args.calib) if port_ready else None

    def reanchor():
        if args.mode == 'sim':
            adopt_pose(env, env.hardware_q)
            print('仿真回到启动时的真机姿态 deg', np.degrees(env.hardware_q).round(1), flush=True)
            return
        show_measured(env, bus.q, bus.dq)
        env.use_cartesian_control()
        print('已用实测重新锚定 deg', np.degrees(bus.q).round(1), flush=True)

    viewer.reanchor = reanchor
    period = env.motors.command_period
    try:
        if args.mode == 'sim':
            if bus is not None:
                adopt_pose(env, bus.q)
                print('仿真从真机当前角度开始 deg', np.degrees(env.hardware_q).round(1), flush=True)
                print('之后仿真自己解 IK，同一套目标发给真机。画面不跟实测回写。', flush=True)
            else:
                adopt_pose(env, np.zeros(7))
                print('没有真机，仿真从标定零位自己解 IK。', flush=True)
                print('标定文件与真机相同:', args.calib, flush=True)
            print('按键在 MuJoCo 窗口里。', flush=True)
            previous = time.monotonic()
            accumulator = 0.
            status_at = 0.
            while not glfw.window_should_close(viewer.window):
                now = time.monotonic()
                accumulator += min(now - previous, 0.1)
                previous = now
                glfw.poll_events()
                while accumulator >= period:
                    viewer.controls(period)
                    env.step(period)
                    if bus is not None:
                        q_cmd, dq_cmd, tau, kp, kd = env.motors.host_command
                        if np.max(np.abs(q_cmd - bus.q)) > TRACK_ABORT_RAD:
                            print(f'真机跟踪误差超过 {TRACK_ABORT_RAD} rad，停机', flush=True)
                            glfw.set_window_should_close(viewer.window, True)
                            break
                        bus.send(q_cmd, dq_cmd, tau, kp, kd)
                        if np.any(bus.err != 0):
                            print('电机报错 mode=', bus.mode, 'err=', [hex(int(e)) for e in bus.err], flush=True)
                            glfw.set_window_should_close(viewer.window, True)
                            break
                    accumulator -= period
                viewer.render()
                if bus is not None and now - status_at > 0.2:
                    status_at = now
                    q_cmd = env.motors.host_command[0]
                    err = np.max(np.abs(q_cmd - bus.q))
                    print(f'仿真 IK  track={err:.3f} rad  '
                          f'mode={bus.mode}  '
                          f'{"JOINT" if viewer.control_mode == "joint" else "IK"}', flush=True)
            return
        if not hold_measured(env, bus, viewer):
            return
        reanchor()
        print('TCP m =', env.target_position.round(3), '可以在窗口里按键', flush=True)
        next_tick = time.monotonic()
        status_at = 0.
        sent = 0
        while not glfw.window_should_close(viewer.window):
            now = time.monotonic()
            if now < next_tick:
                time.sleep(max(0., next_tick - now))
            next_tick += period
            if time.monotonic() - next_tick > 0.05:
                next_tick = time.monotonic()
            show_measured(env, bus.q, bus.dq)
            glfw.poll_events()
            if glfw.KEY_ESCAPE in viewer.keys:
                break
            viewer.controls(period)
            if env.paused:
                q_cmd = env.reference.q.copy()
                dq_cmd = np.zeros(7)
                tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q_cmd)
                       if env.motors.gravity_feedforward_scale else np.zeros(7))
                env.motors.set_command(q_cmd, dq_cmd, tau)
                q_cmd, dq_cmd, tau, kp, kd = env.motors.host_command
            else:
                q_cmd, dq_cmd, tau, kp, kd = command_from_env(env)
            if np.max(np.abs(q_cmd - bus.q)) > TRACK_ABORT_RAD:
                print(f'跟踪误差超过 {TRACK_ABORT_RAD} rad，停机', flush=True)
                break
            bus.send(q_cmd, dq_cmd, tau, kp, kd)
            sent += 1
            show_measured(env, bus.q, bus.dq)
            env.motors.applied_tau = np.array(bus.tau, dtype=float, copy=True)
            viewer.render()
            if np.any(bus.err != 0) or (sent > 30 and np.any(bus.mode != 1)):
                print('电机退出 FOC 或报错 mode=', bus.mode, 'err=', [hex(int(e)) for e in bus.err], flush=True)
                break
            now = time.monotonic()
            if now - status_at > 0.2:
                status_at = now
                err = np.max(np.abs(q_cmd - bus.q))
                print(f'TCP {env.target_position.round(3)}  ik={env.ik_error:.4f}  '
                      f'track={err:.3f} rad  tau={np.array2string(np.asarray(tau), precision=1)}  '
                      f'{"PAUSE" if env.paused else "RUN"}  '
                      f'{"JOINT" if viewer.control_mode == "joint" else "IK"}', flush=True)
    finally:
        if bus is not None:
            bus.stop()
        viewer.close()
        print('已停机', flush=True)


if __name__ == '__main__':
    main()
