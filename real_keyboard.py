#!/usr/bin/env python3
"""Real R1-7A Cartesian keyboard teleop using the simulation IK and motor parameters.

The 100 Hz command is the same five-tuple the simulator holds between updates:
q, dq, tau (gravity feedforward), kp, kd from motor_config.json.
Joint PD runs in the motor firmware. This process does not integrate a second plant.
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
import ctypes
import select
import subprocess
import termios
import time
from pathlib import Path

import mujoco
import numpy as np

from environment import ArmEnvironment

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
HELP = """
R1-7A 真机笛卡尔键盘（与仿真同一套 IK / kp / kd / 重力前馈）
  W/S: 世界坐标 Y +/-     A/D: 世界坐标 X -/+
  Q/E: 世界坐标 Z +/-     Shift: 慢速微调
  I/K: 绕 X 轴旋转       J/L: 绕 Y 轴旋转       U/O: 绕 Z 轴旋转
  Space: 夹爪开/合标记（这条 485 总线不带动夹爪舵机）
  R: 用当前实测重新锚定，保持不动
  P: 暂停笛卡尔增量，继续保持上一帧指令
  Esc: 停机退出
松开按键后目标不再移动。跟踪误差过大或电机报错会停机。
"""
TRACK_ABORT_RAD = 0.40

# X11 keysyms. Same letters as the MuJoCo viewer.
KEY_A, KEY_D = 0x61, 0x64
KEY_W, KEY_S = 0x77, 0x73
KEY_Q, KEY_E = 0x71, 0x65
KEY_I, KEY_K = 0x69, 0x6b
KEY_J, KEY_L = 0x6a, 0x6c
KEY_U, KEY_O = 0x75, 0x6f
KEY_R, KEY_P = 0x72, 0x70
KEY_SPACE = 0x20
KEY_ESC = 0xff1b
KEY_LEFT_SHIFT, KEY_RIGHT_SHIFT = 0xffe1, 0xffe2
KEY_LEFT, KEY_UP, KEY_RIGHT, KEY_DOWN = 0xff51, 0xff52, 0xff53, 0xff54
_ARROW_TO = {KEY_LEFT: KEY_A, KEY_UP: KEY_W, KEY_RIGHT: KEY_D, KEY_DOWN: KEY_S}
_WATCHED = {
    KEY_A, KEY_D, KEY_W, KEY_S, KEY_Q, KEY_E, KEY_I, KEY_K, KEY_J, KEY_L,
    KEY_U, KEY_O, KEY_R, KEY_P, KEY_SPACE, KEY_ESC, KEY_LEFT_SHIFT, KEY_RIGHT_SHIFT,
    KEY_LEFT, KEY_UP, KEY_RIGHT, KEY_DOWN,
}
_KEY_PRESS, _KEY_RELEASE, _FOCUS_OUT = 2, 3, 10


class _XKeyEvent(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int),
        ('serial', ctypes.c_ulong),
        ('send_event', ctypes.c_int),
        ('display', ctypes.c_void_p),
        ('window', ctypes.c_ulong),
        ('root', ctypes.c_ulong),
        ('subwindow', ctypes.c_ulong),
        ('time', ctypes.c_ulong),
        ('x', ctypes.c_int), ('y', ctypes.c_int),
        ('x_root', ctypes.c_int), ('y_root', ctypes.c_int),
        ('state', ctypes.c_uint),
        ('keycode', ctypes.c_uint),
        ('same_screen', ctypes.c_int),
    ]


class _XEvent(ctypes.Union):
    _fields_ = [('key', _XKeyEvent), ('pad', ctypes.c_long * 24)]


class HeldKeys:
    """Focusable X11 window. No OpenGL, so conda's old libstdc++ cannot block it."""

    def __init__(self):
        self.x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XDefaultScreen.restype = ctypes.c_int
        self.x11.XDefaultScreen.argtypes = [ctypes.c_void_p]
        self.x11.XRootWindow.restype = ctypes.c_ulong
        self.x11.XRootWindow.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XBlackPixel.restype = ctypes.c_ulong
        self.x11.XBlackPixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XWhitePixel.restype = ctypes.c_ulong
        self.x11.XWhitePixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.x11.XCreateSimpleWindow.restype = ctypes.c_ulong
        self.x11.XCreateSimpleWindow.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
            ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong]
        self.x11.XStoreName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_char_p]
        self.x11.XSelectInput.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_long]
        self.x11.XMapWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XPending.argtypes = [ctypes.c_void_p]
        self.x11.XPending.restype = ctypes.c_int
        self.x11.XNextEvent.argtypes = [ctypes.c_void_p, ctypes.POINTER(_XEvent)]
        self.x11.XPeekEvent.argtypes = [ctypes.c_void_p, ctypes.POINTER(_XEvent)]
        self.x11.XKeycodeToKeysym.restype = ctypes.c_ulong
        self.x11.XKeycodeToKeysym.argtypes = [ctypes.c_void_p, ctypes.c_ubyte, ctypes.c_int]
        self.x11.XDestroyWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('无法打开 X11 显示，请在桌面终端里运行')
        screen = self.x11.XDefaultScreen(self.display)
        root = self.x11.XRootWindow(self.display, screen)
        self.window = self.x11.XCreateSimpleWindow(
            self.display, root, 80, 80, 640, 160, 1,
            self.x11.XBlackPixel(self.display, screen),
            self.x11.XWhitePixel(self.display, screen))
        self.x11.XStoreName(self.display, self.window, b'R1-7A real keyboard')
        self.x11.XSelectInput(self.display, self.window, (1 << 0) | (1 << 1) | (1 << 21))
        self.x11.XMapWindow(self.display, self.window)
        self.x11.XFlush(self.display)
        self.keys = set()
        self.pressed = []
        self.quit = False

    def _keysym(self, keycode):
        return int(self.x11.XKeycodeToKeysym(self.display, keycode, 0))

    def _repeat(self, event):
        if not self.x11.XPending(self.display):
            return False
        nxt = _XEvent()
        self.x11.XPeekEvent(self.display, ctypes.byref(nxt))
        if (nxt.key.type == _KEY_PRESS and nxt.key.keycode == event.key.keycode
                and nxt.key.time == event.key.time):
            self.x11.XNextEvent(self.display, ctypes.byref(nxt))
            return True
        return False

    def poll(self):
        event = _XEvent()
        while self.x11.XPending(self.display):
            self.x11.XNextEvent(self.display, ctypes.byref(event))
            if event.key.type == _FOCUS_OUT:
                self.keys.clear()
                continue
            if event.key.type not in (_KEY_PRESS, _KEY_RELEASE):
                continue
            if event.key.type == _KEY_RELEASE and self._repeat(event):
                continue
            sym = self._keysym(event.key.keycode)
            if sym not in _WATCHED:
                continue
            sym = _ARROW_TO.get(sym, sym)
            if event.key.type == _KEY_RELEASE:
                self.keys.discard(sym)
                continue
            self.keys.add(sym)
            self.pressed.append(sym)
            if sym == KEY_ESC:
                self.quit = True
        return self.quit

    def close(self):
        if self.display:
            self.x11.XDestroyWindow(self.display, self.window)
            self.x11.XCloseDisplay(self.display)
            self.display = None


class TerminalKeys:
    """Keys typed in this terminal. Arrow keys match W/S/A/D; release follows key repeat."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.active = sys.stdin.isatty()
        self.held_until = {}
        self.pressed = []
        self.buf = b''
        self.old = None
        if not self.active:
            return
        self.old = termios.tcgetattr(self.fd)
        raw = termios.tcgetattr(self.fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO)
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, raw)

    def close(self):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.old = None

    def held(self):
        now = time.monotonic()
        return {key for key, until in self.held_until.items() if until >= now}

    def poll(self):
        if not self.active:
            return
        while select.select([self.fd], [], [], 0)[0]:
            try:
                chunk = os.read(self.fd, 64)
            except BlockingIOError:
                break
            if not chunk:
                break
            self.buf += chunk
        self._parse()
        now = time.monotonic()
        self.held_until = {key: until for key, until in self.held_until.items() if until >= now}

    def _touch(self, sym):
        fresh = sym not in self.held_until
        self.held_until[sym] = time.monotonic() + 0.28
        if fresh:
            self.pressed.append(sym)

    def _parse(self):
        data = self.buf
        i = 0
        while i < len(data):
            if data[i] == 0x1b:
                if i + 1 >= len(data):
                    if not select.select([self.fd], [], [], 0)[0]:
                        self._touch(KEY_ESC)
                        i += 1
                        continue
                    break
                if data[i + 1] != ord('['):
                    self._touch(KEY_ESC)
                    i += 1
                    continue
                j = i + 2
                while j < len(data) and data[j] < 0x40:
                    j += 1
                if j >= len(data):
                    break
                arrows = {ord('A'): KEY_W, ord('B'): KEY_S, ord('C'): KEY_D, ord('D'): KEY_A}
                if data[j] in arrows:
                    if b';2' in data[i:j]:
                        self._touch(KEY_LEFT_SHIFT)
                    self._touch(arrows[data[j]])
                i = j + 1
                continue
            letter = chr(data[i]).lower()
            mapped = {
                'w': KEY_W, 's': KEY_S, 'a': KEY_A, 'd': KEY_D,
                'q': KEY_Q, 'e': KEY_E, 'i': KEY_I, 'k': KEY_K,
                'j': KEY_J, 'l': KEY_L, 'u': KEY_U, 'o': KEY_O,
                'r': KEY_R, 'p': KEY_P, ' ': KEY_SPACE,
            }.get(letter)
            if mapped is not None:
                self._touch(mapped)
            i += 1
        self.buf = data[i:]


def fk(env, q):
    env.ik_data.qpos[env.qids] = q
    mujoco.mj_forward(env.model, env.ik_data)
    pos = env.ik_data.site_xpos[env.tcp_id].copy()
    rot = env.ik_data.site_xmat[env.tcp_id].reshape(3, 3).copy()
    return pos, rot


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


def apply_keys(env, keys, dt):
    if env.paused:
        return
    key = lambda k: float(k in keys)
    slow = KEY_LEFT_SHIFT in keys or KEY_RIGHT_SHIFT in keys
    speed = 0.025 if slow else 0.10
    translation = np.array([key(KEY_D) - key(KEY_A),
                            key(KEY_W) - key(KEY_S),
                            key(KEY_Q) - key(KEY_E)])
    rotation = np.array([key(KEY_I) - key(KEY_K),
                         key(KEY_J) - key(KEY_L),
                         key(KEY_U) - key(KEY_O)])
    if np.any(translation) or np.any(rotation):
        env.move_target(translation * speed * dt, rotation * (0.25 if slow else 0.75) * dt)


def open_view(env):
    """MuJoCo window. The pictured arm is overwritten with measured joints."""
    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(env.model, env.data)
    except Exception as exc:
        print('MuJoCo 窗口没有打开:', exc, flush=True)
        return None
    viewer.cam.lookat[:] = [0., 0.16, 0.76]
    viewer.cam.distance = 1.35
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -24
    print('MuJoCo 窗口显示真机实测关节角。鼠标可转视角，方向键仍在这个终端里按。', flush=True)
    return viewer


def show_hardware(env, viewer, q, dq=None):
    if viewer is None:
        return True
    if not viewer.is_running():
        return False
    with viewer.lock():
        env.data.qpos[env.qids] = q
        if dq is not None:
            env.data.qvel[env.dofs] = dq
        env.data.qpos[env.gripper_q] = 0.8 if env.gripper_open else -0.13
        mujoco.mj_forward(env.model, env.data)
    viewer.sync()
    return True


def hold_measured(env, bus, kp, kd, keys, term_keys, period, viewer):
    """Hold the measured joints. The calibrated pose is already the shared zero.

    A single mode=0 sample does not stop the arm. FOC has to stay up for
    about 0.2 s before keyboard control starts.
    """
    q = np.array(bus.q, dtype=float, copy=True)
    print('停在当前标定姿态，不前往仿真初始姿态。', flush=True)
    stable = 0
    sent = 0
    next_tick = time.monotonic()
    deadline = next_tick + 2.0
    while True:
        if keys.poll():
            return False
        now = time.monotonic()
        if now < next_tick:
            time.sleep(max(0., next_tick - now))
        next_tick += period
        term_keys.poll()
        if KEY_ESC in term_keys.held():
            return False
        bus.send(q, np.zeros(7), np.zeros(7), kp, kd)
        sent += 1
        if not show_hardware(env, viewer, bus.q, bus.dq):
            return False
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


def creep_to_home(env, bus, kp, kd, keys, term_keys, period, viewer):
    """Slew from the measured joints to the MuJoCo initial joint pose.

    The command starts at the measurement, so an out-of-range reading is not
    snapped onto the limit. Speed is 0.08 rad/s. A growing track error stops
    the motion instead of pushing harder.
    """
    q = np.array(bus.q, dtype=float, copy=True)
    target = np.array(env.home_q, dtype=float, copy=True)
    speed = 0.08
    remain = float(np.max(np.abs(target - q)) / speed)
    print(f'慢速前往仿真初始姿态，最长约 {remain:.0f} 秒。Esc 或 Ctrl+C 停止。', flush=True)
    print('目标 deg', np.degrees(target).round(1), flush=True)
    status_at = 0.
    sent = 0
    enabling = True
    next_tick = time.monotonic()
    while True:
        if keys.poll():
            return 'quit'
        now = time.monotonic()
        if now < next_tick:
            time.sleep(max(0., next_tick - now))
        next_tick += period
        if time.monotonic() - next_tick > 0.05:
            next_tick = time.monotonic()
        term_keys.poll()
        if KEY_ESC in term_keys.held():
            return 'quit'
        if enabling:
            if np.all(bus.mode == 1):
                enabling = False
                q = np.array(bus.q, dtype=float, copy=True)
                print('7 个关节都已进入 FOC，开始慢速移动', flush=True)
            elif sent > 200 or np.any(bus.err != 0):
                stopped = [i for i, mode in enumerate(bus.mode) if mode != 1]
                print(f'关节 {stopped} 仍是停止模式 mode={bus.mode} err={[hex(int(e)) for e in bus.err]}', flush=True)
                print('先清故障再重试: ./test_arm_lowlevel --clear --port /dev/ttyUSB0', flush=True)
                return 'fault'
            else:
                bus.send(q, np.zeros(7), np.zeros(7), kp, kd)
                sent += 1
                if not show_hardware(env, viewer, bus.q, bus.dq):
                    return 'quit'
                continue
        gap = target - q
        track = float(np.max(np.abs(q - bus.q)))
        if np.max(np.abs(gap)) < 0.02 and track < 0.08:
            print('已到仿真初始关节角', flush=True)
            return 'done'
        if track > 0.30:
            print(f'靠近时跟踪误差 {track:.3f} rad，停机', flush=True)
            return 'fault'
        step = np.clip(gap, -speed * period, speed * period)
        q = q + step
        inside = bool(np.all((q >= env.limits[:, 0]) & (q <= env.limits[:, 1])))
        tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q)
               if inside else np.zeros(7))
        bus.send(q, step / period, tau, kp, kd)
        sent += 1
        if not show_hardware(env, viewer, bus.q, bus.dq):
            return 'quit'
        if np.any(bus.err != 0) or np.any(bus.mode != 1):
            print('电机退出 FOC 或报错 mode=', bus.mode, 'err=', [hex(int(e)) for e in bus.err], flush=True)
            return 'fault'
        if now - status_at > 0.5:
            status_at = now
            left = float(np.degrees(np.max(np.abs(target - q))))
            print(f'剩余 {left:.1f}°  track={track:.3f} rad', flush=True)


def anchor(env, q):
    env.q_command = np.array(q, dtype=float, copy=True)
    env.target_position, env.target_rotation = fk(env, env.q_command)
    env.reference.reset(env.q_command)
    env.ik_error = 0.
    env.update_marker()


def main():
    parser = argparse.ArgumentParser(description=HELP, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--calib', type=Path, default=SDK_CALIB)
    parser.add_argument('--bridge', type=Path, default=SDK_BIN)
    parser.add_argument('--motor-config', type=Path)
    args = parser.parse_args()
    if not args.bridge.is_file():
        sys.exit(f'缺少 {args.bridge}，请先在 r1-7a_sdk/build 里 cmake && make demo_cmd_stream')

    print('加载仿真模型与 IK……', flush=True)
    env = ArmEnvironment(motor_config=args.motor_config)
    kp = env.motors.default_kp.copy()
    kd = env.motors.default_kd.copy()
    print('kp=', kp, 'kd=', kd, 'gravity_scale=', env.motors.gravity_feedforward_scale, flush=True)
    print(HELP, flush=True)
    print('这个终端里即可按键：方向键 上/下=Y，左/右=X。Shift+方向键为慢速。Esc 或 Ctrl+C 退出。', flush=True)
    keys = HeldKeys()
    term_keys = TerminalKeys()
    viewer = open_view(env)

    bus = Bus(args.bridge, args.port, args.calib)
    show_hardware(env, viewer, bus.q, bus.dq)
    print('真机 / 画面关节角 deg', np.degrees(bus.q).round(1), flush=True)
    period = env.motors.command_period
    try:
        if not hold_measured(env, bus, kp, kd, keys, term_keys, period, viewer):
            return
        anchor(env, bus.q)
        print('TCP m =', env.target_position.round(3), '停在当前姿态，可以按方向键', flush=True)
        next_tick = time.monotonic()
        status_at = 0.
        sent = 0
        while not keys.poll() and KEY_ESC not in term_keys.held():
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            next_tick += period
            if time.monotonic() - next_tick > 0.05:
                next_tick = time.monotonic()

            term_keys.poll()
            held = set(keys.keys) | term_keys.held()
            for key in keys.pressed + term_keys.pressed:
                if key == KEY_R:
                    anchor(env, bus.q)
                    env.paused = False
                elif key == KEY_P:
                    env.paused = not env.paused
                elif key == KEY_SPACE:
                    env.gripper_open = not env.gripper_open
                    env.data.qpos[env.gripper_q] = 0.8 if env.gripper_open else -0.13
                    print('夹爪标记', 'OPEN' if env.gripper_open else 'CLOSED',
                          '（未发送夹爪舵机指令）', flush=True)
            keys.pressed.clear()
            term_keys.pressed.clear()

            apply_keys(env, held, period)
            if not env.paused:
                q_cmd, dq_cmd = env.reference.update(env.q_command, period)
                tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q_cmd)
                       if env.motors.gravity_feedforward_scale else np.zeros(7))
            else:
                q_cmd = env.reference.q.copy()
                dq_cmd = np.zeros(7)
                tau = (env.motors.gravity_feedforward_scale * env.gravity_torque(q_cmd)
                       if env.motors.gravity_feedforward_scale else np.zeros(7))

            if np.max(np.abs(q_cmd - bus.q)) > TRACK_ABORT_RAD:
                print(f'跟踪误差超过 {TRACK_ABORT_RAD} rad，停机', flush=True)
                break
            bus.send(q_cmd, np.zeros(7) if env.paused else dq_cmd, tau, kp, kd)
            sent += 1
            if not show_hardware(env, viewer, bus.q, bus.dq):
                break
            if np.any(bus.err != 0) or (sent > 30 and np.any(bus.mode != 1)):
                print('电机退出 FOC 或报错 mode=', bus.mode, 'err=', [hex(int(e)) for e in bus.err], flush=True)
                break
            now = time.monotonic()
            if now - status_at > 0.2:
                status_at = now
                err = np.max(np.abs(q_cmd - bus.q))
                print(f'TCP {env.target_position.round(3)}  ik={env.ik_error:.4f}  '
                      f'track={err:.3f} rad  tau={np.array2string(tau, precision=1)}  '
                      f'{"PAUSE" if env.paused else "RUN"}', flush=True)
    finally:
        bus.stop()
        if viewer is not None:
            viewer.close()
        term_keys.close()
        keys.close()
        print('已停机', flush=True)


if __name__ == '__main__':
    main()
