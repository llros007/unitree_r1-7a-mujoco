"""Visible X11/GLFW end-to-end teleop check; sends keys only to its own window.

Run on the desktop with MUJOCO_GL=glfw. No serial hardware is opened.
Screenshots and a JSON report are written to outputs/teleop_gui_check/.
"""
import ctypes as C
import json
import time
from pathlib import Path

import glfw
import mujoco
import numpy as np
from PIL import Image

from environment import ArmEnvironment
from real_keyboard import Session, RealKeyboardViewer, parse_args

ROOT = Path(__file__).resolve().parent


class KeyEvent(C.Structure):
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                ('display', C.c_void_p), ('window', C.c_ulong), ('root', C.c_ulong),
                ('subwindow', C.c_ulong), ('time', C.c_ulong), ('x', C.c_int),
                ('y', C.c_int), ('x_root', C.c_int), ('y_root', C.c_int),
                ('state', C.c_uint), ('keycode', C.c_uint), ('same_screen', C.c_int)]


class XEvent(C.Union):
    _fields_ = [('key', KeyEvent), ('pad', C.c_long * 24)]


class WindowKeys:
    def __init__(self, window):
        self.lib = C.CDLL('libX11.so.6')
        self.lib.XOpenDisplay.restype = C.c_void_p
        self.lib.XOpenDisplay.argtypes = [C.c_char_p]
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('This desktop check requires X11')
        self.lib.XDefaultRootWindow.restype = C.c_ulong
        self.lib.XDefaultRootWindow.argtypes = [C.c_void_p]
        self.lib.XKeysymToKeycode.restype = C.c_uint
        self.lib.XKeysymToKeycode.argtypes = [C.c_void_p, C.c_ulong]
        self.lib.XSendEvent.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.POINTER(XEvent)]
        self.lib.XFlush.argtypes = [C.c_void_p]
        self.lib.XCloseDisplay.argtypes = [C.c_void_p]
        self.window = glfw.get_x11_window(window)
        self.root = self.lib.XDefaultRootWindow(self.display)
        self.held = set()

    def send(self, sym, down):
        event = XEvent()
        event.key.type = 2 if down else 3
        event.key.display = self.display
        event.key.window, event.key.root = self.window, self.root
        event.key.same_screen = 1
        event.key.keycode = self.lib.XKeysymToKeycode(self.display, sym)
        event.key.state = 1 if 0xffe1 in self.held else 0
        if not self.lib.XSendEvent(self.display, self.window, False,
                                  1 if down else 2, C.byref(event)):
            raise RuntimeError('XSendEvent failed')
        self.lib.XFlush(self.display)
        if down:
            self.held.add(sym)
        else:
            self.held.discard(sym)

    def close(self):
        for sym in list(self.held):
            self.send(sym, False)
        self.lib.XCloseDisplay(self.display)


def main():
    args = parse_args(['--mode', 'sim', '--control-mode', 'joint'])
    assert not args.hardware
    env = ArmEnvironment(initial_pose='zero', calibration_path=args.calib)
    display = ArmEnvironment(initial_pose='zero', calibration_path=args.calib)
    session = Session(env, args)
    viewer = RealKeyboardViewer(display, session)
    keys = WindowKeys(viewer.window)
    out = ROOT/'outputs/teleop_gui_check'
    out.mkdir(parents=True, exist_ok=True)
    report = {'hardware_connected': False, 'input': 'X11 key events to the test GLFW window',
              'checks': [], 'screenshots': []}
    original_swap = glfw.swap_buffers
    capture = [None]

    def swap(window):
        if capture[0]:
            w, h = glfw.get_framebuffer_size(window)
            rgb = np.zeros((h, w, 3), dtype=np.uint8)
            mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0, 0, w, h), viewer.context)
            path = out/capture[0]
            Image.fromarray(rgb[::-1]).save(path)
            report['screenshots'].append(str(path))
            capture[0] = None
        original_swap(window)

    glfw.swap_buffers = swap

    def state():
        with session.lock:
            s = session.snapshot
        if session.error:
            raise RuntimeError(session.error)
        if session.stop_event.is_set():
            raise RuntimeError('Control worker unexpectedly stopped')
        return s

    def pump(seconds=.1, screenshot=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if glfw.window_should_close(viewer.window):
                raise RuntimeError('Test window closed')
            glfw.poll_events()
            session.input_at = time.monotonic()
            viewer.display(state())
            viewer.render()
        if screenshot:
            capture[0] = screenshot
            viewer.display(state())
            viewer.render()
        return state()

    def tap(sym):
        keys.send(sym, True); pump(.06)
        keys.send(sym, False); pump(.06)

    def check(name, condition, **measurements):
        report['checks'].append({'name': name, 'passed': bool(condition), **measurements})
        print(('PASS ' if condition else 'FAIL ') + name, measurements, flush=True)
        if not condition:
            raise AssertionError(name)

    def angle(s):
        return np.degrees(s['qpos'][env.qids])

    try:
        session.thread.start()
        pump(.7, '01-zero.png')
        s = state()
        check('zero startup, worker running', not s['paused'] and np.max(np.abs(angle(s))) < .1,
              measured_deg=angle(s).round(4).tolist())
        tap(ord('4'))
        keys.send(0xff52, True)
        s = pump(1., '02-j4-moving.png')
        check('X11 hold Up moves selected J4', angle(s)[3] > 8 and s['selected_joint'] == 3,
              measured_deg=angle(s).round(3).tolist())
        keys.send(0xff52, False)
        pump(.7)
        before = state()['q_command'].copy()
        q_before = angle(state())
        s = pump(.7, '03-release-hold.png')
        check('release keeps target and settles', np.max(np.abs(s['q_command']-before)) < 1e-10
              and np.max(np.abs(angle(s)-q_before)) < .1,
              settled_change_deg=float(np.max(np.abs(angle(s)-q_before))))
        before = angle(s)
        tap(ord('r'))
        s = pump(.2)
        check('R begins slow zero return without teleport', abs(angle(s)[3]-before[3]) < 1.)
        s = pump(np.pi * abs(before[3]) / (2*5.) + 1., '07-slow-zero.png')
        check('R returns all joints to zero and holds', np.max(np.abs(angle(s))) < 1.,
              measured_deg=angle(s).round(4).tolist())
        tap(ord('p'))
        before = state()['q_command'].copy()
        keys.send(0xff52, True)
        s = pump(.5, '04-paused.png')
        check('P pauses, held movement is ignored', s['paused'] and np.array_equal(s['q_command'], before)
              and np.max(np.abs(env.motors.host_command[1])) == 0)
        keys.send(0xff52, False); tap(ord('p'))
        s = pump(.3)
        check('resume has no stale motion', not s['paused'] and np.max(np.abs(s['q_command']-before)) < 1e-10)
        before = state()['q_command'][3]
        keys.send(0xffe1, True); keys.send(0xff52, True)
        s = pump(.8)
        delta = float(np.degrees(s['q_command'][3]-before))
        keys.send(0xff52, False); keys.send(0xffe1, False); pump(.5)
        check('Shift gives quarter-speed jog', 2 < delta < 6, target_change_deg=delta)
        # J2 zero lower limit: negative jog must be blocked.
        tap(ord('2')); keys.send(0xff54, True)
        s = pump(.4); keys.send(0xff54, False)
        check('joint lower limit blocks negative J2', s['q_command'][1] >= 0)
        for j in range(7):
            tap(ord(str(j+1)))
            before = state()['q_command'][j]
            keys.send(0xff52, True)
            s = pump(.2)
            keys.send(0xff52, False)
            increase = float(np.degrees(s['q_command'][j]-before))
            pump(.25)
            measured_increase = float(angle(state())[j]-np.degrees(before))
            check(f'J{j+1} responds to window jog', increase > .5 and measured_increase > .3,
                  target_increase_deg=increase, measured_increase_deg=measured_increase)
            keys.send(0xff54, True); pump(.2)
            keys.send(0xff54, False); pump(.25)
        before = angle(state())
        tap(0xff09)
        s = state()
        check('Tab changes to IK without pose jump', s['control_mode'] == 'cartesian'
              and np.max(np.abs(angle(s)-before)) < .15)
        before = s['target_position'].copy()
        keys.send(ord('q'), True)
        s = pump(.5, '05-ik-moving.png')
        keys.send(ord('q'), False); pump(.7)
        check('IK Q raises TCP target', s['target_position'][2] > before[2]+.005,
              target_delta_m=(s['target_position']-before).round(4).tolist(),
              ik_residual=float(s['ik_error']))
        # Pause the UI thread for longer than the old erroneous watchdog threshold.
        time.sleep(.8)
        s = pump(.2)
        check('0.8-second render stall does not pause pure simulation', not s['paused'])
        was_open = s['gripper_open']
        tap(0x20)
        s = pump(3.5, '06-gripper.png')
        check('Space moves simulated gripper', s['gripper_open'] != was_open,
              jaw_angle_deg=float(np.degrees(s['qpos'][env.gripper_q])))
        # Compare the actual renderer's joint data to the last displayed snapshot.
        with session.lock:
            s = session.snapshot
        viewer.display(s)
        check('rendered joints equal copied simulation state',
              np.array_equal(viewer.env.data.qpos[env.qids], s['qpos'][env.qids]))
        check('finite dynamics, no tool/arm penetration', np.isfinite(s['qpos']).all()
              and np.isfinite(s['qvel']).all() and env.tool_collision(s['qpos'][env.qids]) is None)
        keys.send(0xff1b, True)
        end = time.monotonic() + .3
        while not session.stop_event.is_set() and time.monotonic() < end:
            glfw.poll_events()
            time.sleep(.01)
        check('X11 Escape stops control worker', session.stop_event.is_set())
        report['passed'] = True
    except Exception as exc:
        report['passed'] = False
        report['error'] = str(exc)
        try:
            pump(.1, 'failure.png')
        except Exception:
            pass
        raise
    finally:
        session.stop_event.set(); session.thread.join(3)
        keys.close()
        glfw.swap_buffers = original_swap
        viewer.close()
        (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print('Report:', out/'report.json', flush=True)


if __name__ == '__main__':
    main()
