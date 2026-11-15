"""Read-only side-by-side viewer; the test worker exclusively owns hardware and physics."""
import queue
import sys
import threading
import time
from types import SimpleNamespace

import glfw
import mujoco
import numpy as np

from environment import ArmEnvironment
from main import KeyboardViewer
from real_keyboard import terminal_exit


class ComparisonViewer(KeyboardViewer):
    def __init__(self, env, stop_event):
        self.stop_event = stop_event
        self.latest = None
        self.test_status = 'Preparing test'
        super().__init__(env, control_mode='joint')
        glfw.set_window_title(self.window, 'Joint test | LEFT: feedback FK | RIGHT: independent simulation')

    def on_key(self, window, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            self.stop_event.set()
            glfw.set_window_should_close(window, True)

    def render(self):
        width, height = glfw.get_framebuffer_size(self.window)
        if not width or not height:
            return
        state = self.latest
        for side in (0, 1):
            if state is not None:
                self.env.data.qpos[self.env.qids] = state['q'] if side == 0 else state['sim_q']
                self.env.data.qvel[self.env.dofs] = state['dq'] if side == 0 else state['sim_dq']
                if 'tcp_target' in state:
                    self.env.data.mocap_pos[self.env.mocap_id] = state['tcp_target']
                mujoco.mj_forward(self.env.model, self.env.data)
            viewport = mujoco.MjrRect(side * (width // 2), 0, width // 2, height)
            mujoco.mjv_updateScene(self.env.model, self.env.data, self.option, None, self.camera,
                                   mujoco.mjtCatBit.mjCAT_ALL, self.scene)
            mujoco.mjr_render(viewport, self.scene, self.context)
            title = ('FEEDBACK -> model FK (not measured TCP)' if side == 0 else
                     'INDEPENDENT SIMULATION (same commands)')
            text = self.test_status
            if state is not None:
                text += f"\n{state['source']} | segment {state['segment']+1} | {state['phase']}"
                if 'tcp_error_mm' in state:
                    text += f"\nTCP feedback FK vs sim: {state['tcp_error_mm']:.2f} mm"
                text += '\nJoint   Target    Feedback    Sim     Real-Sim (deg)'
                for j, (target, measured, sim) in enumerate(zip(
                        np.rad2deg(state['target']), np.rad2deg(state['q']), np.rad2deg(state['sim_q']))):
                    text += f'\nJ{j+1}   {target:+8.3f}  {measured:+8.3f}  {sim:+8.3f}  {measured-sim:+8.3f}'
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                               viewport, title, '', self.context)
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                               viewport, text + '\nEsc / close: stop and exit | mouse: orbit / zoom', '', self.context)
        glfw.swap_buffers(self.window)

    def close(self):
        try:
            self.context.free()
        finally:
            glfw.destroy_window(self.window)
            glfw.terminate()


def run_viewer(args):
    from auto_joint_test import run
    # Window creation must succeed before any hardware connection is started.
    display = ArmEnvironment(initial_pose='clearance', motor_config=args.motor_config, calibration_path=args.calib)
    stop_event = threading.Event()
    done = threading.Event()
    snapshots = queue.Queue(maxsize=1)
    result = [1]
    heartbeat = [time.monotonic()]
    viewer = ComparisonViewer(display, stop_event)

    def publish(state):
        try:
            snapshots.get_nowait()
        except queue.Empty:
            pass
        snapshots.put_nowait(state)

    def worker():
        try:
            result[0] = run(args, publish, stop_event, lambda: heartbeat[0])
        except Exception as exc:
            print(f'Test failed before running: {exc}', flush=True)
        finally:
            done.set()

    thread = threading.Thread(target=worker, name='r1-joint-test', daemon=True)
    try:
        thread.start()
        if sys.stdin.isatty():
            threading.Thread(target=terminal_exit, args=(SimpleNamespace(stop_event=stop_event), sys.stdin),
                             daemon=True).start()
        while not glfw.window_should_close(viewer.window) and not stop_event.is_set():
            glfw.poll_events()
            heartbeat[0] = time.monotonic()
            try:
                viewer.latest = snapshots.get_nowait()
            except queue.Empty:
                pass
            viewer.test_status = ('FINISHED - driver stopped; Esc to close' if done.is_set() else
                                  'AUTOMATIC TEST - no manual motion keys')
            viewer.render()
            if args.close_on_finish and done.is_set():
                glfw.set_window_should_close(viewer.window, True)
            time.sleep(.001)
    finally:
        stop_event.set()
        thread.join(timeout=7.)
        viewer.close()
    return result[0]
