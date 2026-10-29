#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""YOLO + simulated RGB-D + top-down contact grasping, without GraspNet."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

# Configure rendering before importing MuJoCo/GLFW.
if '--headless' in sys.argv:
    os.environ.setdefault('MUJOCO_GL', 'egl')
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))

import cv2
import glfw
import mujoco
import numpy as np
from PIL import Image

from environment import ArmEnvironment
from recording import JointRecorder
# Avoid importing this file again as the parent viewer module named "main".
import importlib.util
_spec = importlib.util.spec_from_file_location('r1_keyboard_viewer', ROOT.parent / 'main.py')
_viewer_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_viewer_module)
KeyboardViewer = _viewer_module.KeyboardViewer
snapshot = _viewer_module.snapshot

# Support both direct execution and python -m yolo_grasp.main.
from yolo_grasp.camera import RGBDCamera
from yolo_grasp.search import SearchPickPlace
from yolo_grasp.detector import Detector
from yolo_grasp.planner import annotate, choose


class Application:
    def __init__(self, args):
        if not 0 <= args.insertion <= .02:
            raise ValueError('--insertion must be in [0, 0.02] m')
        self.args = args
        object_kind = getattr(args, 'object', 'plant')
        speed = getattr(args, 'speed', .5)
        if not np.isfinite(speed) or not .1 <= speed <= 1.:
            raise ValueError('--speed must be in [0.1, 1.0]')
        motor_config = getattr(args, 'motor_config', None)
        self.env = ArmEnvironment(motor_config=motor_config, object_kind=object_kind,
                                  end_effector_config=getattr(args, 'end_effector_config', None))
        self.controller = SearchPickPlace(self.env, self.probe, lambda: self.perceive(_scan=True), speed=speed)
        self.rng = np.random.default_rng(getattr(args, 'seed', None))
        if getattr(args, 'randomize', False):
            self.randomize_object()
        self.output = args.output.expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.detector = Detector(args.weights, args.classes, args.device, args.conf)
        self.camera = RGBDCamera(self.env)
        self.image = None
        self.grasp = None
        self.message = 'G: detect and grasp | V: detect only | R: reset | P: pause'
        self.lift_saved = False
        self.place_saved = False
        self.report_saved = False
        self.live_frame = None
        try:
            self.park()
        except Exception:
            self.camera.close()
            raise

    def park(self):
        # Fixed observation pose over the workspace. The wrist camera moves with
        # the hand; retain sufficient distance for the D435 depth profile.
        self.env.move_to(self.env.home_position, 3., rotation=self.env.home_rotation)
        for _ in range(60):
            self.env.step()
        self.grasp = None
        self.lift_saved = self.place_saved = False
        self.report_saved = False

    def randomize_object(self):
        """Arrange a simulation trial; perception never reads this sampled pose."""
        xy = self.rng.uniform([.10, .25], [.18, .31])
        self.env.data.qpos[self.env.cube_q:self.env.cube_q + 3] = [*xy, self.env.object_rest_height + .001]
        yaw = self.rng.uniform(-np.pi, np.pi)
        self.env.data.qpos[self.env.cube_q + 3:self.env.cube_q + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(self.env.model, self.env.data)

    def probe(self):
        frame = self.camera.capture()
        detections = self.detector.detect(frame.rgb)
        self.live_frame = frame
        self.image = annotate(frame, detections)
        return bool(detections)

    def perceive(self, _scan=False):
        if self.controller.active and not _scan:
            raise RuntimeError('Wait for the current grasp or reset before detecting again')
        if not _scan:
            self.controller.cancel()
        self.grasp = None
        self.image = None
        self.lift_saved = self.place_saved = self.report_saved = False
        # Remove only this pipeline's old artifacts so failed runs cannot look successful.
        for name in ('grasp.json', 'result.json', 'lift.png', 'placed.png'):
            (self.output / name).unlink(missing_ok=True)
        frame = self.camera.capture()
        detections = self.detector.detect(frame.rgb)
        self.last_frame, self.last_detections = frame, detections
        try:
            self.grasp = choose(frame, detections, self.args.insertion,
                                object_kind=getattr(self.args, 'object', 'plant'))
        finally:
            self.image = annotate(frame, detections, self.grasp)
            Image.fromarray(self.image).save(self.output / 'detection.png')
            Image.fromarray(frame.rgb).save(self.output / 'rgb.png')
            np.savez_compressed(self.output / 'rgbd.npz', depth_m=frame.depth,
                                K=frame.K, world_from_camera=frame.world_from_camera,
                                timestamp_s=self.env.data.time)
            self.live_frame = frame
        (self.output / 'grasp.json').write_text(json.dumps(self.grasp.as_dict(), indent=2) + '\n')
        print(json.dumps(self.grasp.as_dict(), indent=2), flush=True)
        self.message = f'{self.grasp.label} {self.grasp.confidence:.2f} | G: grasp | V: detect'
        return self.grasp

    def begin(self):
        if self.controller.active:
            return
        self.grasp = None
        self.lift_saved = self.place_saved = self.report_saved = False
        for name in ('grasp.json', 'result.json', 'lift.png', 'placed.png', 'detection.png', 'rgb.png', 'rgbd.npz'):
            (self.output / name).unlink(missing_ok=True)
        self.controller.begin_search()
        self.message = 'Searching wrist observation poses; stop, reacquire, then grasp'

    def save_progress(self):
        if self.controller.phase == 'transfer' and not self.lift_saved:
            snapshot(self.env, self.output / 'lift.png')
            self.lift_saved = True
        if self.controller.phase in ('done', 'failed') and not self.report_saved:
            if self.controller.phase == 'done' and not self.place_saved:
                snapshot(self.env, self.output / 'placed.png')
                self.place_saved = True
            report = dict(self.controller.report, phase=self.controller.phase,
                          grasp=self.grasp.as_dict() if self.grasp else None)
            (self.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
            self.report_saved = True


class Viewer(KeyboardViewer):
    def __init__(self, app):
        self.app = app
        super().__init__(app.env)
        self.closed = False
        self.last_sensor_render = -float('inf')

    def on_key(self, window, key, scancode, action, mods):
        if action == glfw.PRESS and key == glfw.KEY_H:
            print('G: detect + grasp | V: detect only | R: reset | P: pause | Esc: exit\n'
                  'Idle: original keyboard controls; mouse: orbit / pan / zoom', flush=True)
            return
        if action == glfw.PRESS and key in (glfw.KEY_G, glfw.KEY_V):
            if self.app.controller.active:
                return
            try:
                self.app.lift_saved = self.app.place_saved = False
                self.app.begin() if key == glfw.KEY_G else self.app.perceive()
            except (ValueError, RuntimeError) as exc:
                self.app.message = str(exc)
                self.app.controller.fail(str(exc))
                print(self.app.message, flush=True)
            finally:
                glfw.make_context_current(self.window)
            return
        if action == glfw.PRESS and key == glfw.KEY_R:
            self.app.controller.cancel()
            self.env.reset()
            if getattr(self.app.args, 'randomize', False):
                self.app.randomize_object()
            self.keys.clear()
            self.app.park()
            self.app.message = 'Reset | G: detect and grasp | V: detect only'
            return
        # Manual movement/gripper commands are blocked during an automatic grasp.
        if self.app.controller.active and key not in (glfw.KEY_ESCAPE, glfw.KEY_P, glfw.KEY_H):
            return
        super().on_key(window, key, scancode, action, mods)

    def controls(self, dt):
        if self.app.controller.active:
            self.app.controller.tick(dt)
        else:
            super().controls(dt)

    def render(self):
        self.app.save_progress()
        if time.monotonic() - self.last_sensor_render >= .2:
            self.app.live_frame = self.app.camera.capture()
            self.last_sensor_render = time.monotonic()
        glfw.make_context_current(self.window)
        width, height = glfw.get_framebuffer_size(self.window)
        if not width or not height:
            return
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(self.env.model, self.env.data, self.option, None,
                               self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        phase = self.app.controller.phase
        text = f'D435 wrist RGB-D + YOLO | {phase}\n{self.app.message}\nG: detect + grasp | V: detect | R: reset | P: pause | ESC: exit\nMouse: orbit / pan / zoom'
        if self.app.controller.error:
            text += '\n' + self.app.controller.error
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           viewport, text, '', self.context)
        if self.app.live_frame is not None and width >= 680 and height >= 320:
            frame = self.app.live_frame
            depth = np.clip(frame.depth / 1.0 * 255, 0, 255).astype(np.uint8)
            depth = cv2.applyColorMap(depth, cv2.COLORMAP_TURBO)[:, :, ::-1].copy()
            depth[frame.depth == 0] = 0
            for image, left, label in [(frame.rgb, width - 650, 'D435 live RGB'),
                                       (depth, width - 325, 'D435 aligned depth (black: invalid)')]:
                image = cv2.resize(image, (320, 240))
                cv2.putText(image, label, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
                inset = mujoco.MjrRect(left, 10, 320, 240)
                mujoco.mjr_drawPixels(np.ascontiguousarray(image[::-1]).ravel(), None, inset, self.context)
        if self.app.image is not None and width >= 680 and height >= 580:
            image = cv2.resize(self.app.image, (320, 240))
            cv2.putText(image, 'Last YOLO capture', (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            inset = mujoco.MjrRect(width - 325, 260, 320, 240)
            mujoco.mjr_drawPixels(np.ascontiguousarray(image[::-1]).ravel(), None, inset, self.context)
        glfw.swap_buffers(self.window)

    def close(self):
        if self.closed:
            return
        # Close the sensor's hidden GL window before GLFW terminates all windows.
        self.app.camera.close()
        glfw.make_context_current(self.window)
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()
        self.closed = True

    def run(self, seconds=None):
        print('YOLO RGB-D grasp | G: detect + grasp | V: detect only | R: reset | P: pause | Esc: exit', flush=True)
        start = previous = time.monotonic()
        accumulator = 0.
        try:
            while not glfw.window_should_close(self.window):
                now = time.monotonic()
                if seconds is not None and now - start >= seconds:
                    break
                accumulator += min(now - previous, .1)
                previous = now
                glfw.poll_events()
                while accumulator >= .01:
                    self.controls(.01)
                    self.env.step(.01)
                    accumulator -= .01
                self.render()
                time.sleep(.001)
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', default=str(ROOT / 'models' / 'yoloe-26s-seg.pt'))
    parser.add_argument('--object', choices=['plant', 'cube'], default='plant', help='Scene target (default: community potted plant)')
    parser.add_argument('--classes', nargs='+', help='YOLO prompt labels; default follows --object')
    parser.add_argument('--speed', type=float, default=.5, help='Motion speed ratio 0.1..1.0 (default: 0.5)')
    parser.add_argument('--conf', type=float, default=.20)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--insertion', type=float, help='TCP distance below visible top surface, meters (plant: 0.020, cube: 0.014)')
    parser.add_argument('--motor-config', type=Path, help='Optional PD motor configuration for environments with torque control')
    parser.add_argument('--end-effector-config', type=Path, help='Shared end-effector mass/COM configuration')
    parser.add_argument('--record', type=Path, help='Shared command/state CSV recorder (new file)')
    parser.add_argument('--headless', action='store_true', help='Run one complete grasp with EGL rendering')
    parser.add_argument('--auto', action='store_true', help='Start a grasp immediately in the viewer')
    parser.add_argument('--randomize', action='store_true', help='Random plant XY in x=0.10..0.18, y=0.25..0.31 m and random yaw')
    parser.add_argument('--seed', type=int, help='Reproducible random placement seed')
    parser.add_argument('--detect-only', action='store_true', help='Save perception output without executing a grasp')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs' / 'latest')
    parser.add_argument('--seconds', type=float, help='Viewer time limit for smoke testing')
    args = parser.parse_args()
    if args.insertion is None:
        args.insertion = .020 if args.object == 'plant' else .014
    if args.classes is None:
        args.classes = ['potted plant', 'plant', 'flower pot'] if args.object == 'plant' else ['red block', 'red cube']
    app = None
    try:
        app = Application(args)
        with JointRecorder(app.env, args.record):
            if args.detect_only:
                app.perceive()
            elif args.headless or args.auto:
                app.begin()
            if args.headless:
                if not args.detect_only:
                    max_steps = int((120. / args.speed + 40.) / .01)
                    for _ in range(max_steps):
                        app.controller.tick()
                        app.env.step()
                        app.save_progress()
                        if not app.controller.active:
                            break
                    if app.controller.active:
                        app.controller.fail('Simulation time limit exceeded')
                        app.save_progress()
                    print(json.dumps(app.controller.report, indent=2))
                    if app.controller.phase != 'done':
                        return 1
            elif not args.detect_only:
                viewer = Viewer(app)
                viewer.run(args.seconds)
        return 0
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        print(f'YOLO grasp stopped: {exc}', file=sys.stderr)
        if app is not None:
            (app.output / 'result.json').write_text(json.dumps({'passed': False, 'error': str(exc)}, indent=2) + '\n')
        return 1
    finally:
        if app is not None:
            app.camera.close()


if __name__ == '__main__':
    raise SystemExit(main())
