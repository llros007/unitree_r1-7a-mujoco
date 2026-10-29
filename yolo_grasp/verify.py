#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""End-to-end YOLO/RGB-D/contact verification at three target positions."""
import os
import sys
os.environ.setdefault('MUJOCO_GL', 'glfw' if '--gui' in sys.argv else 'egl')
import argparse
import json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from yolo_grasp.main import Application
from yolo_grasp.planner import choose
from yolo_grasp.wrist_mount import MIN_DEPTH
from motion import DOWN_ROTATION


def object_half_height(object_kind):
    if object_kind == 'plant':
        from yolo_grasp.plant_asset import POT_HEIGHT
        return POT_HEIGHT / 2
    return .02


def step_budget(controller):
    if controller.phase.startswith('scan_'):
        return int((120. / controller.speed + 40.) / .01)
    return int((sum(action[2] for action in controller.actions) + 20) / .01)


def verify_wrist_camera(app):
    import mujoco
    hand_id = app.env.model.body('hand').id
    def hand_pose():
        pose = np.eye(4)
        pose[:3, :3] = app.env.data.xmat[hand_id].reshape(3, 3)
        pose[:3, 3] = app.env.data.xpos[hand_id]
        return pose
    first = app.camera.capture()
    relative = np.linalg.inv(hand_pose()) @ first.world_from_camera
    app.env.move_to([.10, .23, .86], 2.)
    second = app.camera.capture()
    assert np.linalg.norm(first.world_from_camera[:3, 3] - second.world_from_camera[:3, 3]) > .02
    assert np.allclose(np.linalg.inv(hand_pose()) @ second.world_from_camera, relative, atol=1e-6), 'Camera must be rigidly attached to hand'
    assert np.linalg.det(second.world_from_camera[:3, :3]) > .999
    assert np.all((second.depth == 0) | (second.depth >= MIN_DEPTH))
    # At grasp height the target is inside the D435 near blind range.
    app.env.reset()  # vertical grasp orientation, independent of observation tilt
    app.env.move_to([.12, .25, .626], 2., rotation=DOWN_ROTATION)
    close = app.camera.capture()
    expected = app.env.cube_position + [0, 0, object_half_height(app.args.object)]
    optical = np.linalg.inv(close.world_from_camera) @ np.r_[expected, 1.]
    uv = close.K @ optical[:3]
    u, v = np.round(uv[:2] / uv[2]).astype(int)
    assert 0 < optical[2] < MIN_DEPTH
    if 0 <= v < close.depth.shape[0] and 0 <= u < close.depth.shape[1]:
        assert close.depth[v, u] == 0, 'Near-range target depth must be invalid'
    app.env.reset()
    app.park()


def verify(weights, output, object_kind='plant', speed=.5, confidence=.20):
    labels = ['potted plant', 'plant', 'flower pot'] if object_kind == 'plant' else ['red block', 'red cube']
    args = argparse.Namespace(weights=weights, classes=labels, object=object_kind, speed=speed,
                              device='cpu', conf=confidence, insertion=.02 if object_kind == 'plant' else .014, output=output)
    app = Application(args)
    results = []
    try:
        verify_wrist_camera(app)
        rng = np.random.default_rng(42)
        positions = [(0.12, .25), (.10, .25), (.18, .31)]
        positions += [tuple(xy) for xy in rng.uniform([.10, .25], [.18, .31], (3, 2))]
        for x, y in positions:
            app.controller.cancel()
            app.env.reset()
            # Ground truth is modified only to arrange independent test scenes.
            # Perception/controller still receive only RGB-D-derived grasps.
            app.env.data.qpos[app.env.cube_q:app.env.cube_q + 3] = [x, y, .601 + object_half_height(object_kind)]
            if object_kind == 'plant':
                yaw = rng.uniform(-np.pi, np.pi)
                app.env.data.qpos[app.env.cube_q + 3:app.env.cube_q + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
            import mujoco
            mujoco.mj_forward(app.env.model, app.env.data)
            app.park()
            app.output = output / f'{x:.2f}_{y:.2f}'
            app.output.mkdir(parents=True, exist_ok=True)
            expected = app.env.cube_position + [0, 0, object_half_height(object_kind)]
            app.begin()
            for _ in range(step_budget(app.controller)):
                app.controller.tick()
                previous_command = app.env.gripper_command
                app.env.step()
                assert abs(app.env.gripper_command - previous_command) <= app.env.gripper_speed * .01 + 1e-9, 'Gripper must open/close gradually'
                app.save_progress()
                if not app.controller.active:
                    break
            assert app.controller.phase == 'done', app.controller.report
            grasp = app.grasp
            error = float(np.linalg.norm(grasp.surface - expected))
            assert error < .005, f'RGB-D localization error: {error} m'
            assert np.allclose(grasp.rotation.T @ grasp.rotation, np.eye(3), atol=1e-6)
            events = app.controller.report['search']
            detected = next(e for e in events if e['event'] == 'detected')
            localized = next(e for e in events if e['event'] == 'localized')
            assert localized['timestamp_s'] - detected['timestamp_s'] >= .49, 'Must reacquire after stable hold'
            results.append(dict(scene_xy=[x, y], localization_error_m=error, **app.controller.report))
        frame = app.last_frame
        assert app.last_detections, 'Need real detections to test depth rejection'
        for detections in [[], app.last_detections]:
            if detections:
                frame.depth[:] = np.nan
            try:
                choose(frame, detections)
            except ValueError:
                pass
            else:
                raise AssertionError('Missing detection/invalid depth must not produce a grasp')
        # Exercise an empty grasp using the real contact dynamics. It must stop
        # before lift/transfer rather than report a successful grasp.
        app.env.reset()
        app.park()
        app.output = output / 'no_contact'
        app.output.mkdir(parents=True, exist_ok=True)
        empty_grasp = app.perceive()
        empty_grasp.tcp[0] += .08
        app.controller.start(empty_grasp)
        for _ in range(step_budget(app.controller)):
            app.controller.tick()
            app.env.step()
            if not app.controller.active:
                break
        assert app.controller.phase == 'failed' and 'contact' in app.controller.error.lower(), app.controller.report
        summary = dict(passed=True, object=object_kind, speed=speed, confidence_threshold=confidence, smooth_gripper='passed', scenes=results, wrist_extrinsics='passed', d435_near_range='passed', no_detection='passed',
                       invalid_depth='passed', no_contact='passed')
        (output / 'verification.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)
    finally:
        app.camera.close()


def verify_viewer(weights, output, object_kind='plant', speed=.5, confidence=.20):
    """Optional real-window check, including sensor/snapshot GL context switches."""
    from yolo_grasp.main import Viewer
    import glfw
    labels = ['potted plant', 'plant', 'flower pot'] if object_kind == 'plant' else ['red block', 'red cube']
    args = argparse.Namespace(weights=weights, classes=labels, object=object_kind, speed=speed,
                              device='cpu', conf=confidence, insertion=.02 if object_kind == 'plant' else .014, output=output)
    app = Application(args)
    viewer = None
    try:
        viewer = Viewer(app)
        assert glfw.get_window_attrib(viewer.window, glfw.VISIBLE), 'Viewer must be visible after the offscreen camera is created'
        def press(key):
            viewer.on_key(viewer.window, key, 0, glfw.PRESS, 0)
        press(glfw.KEY_V)
        assert app.grasp is not None and not app.controller.active
        viewer.render()
        press(glfw.KEY_G)
        assert app.controller.active
        press(glfw.KEY_SPACE)
        assert app.env.gripper_open, 'Manual gripper command must be blocked while active'
        press(glfw.KEY_P)
        elapsed, sim_time = app.controller.elapsed, app.env.data.time
        viewer.controls(.01)
        app.env.step()
        assert app.env.paused and app.controller.elapsed == elapsed and app.env.data.time == sim_time
        press(glfw.KEY_P)
        phase = app.controller.phase
        for _ in range(step_budget(app.controller)):
            viewer.controls(.01)
            app.env.step()
            if phase != app.controller.phase:
                viewer.render()
                phase = app.controller.phase
            if not app.controller.active:
                break
        assert app.controller.phase == 'done', app.controller.report
        press(glfw.KEY_R)
        assert app.controller.phase == 'idle' and not app.env.paused
        press(glfw.KEY_V)
        assert not (app.output / 'result.json').exists(), 'Old success report must not survive new detection'
        viewer.run(.2)
        print('GUI: visible window, detection inset, G/V/R/P, contact grasp, snapshot context and clean close passed')
    finally:
        if viewer is not None:
            viewer.close()
        app.camera.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', default=str(Path(__file__).resolve().parent / 'models' / 'yoloe-26s-seg.pt'))
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'outputs' / 'verification')
    parser.add_argument('--gui', action='store_true', help='Also test a real GLFW viewer; requires a desktop')
    parser.add_argument('--object', choices=['plant', 'cube'], default='plant')
    parser.add_argument('--speed', type=float, default=.5)
    parser.add_argument('--conf', type=float, default=.20, help='YOLO confidence, matching the application option')
    args = parser.parse_args()
    verify(args.weights, args.output.resolve(), args.object, args.speed, args.conf)
    if args.gui:
        verify_viewer(args.weights, args.output.resolve() / 'gui', args.object, args.speed, args.conf)
