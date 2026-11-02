#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Integration checks using real librealsense, without a physical USB camera."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from environment import ArmEnvironment
from yolo_grasp.realsense_camera import RealSenseSDKCamera
from yolo_grasp.camera import Frame


def main():
    env = ArmEnvironment(object_kind='plant')
    camera = RealSenseSDKCamera(env)
    try:
        capture = camera.source.capture
        snapshots = []
        def record_capture():
            frame = capture()
            snapshots.append(frame)
            return frame
        camera.source.capture = record_capture
        first = camera.capture()
        raw = snapshots[-1]
        assert np.array_equal(first.rgb, raw.rgb)
        assert np.max(np.abs(first.depth - raw.depth)) <= .000501
        assert np.array_equal(first.depth == 0, raw.depth == 0)
        assert np.allclose(first.K, raw.K)
        assert np.allclose(first.world_from_camera, raw.world_from_camera)
        points = camera.sdk_points()
        v, u = np.nonzero(first.depth > 0)
        assert len(u), 'Expected visible depth'
        reference = Frame(first.rgb, first.depth, first.K, first.world_from_camera)
        expected = reference.backproject(np.column_stack((u, v)), first.depth[v, u])
        assert first.camera_points is not None
        assert np.allclose(first.backproject(np.column_stack((u, v)), first.depth[v, u]), expected, atol=1e-6)
        actual = points[v, u] @ first.world_from_camera[:3, :3].T + first.world_from_camera[:3, 3]
        assert np.allclose(actual, expected, atol=1e-6)
        for _ in range(4):
            env.step()
            frame = camera.capture()
            assert frame.frame_number > first.frame_number
            assert frame.timestamp_ms > first.timestamp_ms
            first = frame
        env.reset()
        reset = camera.capture()
        assert reset.timestamp_ms > first.timestamp_ms
        assert reset.frame_number == first.frame_number + 1
        print('PASS: SDK RGB/Z16 transport, intrinsics, SDK pointcloud, synchronization and reset clock')
    finally:
        camera.close()
        camera.close()


if __name__ == '__main__':
    main()
