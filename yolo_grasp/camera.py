# SPDX-License-Identifier: AGPL-3.0-or-later
"""Eye-in-hand D435 RGB-D approximation, rigidly attached to the SO101 wrist."""
from dataclasses import dataclass

import mujoco
import numpy as np
from .wrist_mount import CAMERA_NAME, MIN_DEPTH, MAX_DEPTH


@dataclass
class Frame:
    rgb: np.ndarray
    depth: np.ndarray
    K: np.ndarray
    world_from_camera: np.ndarray

    def backproject(self, uv, depth):
        uv = np.asarray(uv, dtype=float)
        rays = np.column_stack(((uv[:, 0] - self.K[0, 2]) / self.K[0, 0],
                                (uv[:, 1] - self.K[1, 2]) / self.K[1, 1],
                                np.ones(len(uv))))
        points = rays * np.asarray(depth)[:, None]
        return points @ self.world_from_camera[:3, :3].T + self.world_from_camera[:3, 3]


class RGBDCamera:
    def __init__(self, env, width=640, height=480):
        if (width, height) != (640, 480):
            raise ValueError('The simulated D435 profile is calibrated for 640x480')
        self.env = env
        self.width, self.height = width, height
        self.camera_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
        if self.camera_id < 0:
            raise ValueError('D435 wrist camera is missing; build the wrist scene first')
        self.renderer = mujoco.Renderer(env.model, height=height, width=width)
        self.option = mujoco.MjvOption()
        self.option.sitegroup[:] = 0

    def optical_pose(self):
        """Current camera-to-world pose in OpenCV right/down/forward convention."""
        transform = np.eye(4)
        gl_rotation = self.env.data.cam_xmat[self.camera_id].reshape(3, 3)
        transform[:3, :3] = gl_rotation @ np.diag([1., -1., -1.])
        transform[:3, 3] = self.env.data.cam_xpos[self.camera_id]
        return transform

    def capture(self):
        # Synchronize camera pose with the current joints, not the last render.
        mujoco.mj_forward(self.env.model, self.env.data)
        transform = self.optical_pose()
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.env.data, camera=self.camera_id, scene_option=self.option)
        rgb = self.renderer.render().copy()
        focal = .5 * self.height / np.tan(np.deg2rad(self.env.model.cam_fovy[self.camera_id]) / 2)
        K = np.array([[focal, 0, (self.width - 1) / 2],
                      [0, focal, (self.height - 1) / 2], [0, 0, 1.]])
        self.renderer.enable_depth_rendering()
        try:
            depth = self.renderer.render().copy()
        finally:
            self.renderer.disable_depth_rendering()
        # Ideal depth aligned to RGB; simulate the profile's unavailable ranges.
        depth[(~np.isfinite(depth)) | (depth < MIN_DEPTH) | (depth > MAX_DEPTH)] = 0
        return Frame(rgb, depth, K, transform)

    def close(self):
        self.renderer.close()
