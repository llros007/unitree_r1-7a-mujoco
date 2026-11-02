# SPDX-License-Identifier: AGPL-3.0-or-later
"""MuJoCo images transported through a genuine librealsense software device.

Both streams share the rendered optical frame. This is an ideal registered
RGB-D sensor, not a simulation of the D435 stereo matching electronics.
"""
import numpy as np
from .camera import Frame, RGBDCamera


class RealSenseSDKCamera:
    depth_scale = 0.001  # metres per Z16 unit

    def __init__(self, env, width=640, height=480):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError('RealSense SDK backend requires: python -m pip install -r '
                               'yolo_grasp/requirements-realsense.txt') from exc
        self.rs = rs
        self.source = RGBDCamera(env, width, height)
        self.frame_number = 0
        self.sensors = []
        self.closed = False
        try:
            self.device = rs.software_device()
            self.syncer = rs.syncer()
            self.profiles = []
            focal = .5 * height / np.tan(np.deg2rad(env.model.cam_fovy[self.source.camera_id]) / 2)
            intr = rs.intrinsics()
            intr.width, intr.height = width, height
            intr.fx = intr.fy = float(focal)
            intr.ppx, intr.ppy = (width - 1) / 2, (height - 1) / 2
            intr.model = rs.distortion.none
            intr.coeffs = [0.] * 5
            for uid, (name, stream_type, fmt, bpp) in enumerate([
                ('Depth', rs.stream.depth, rs.format.z16, 2),
                ('Color', rs.stream.color, rs.format.rgb8, 3),
            ]):
                sensor = self.device.add_sensor(name)
                self.sensors.append(sensor)
                stream = rs.video_stream()
                stream.type, stream.index, stream.uid = stream_type, 0, uid
                stream.width, stream.height, stream.fps = width, height, 30
                stream.bpp, stream.fmt, stream.intrinsics = bpp, fmt, intr
                profile = sensor.add_video_stream(stream).as_video_stream_profile()
                self.profiles.append(profile)
                if stream_type == rs.stream.depth:
                    sensor.add_read_only_option(rs.option.depth_units, self.depth_scale)
            self.device.create_matcher(rs.matchers.default)
            extr = rs.extrinsics()
            extr.rotation = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            extr.translation = [0., 0., 0.]
            self.profiles[0].register_extrinsics_to(self.profiles[1], extr)
            for sensor, profile in zip(self.sensors, self.profiles):
                sensor.open(profile)
                sensor.start(self.syncer)
            self.pointcloud = rs.pointcloud()
            self.last_sdk_frames = None
        except Exception:
            self.close()
            raise

    def capture(self):
        if self.closed:
            raise RuntimeError('Camera is closed')
        raw = self.source.capture()
        depth = np.ascontiguousarray(np.clip(np.rint(raw.depth / self.depth_scale), 0, 65535), dtype=np.uint16)
        rgb = np.ascontiguousarray(raw.rgb, dtype=np.uint8)
        self._buffers = (depth, rgb)
        # librealsense's timestamp matcher can emit incomplete startup pairs.
        # Prime it with the SAME captured snapshot; never use a stale image/pose.
        for attempt in range(5):
            self.frame_number += 1
            timestamp = self.frame_number * 1000. / 30.
            for sensor, profile, pixels, bpp in zip(self.sensors, self.profiles, self._buffers, (2, 3)):
                frame = self.rs.software_video_frame()
                frame.pixels, frame.stride, frame.bpp = pixels, pixels.strides[0], bpp
                frame.timestamp = timestamp
                frame.domain = self.rs.timestamp_domain.hardware_clock
                frame.frame_number, frame.profile = self.frame_number, profile
                if bpp == 2:
                    frame.depth_units = self.depth_scale
                sensor.on_video_frame(frame)
            frames = self.syncer.wait_for_frames(5000)
            d, c = frames.get_depth_frame(), frames.get_color_frame()
            if d and c and d.get_frame_number() == self.frame_number and c.get_frame_number() == self.frame_number:
                break
        else:
            raise RuntimeError('RealSense SDK did not return the current synchronized RGB-D pair')
        self.last_sdk_frames = frames
        intr = d.profile.as_video_stream_profile().get_intrinsics()
        K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
        return Frame(np.asanyarray(c.get_data()).copy(),
                     np.asanyarray(d.get_data()).astype(np.float32) * d.get_units(),
                     K, raw.world_from_camera, d.get_timestamp(), d.get_frame_number(),
                     self.sdk_points())

    def sdk_points(self):
        """Organized camera-coordinate XYZ from SDK pointcloud (metres)."""
        if self.last_sdk_frames is None:
            raise RuntimeError('Capture a frame before requesting points')
        points = self.pointcloud.calculate(self.last_sdk_frames.get_depth_frame())
        return np.asanyarray(points.get_vertices()).view(np.float32).reshape(
            self.source.height, self.source.width, 3).copy()

    def close(self):
        if self.closed:
            return
        self.closed = True
        for sensor in self.sensors:
            try:
                sensor.stop()
            except RuntimeError:
                pass
            try:
                sensor.close()
            except RuntimeError:
                pass
        self.source.close()
