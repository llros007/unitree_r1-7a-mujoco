# SPDX-License-Identifier: AGPL-3.0-or-later
"""Top-down 2.5D grasps from YOLO masks and calibrated depth, without GraspNet."""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Grasp:
    label: str
    confidence: float
    surface: np.ndarray
    tcp: np.ndarray
    rotation: np.ndarray
    width: float
    rectangle: np.ndarray
    center_px: np.ndarray

    def as_dict(self):
        return dict(label=self.label, confidence=self.confidence,
                    surface_world_m=self.surface.tolist(), tcp_world_m=self.tcp.tolist(),
                    rotation_world=self.rotation.tolist(), jaw_width_m=self.width,
                    center_px=self.center_px.tolist())


def estimate(frame, detection, insertion=.014, object_kind='cube'):
    if not 0 <= insertion <= .02:
        raise ValueError('Insertion depth must be in [0, 0.02] m')
    valid = (detection.mask > 0) & np.isfinite(frame.depth) & (frame.depth > .05) & (frame.depth < 2.)
    # Erode mask to avoid object/table mixtures at the silhouette.
    inner = cv2.erode(valid.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    ys, xs = np.nonzero(inner)
    if len(xs) < 12:
        raise ValueError('Too few valid depth pixels')
    points = frame.backproject(np.column_stack((xs, ys)), frame.depth[ys, xs])
    # Table is a calibrated plane at z=.60. Reject table/robot/background samples.
    keep = (points[:, 2] > .608) & (points[:, 2] < .70)
    if object_kind == 'plant':
        # The downloaded 04 pot is pale ceramic. Reject green foliage inside
        # the YOLO instance and restrict samples to the calibrated pot height.
        from .plant_asset import POT_HEIGHT
        colors = frame.rgb[ys, xs].astype(float)
        keep &= ((colors.max(axis=1) - colors.min(axis=1) < 60) &
                 (colors.mean(axis=1) > 70) & (points[:, 2] < .60 + POT_HEIGHT + .002))
    points, xs, ys = points[keep], xs[keep], ys[keep]
    if len(points) < 12:
        raise ValueError('No object surface above the table in the grasp workspace')
    # Fit the XY footprint; using world coordinates also handles camera-axis signs.
    rect = cv2.minAreaRect(points[:, :2].astype(np.float32))
    corners = cv2.boxPoints(rect)
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    short, long = float(lengths.min()), float(lengths.max())
    if object_kind != 'plant' and (not .015 <= short <= .065 or long > .12):
        raise ValueError(f'Object footprint outside gripper range: {short:.3f} x {long:.3f} m')
    surface = np.r_[rect[0], np.quantile(points[:, 2], .90) if object_kind == 'plant' else np.median(points[:, 2])]
    if object_kind == 'plant':
        # The oblique camera sees primarily the front half of the pot. A box
        # around that visible patch biases the center toward the camera. Fit
        # the round rim in world XY instead; its radius is estimated from RGB-D.
        rim = points[points[:, 2] >= surface[2] - .002, :2]
        if len(rim) < 24:
            raise ValueError('Too few pot rim samples')
        origin = rim.mean(axis=0)
        local = rim - origin
        system = np.column_stack((2 * local, np.ones(len(local))))
        fit, _, rank, _ = np.linalg.lstsq(system, np.sum(local**2, axis=1), rcond=None)
        center = fit[:2] + origin
        radius_squared = fit[2] + fit[:2] @ fit[:2]
        if rank < 3 or radius_squared <= 0:
            raise ValueError('Pot rim circle is degenerate')
        radius = np.sqrt(radius_squared)
        residual = np.sqrt(np.mean((np.linalg.norm(rim - center, axis=1) - radius)**2))
        from .plant_asset import POT_RADIUS
        if abs(radius - POT_RADIUS) > .006 or residual > .003:
            raise ValueError('Pot rim fit outside gripper range or too uncertain')
        surface[:2] = center
        short = long = float(2 * radius)
    if not (-.22 <= surface[0] <= .24 and .12 <= surface[1] <= .38):
        raise ValueError('Detected target outside configured pick workspace')
    # Symmetric square objects have no preferred yaw: choose the stable home yaw.
    axis = np.array([1., 0., 0.])
    if object_kind != 'plant' and long / max(short, 1e-9) > 1.15:
        axis[:2] = edges[int(np.argmin(lengths))] / short
        if axis[0] < 0:
            axis = -axis
    approach = np.array([0., 0., -1.])
    rotation = np.column_stack((axis, np.cross(approach, axis), approach))
    tcp = surface.copy()
    tcp[2] = max(.622, surface[2] - insertion)
    pixel_rect = cv2.boxPoints(cv2.minAreaRect(np.column_stack((xs, ys)).astype(np.float32)))
    optical = np.linalg.inv(frame.world_from_camera) @ np.r_[surface, 1.]
    projected_center = frame.K @ optical[:3]
    center_px = projected_center[:2] / projected_center[2]
    if object_kind == 'plant':
        angles = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        circle = surface + np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.zeros(len(angles))))
        camera_points = (circle - frame.world_from_camera[:3, 3]) @ frame.world_from_camera[:3, :3]
        pixels = camera_points @ frame.K.T
        pixel_rect = pixels[:, :2] / pixels[:, 2:3]
    return Grasp(detection.label, detection.confidence, surface, tcp, rotation,
                 short, pixel_rect, center_px)


def choose(frame, detections, insertion=.014, object_kind='cube'):
    candidates, rejected = [], []
    for detection in detections:
        try:
            candidates.append(estimate(frame, detection, insertion, object_kind))
        except ValueError as exc:
            rejected.append(f'{detection.label}: {exc}')
    if not candidates:
        raise ValueError('No valid YOLO grasp. ' + '; '.join(rejected))
    return max(candidates, key=lambda g: g.confidence)


def annotate(frame, detections, grasp=None):
    image = frame.rgb.copy()
    for d in detections:
        x1, y1, x2, y2 = d.bbox.astype(int)
        cv2.rectangle(image, (x1, y1), (x2, y2), (40, 230, 80), 2)
        cv2.putText(image, f'{d.label} {d.confidence:.2f}', (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (40, 230, 80), 1)
    if grasp is not None:
        cv2.polylines(image, [grasp.rectangle.astype(np.int32)], True, (255, 210, 30), 2)
        cv2.drawMarker(image, tuple(grasp.center_px.astype(int)), (255, 40, 40), cv2.MARKER_CROSS, 16, 2)
    return image
