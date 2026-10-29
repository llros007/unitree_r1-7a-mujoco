# SPDX-License-Identifier: AGPL-3.0-or-later
"""Ultralytics detections adapted to the simple RGB-D grasp pipeline."""
from dataclasses import dataclass
from contextlib import contextmanager
import os
from pathlib import Path

import cv2
import numpy as np


@contextmanager
def model_cache():
    """Ultralytics downloads text encoders relative to the working directory."""
    previous = Path.cwd()
    cache = Path(__file__).resolve().parent / 'models'
    cache.mkdir(parents=True, exist_ok=True)
    try:
        os.chdir(cache)
        yield
    finally:
        os.chdir(previous)


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: np.ndarray
    mask: np.ndarray


class Detector:
    def __init__(self, weights, labels, device='cpu', confidence=.25):
        from ultralytics import YOLO, YOLOE, YOLOWorld
        if not 0 < confidence <= 1 or not labels:
            raise ValueError('Provide target classes and a confidence threshold in (0, 1]')
        weights = Path(weights).expanduser().resolve()
        name = weights.name.lower()
        with model_cache():
            if 'yoloe' in name:
                self.model = YOLOE(str(weights))
                self.model.set_classes(labels)
            elif 'world' in name:
                self.model = YOLOWorld(str(weights))
                self.model.set_classes(labels)
            else:
                self.model = YOLO(str(weights))
        self.labels = set(labels)
        self.device, self.confidence = device, confidence

    def detect(self, rgb):
        # Ultralytics numpy input uses BGR; MuJoCo returns RGB.
        results = self.model.predict(np.ascontiguousarray(rgb[:, :, ::-1]),
                                     conf=self.confidence, device=self.device,
                                     imgsz=640, verbose=False)
        detections = []
        height, width = rgb.shape[:2]
        for result in results:
            if result.boxes is None:
                continue
            for i, box in enumerate(result.boxes):
                label = result.names[int(box.cls.item())]
                if label not in self.labels:
                    continue
                bbox = box.xyxy[0].cpu().numpy()
                bbox[[0, 2]] = np.clip(bbox[[0, 2]], 0, width - 1)
                bbox[[1, 3]] = np.clip(bbox[[1, 3]], 0, height - 1)
                mask = np.zeros((height, width), dtype=np.uint8)
                if result.masks is not None:
                    # Polygon coordinates are already in original image coordinates.
                    polygon = np.round(result.masks.xy[i]).astype(np.int32)
                    if len(polygon) >= 3:
                        cv2.fillPoly(mask, [polygon], 1)
                else:
                    x1, y1, x2, y2 = np.round(bbox).astype(int)
                    mask[y1:y2 + 1, x1:x2 + 1] = 1
                detections.append(Detection(label, float(box.conf.item()), bbox, mask))
        return detections
