# SPDX-License-Identifier: AGPL-3.0-or-later
"""Compatibility entry for the shared scene and camera constants."""
from wrist_scene import CAMERA_NAME, FOVY, MIN_DEPTH, MAX_DEPTH, build_scene as _build_scene


def build_scene(object_kind='plant', config_path=None):
    return _build_scene(config_path, object_kind=object_kind)
