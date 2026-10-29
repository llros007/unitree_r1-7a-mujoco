# D435 wrist camera: sources and simulation assumptions

## Community layout reference

- TheRobotStudio SO-ARM100 / SO-101 optional D435/D435i wrist mount:
  https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Wrist_Cam_Mount_RealSense_D435
- Assembly text:
  https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Optional/Wrist_Cam_Mount_RealSense_D435/README.md
- Community photographs: `media/d435_mount.jpg` and `media/d435_mount_view.png` in that repository.

The reference mounts a horizontal camera bar above the gripper wrist, centered for depth sensing. Its instructions specify five small wrist screws and two M3x6 camera screws. The current simulation uses that layout concept with an original simplified bracket for the existing XLeRobot TPU gripper. It does not include or modify the community STL. The actual mounting dimensions, fastener clearances, cable routing and compatibility with the TPU fixed holder have not been mechanically validated.

## Manufacturer references

- D435 dimensions (90x25x25 mm):
  https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html
- D400 Series datasheet, Minimum-Z Depth table: D435 640x480 depth minimum Z = 175 mm:
  https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf

## Coordinate model

`hand` is the fixed SO101 gripper body in the existing simulator. Its +Z points from the wrist toward the fingers; X is the jaw opening axis. The camera case center is at `(0, -0.050, 0.018)` m in `hand`. A -15 degree local X rotation points the lens toward the finger region. The camera RGB optical origin is `(0.035, 0, 0.014)` m in the case frame.

The rigid transform at each capture is:

```
T_world_optical = T_world_hand @ T_hand_case @ T_case_MuJoCo_camera @ diag(1, -1, -1, 1)
```

MuJoCo uses camera +X right, +Y up and -Z forward. Backprojection uses OpenCV +X right, +Y down and +Z forward. `camera.py` obtains the named camera's current `cam_xpos` and `cam_xmat` after `mj_forward`, then applies the axis conversion. The transform is frozen together with the RGB-D frame for detection; it is never replaced with a later wrist pose when calculating that frame's target position.

## RGB-D approximation

640x480 color and depth are rendered with the same ideal pinhole projection. A vertical FOV of 42.5 degrees approximates a 4:3 center crop of the D435 RGB view. Depth outside [0.175, 3.0] optical meters is zeroed. The camera and bracket add an approximate 90 g fixed payload. The body has a collision envelope, but the simplified mount and lens decorations are visual geometry.

This is a simulated D435-style, registered RGB-D stream. It is not a factory-calibrated D435 model: real RGB/depth camera intrinsics, their relative transform, distortion, baseline occlusions, stereo matching, exposure, projector patterns and noise are not reproduced. Real use requires intrinsics from the actual streams, depth-to-color alignment and hand-eye calibration of the installed camera. No physical camera or robot is opened by this module.
