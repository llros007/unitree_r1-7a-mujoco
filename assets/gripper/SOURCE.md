# XLeRobot SO101 TPU95A Fin Ray gripper assets

Source: https://github.com/Vector-Wangel/XLeRobot/tree/3d14695e40c9c68229c0aacffca6053c75cd3eb6/hardware

Local source commit: `3d14695e40c9c68229c0aacffca6053c75cd3eb6`

Project: `hardware/XLeRobot_0_3_0.3mf`, mesh objects 47 (fixed PLA holder), 39 (moving PLA holder), 49 (TPU Fin Ray finger).

Source SHA256: `4b41ae03895144ffff0a22b0d1a76e02d4ff0db0bc5ff0066ab528df8ce4fa4d`

Modified: extracted triangle meshes, converted mm to m, and applied rigid rotations/translations for a simulation-only R1 flange adapter. No scaling of the physical dimensions. The two fin meshes are two placements of the same source finger.

Motion uses one fixed finger and one servo-driven rotary jaw. The adapter, TCP, collision proxies, masses and contact parameters are simulation approximations, not a manufacturer-provided R1 mounting design or measured TPU material model. Contact compliance does not simulate Fin Ray rib deformation.

License: Apache-2.0, see LICENSE.
