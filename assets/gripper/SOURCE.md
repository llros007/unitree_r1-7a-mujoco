# XLeRobot SO101 TPU95A Fin Ray gripper assets

Source: https://github.com/Vector-Wangel/XLeRobot/tree/3d14695e40c9c68229c0aacffca6053c75cd3eb6/hardware

Local source commit: `3d14695e40c9c68229c0aacffca6053c75cd3eb6`

Project: `hardware/XLeRobot_0_3_0.3mf`, mesh objects 47 (fixed PLA holder), 39 (moving PLA holder), 49 (TPU Fin Ray finger).

Source SHA256: `4b41ae03895144ffff0a22b0d1a76e02d4ff0db0bc5ff0066ab528df8ce4fa4d`

Modified: extracted triangle meshes, converted mm to m, and applied rigid rotations/translations for a simulation-only R1 flange adapter. No scaling of the physical dimensions. The two fin meshes are two placements of the same source finger.

Motion uses one fixed finger and one servo-driven rotary jaw. The adapter, TCP, collision proxies, masses and contact parameters are simulation approximations, not a manufacturer-provided R1 mounting design or measured TPU material model. Contact compliance does not simulate Fin Ray rib deformation.

License: Apache-2.0, see LICENSE.

## Radial flange attachment correction

The SDK Link7 mesh has its radial mounting plate near local Y=-47.736 mm.
The hand attaches at `(0,-0.061,0)` m with tool +Z along -Link7 Y,
tool +X along -Link7 Z and tool +Y along Link7 X. This follows the user's
CAD connection reference; the adapter thickness and stand-off are approximate.
The former `(0,0,0.04)` +Z mounting incorrectly attached the tool to the
side of the wrist. See `xle_gripper.configure_mount` for the authoritative
transform; TCP, camera and local inertias follow the hand frame.
