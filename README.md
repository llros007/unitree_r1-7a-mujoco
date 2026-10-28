# Unitree R1-7A MuJoCo

**English** | [简体中文](README.zh-CN.md)

A standalone MuJoCo simulation of the Unitree R1-7A seven-axis arm with an XLeRobot / SO101 TPU95A Fin Ray gripper. Move and rotate the end effector with your keyboard, grasp a free cube through physical contact, and transfer it to a drop zone.

![Silver R1-7A arm lifting a cube](grasp_preview.png)

The scene includes the original arm meshes, joint limits and inertial properties from the Unitree SDK. The silver housing and dark end interface visually approximate the [official product photos](https://www.unitree.com/R1-ARM/); they are not an official numeric color specification. Runtime assets are included: ROS, the SDK and XLeRobot checkouts are not required to run the simulation.

## Quick start

Requires Python 3.10+ and an OpenGL desktop for the interactive viewer. Tested on Linux with Python 3.12.

```bash
git clone https://github.com/llros007/unitree_r1-7a-mujoco.git
cd unitree_r1-7a-mujoco
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
./run.sh
```

Alternatively, install the requirements in an activated conda environment and run `python main.py`. Set `PYTHON=/path/to/python ./run.sh` to select an interpreter explicitly. The launch script resolves its own directory, so it can be called from another working directory.

## Controls

Click the simulation window to focus it. Hold keys for continuous motion; release them to hold the target pose. Translation and rotation use world coordinates.

| Key / mouse | Action |
| --- | --- |
| W / S | Move along +Y / −Y |
| A / D | Move along −X / +X |
| Q / E | Raise / lower along Z |
| I / K | Rotate about +X / −X |
| J / L | Rotate about +Y / −Y |
| U / O | Rotate about +Z / −Z |
| Shift + motion | Fine adjustment |
| Space | Toggle gripper open / closed |
| R | Reset the arm, cube and gripper |
| P | Pause / resume |
| H | Print help in the terminal |
| Esc | Exit |
| Left drag | Orbit camera |
| Right drag | Pan camera |
| Scroll | Zoom |

Translation speed is 0.10 m/s, or 0.025 m/s with Shift. Rotation speed is 0.75 rad/s, or 0.25 rad/s with Shift. The overlay shows actual and target TCP positions, cube position and finger contact status. The green marker is the actual TCP; the yellow marker is the target.

## Pick and place

1. The initial TCP points down above the cube at approximately `(0.12, 0.25, 0.80)` m.
2. Hold **E** until the target TCP Z approaches **0.626 m**. Release and let the arm settle. The table surface is at Z = 0.60 m; the cube is 0.04 m wide and weighs 55 g.
3. Press **Space** to close the gripper and wait about a second. `contacts 2/2` means both fingers contact the cube.
4. Hold **Q** to lift. The overlay displays `CUBE LIFTED!` when the cube clears the table.
5. Hold **A** to move toward the green drop zone at approximately `(-0.12, 0.25)` m. Lower the target TCP to about Z = 0.633 m, stop moving, then press **Space** to release.

The inverse kinematics controller limits targets to the reachable workspace. It does not plan collision-free paths; control the route manually and use **R** to reset if needed.

## Headless verification and screenshots

Run a complete physical grasp, transfer and release without opening a window:

```bash
./run.sh --headless --demo
python verify.py
```

Both commands return a nonzero exit code on failure. `verify.py` also exercises the viewer's keyboard handlers for descent, closure, lift, pause and reset, and checks for non-finite state and MuJoCo warnings. Pure headless verification does not create an OpenGL context.

Save a rendered image at the lift phase:

```bash
./run.sh --headless --demo --snapshot grasp_preview.png
```

On a server with EGL support:

```bash
MUJOCO_GL=egl ./run.sh --headless --demo --snapshot grasp_preview.png
```

A screenshot requires a working rendering backend. For a short interactive window check, run `./run.sh --seconds 3`.

## Model and simulation details

- Fixed-base seven-axis arm, 500 Hz physics and 100 Hz control, with damped least-squares Cartesian position and orientation IK.
- Gravity compensation and force-limited position servos for interactive control. Arm collisions with the table, floor and cube are enabled; arm self-collision is disabled. Arm mesh collisions use MuJoCo convex hulls.
- A fixed finger and a single servo-driven rotating jaw, using original PLA holder and TPU finger meshes extracted from XLeRobot's 3MF project. Dimensions are preserved with millimeter-to-meter conversion and rigid coordinate transforms.
- The cube remains a free rigid body. Grasping uses contact friction, without welding, attachment constraints or cube teleportation.
- Gripper opening target: 0.8 rad; closing target: −0.13 rad; joint range: [−0.20, 1.05] rad. These are simulation settings, not hardware servo calibration.
- TCP: `(0, 0, 0.106)` m in the adapted gripper frame, 0.146 m from the original Link7 origin.
- Fin Ray ribs are visible, but the fingers are rigid meshes. Segmented collision proxies and soft contact (`solref=0.012 1`, friction 1.8) approximate compliance; TPU elasticity and rib bending are not simulated.

The flange adapter, mounting pose, TCP, gripper masses, inertias, friction, compliance and drive torque are simulation approximations without hardware identification. Use this project for interaction and algorithm prototyping; it does not establish real grasp force or deformation. The program does not connect to or control a physical arm.

## Project files

| File | Purpose |
| --- | --- |
| `main.py` | GLFW viewer, keyboard/mouse controls, overlay and snapshots |
| `environment.py` | Dynamics, Cartesian IK and scripted pick-and-place validation |
| `build_scene.py` | Optional scene regeneration from the Unitree SDK URDF |
| `xle_gripper.py` | SO101 rotary gripper, TCP and contact parameters |
| `prepare_gripper.py` | Optional mesh extraction from the XLeRobot 3MF project |
| `verify.py` | Headless keyboard and physical transfer regression checks |
| `assets/scene.xml` | Ready-to-run MJCF scene |
| `assets/meshes/` | Original arm meshes copied from the SDK |
| `assets/gripper/` | Transformed gripper meshes, license and provenance |
| `grasp_preview.png` | Screenshot of a physical cube lift |

## Optional asset regeneration

All assets required for normal use are already included.

To rebuild the arm scene, clone the [Unitree SDK](https://github.com/unitreerobotics/r1-7a_sdk) beside this repository as `../r1-7a_sdk`, then run:

```bash
python build_scene.py
```

The included arm meshes originate from SDK commit `2c89fe4f60f0c98f1bc06795cb2477cc73d0c92c`.

To re-extract the gripper, check out the XLeRobot revision recorded in [assets/gripper/SOURCE.md](assets/gripper/SOURCE.md), then run:

```bash
python prepare_gripper.py --source /path/to/XLeRobot
```

The extraction uses objects 47 (fixed holder), 39 (moving holder) and 49 (TPU finger) from `hardware/XLeRobot_0_3_0.3mf`. The provenance file records the source commit, source hash and transformations.

## License and acknowledgments

Project code is licensed under [BSD-3-Clause](LICENSE). Unitree arm assets retain their original [BSD-3-Clause license](assets/SDK_LICENSE). XLeRobot gripper assets retain their [Apache-2.0 license](assets/gripper/LICENSE), with source and modification details in [SOURCE.md](assets/gripper/SOURCE.md).

This is a community simulation project. Thanks to [Unitree Robotics](https://github.com/unitreerobotics/r1-7a_sdk), [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) and [MuJoCo](https://mujoco.readthedocs.io/).
