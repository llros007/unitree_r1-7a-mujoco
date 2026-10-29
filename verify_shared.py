"""Check both entry points share scene, parameters, home pose and dynamics."""
import json
from pathlib import Path

import numpy as np

from environment import ArmEnvironment
from motion import PickPlace
from wrist_scene import build_scene
from yolo_grasp.controller import PickPlace as YoloPickPlace
from yolo_grasp.wrist_mount import build_scene as build_yolo_scene

ROOT = Path(__file__).resolve().parent


def verify():
    assert PickPlace is YoloPickPlace, 'Motion controllers must be the same implementation'
    assert (ROOT / 'motor_config.json').samefile(ROOT / 'yolo_grasp/motor_config.json')
    keyboard = ArmEnvironment()
    yolo = ArmEnvironment(scene_path=build_yolo_scene())
    assert keyboard.scene_path == yolo.scene_path == build_scene().resolve()
    assert keyboard.object_kind == yolo.object_kind == 'plant'
    from yolo_grasp.plant_asset import ASSETS, POT_HEIGHT
    vertices = np.array([list(map(float, line.split()[1:]))
                         for line in (ASSETS / 'potted_plant_04_pot.obj').read_text().splitlines()
                         if line.startswith('v ')])
    np.testing.assert_allclose(np.ptp(vertices, axis=0), [.06, .06, POT_HEIGHT], atol=2e-9)
    assert keyboard.motors.config == yolo.motors.config
    assert keyboard.model.opt.noslip_iterations == yolo.model.opt.noslip_iterations == 5
    assert keyboard.gripper_speed == yolo.gripper_speed == .3
    for name in ['body_mass', 'body_inertia', 'geom_friction', 'dof_damping',
                 'dof_armature', 'actuator_forcerange', 'body_gravcomp']:
        np.testing.assert_array_equal(getattr(keyboard.model, name), getattr(yolo.model, name))
    np.testing.assert_array_equal(keyboard.home_q, yolo.home_q)
    for index in range(100):
        if index < 40:
            for env in [keyboard, yolo]:
                env.move_target(np.array([.00025, 0, 0]))
        for env in [keyboard, yolo]:
            env.step()
        np.testing.assert_array_equal(keyboard.data.qpos, yolo.data.qpos)
        np.testing.assert_array_equal(keyboard.data.qvel, yolo.data.qvel)
        np.testing.assert_array_equal(keyboard.motors.applied_tau, yolo.motors.applied_tau)
    return {'shared_scene': str(keyboard.scene_path), 'object': 'plant',
            'shared_motion_controller': True, 'same_dynamics_over_100_steps': True,
            'kp': keyboard.motors.default_kp.tolist(), 'kd': keyboard.motors.default_kd.tolist(),
            'gravity_feedforward_scale': keyboard.motors.gravity_feedforward_scale,
            'command_hz': 1 / keyboard.motors.command_period,
            'gripper_speed_rad_s': keyboard.gripper_speed}


if __name__ == '__main__':
    print(json.dumps(verify(), indent=2))
