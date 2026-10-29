# SPDX-License-Identifier: AGPL-3.0-or-later
"""Search safety regressions using the real motor/IK dynamics, without rendering."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from environment import ArmEnvironment
from yolo_grasp.search import SearchPickPlace
from motion import ScriptedGrasp, DOWN_ROTATION


def verify():
    env = ArmEnvironment(object_kind='cube')
    probes = []
    def probe():
        probes.append(float(env.data.time))
        return False
    def forbidden_localize():
        raise AssertionError('Empty scans must never localize or start grasping')
    controller = SearchPickPlace(env, probe, forbidden_localize)
    controller.begin_search()
    env.paused = True
    controller.tick()
    assert controller.elapsed == 0
    env.paused = False
    for _ in range(10000):
        controller.tick()
        env.step()
        if not controller.active:
            break
    assert controller.phase == 'failed' and len(probes) == len(controller.views)
    assert env.gripper_open and not controller.actions
    controller.begin_search()
    controller.cancel()
    assert not controller.active and controller.phase == 'idle'
    # An invalid reacquisition must advance to a new view and obtain a fresh
    # grasp. Search detections themselves cannot serve as the grasp pose.
    env.reset()
    captures = []
    grasp = ScriptedGrasp(env.cube_position + [0, 0, .02],
                          env.cube_position + [0, 0, .006], DOWN_ROTATION)
    def localize():
        captures.append(float(env.data.time))
        if len(captures) == 1:
            raise ValueError('invalid depth')
        return grasp
    controller = SearchPickPlace(env, lambda: True, localize)
    controller.begin_search()
    for _ in range(5000):
        controller.tick()
        env.step()
        if not controller.phase.startswith('scan_'):
            break
    assert controller.phase == 'align' and controller.grasp is grasp and len(captures) == 2
    events = controller.report['search']
    assert any(event['event'] == 'localization_rejected' for event in events)
    detected = [event['timestamp_s'] for event in events if event['event'] == 'detected']
    assert all(capture - detection >= .49 for capture, detection in zip(captures, detected))
    controller.cancel()
    print('Search: six views, empty target, invalid-depth retry, fresh capture, pause and cancel passed')


if __name__ == '__main__':
    verify()
