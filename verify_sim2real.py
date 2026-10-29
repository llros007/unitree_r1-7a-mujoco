"""Offline SDK command parity, command hold, camera payload and trace checks."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory

import mujoco
import numpy as np

from environment import ArmEnvironment
from recording import JointRecorder
from sdk_protocol import SdkProtocol
from wrist_scene import build_scene

ROOT = Path(__file__).resolve().parent


def verify_sdk(sdk_root):
    source = sdk_root / 'thirdparty/motor_controller/src/MotorProtocol.cpp'
    compiler = shutil.which('g++')
    if not compiler or not source.exists():
        return {'status': 'not run; requires g++ and sibling SDK sources'}
    codec = SdkProtocol(ROOT / 'assets/joint_calib.txt')
    rng = np.random.default_rng(29)
    lines, expected = [], []
    for _ in range(32):
        values = [rng.uniform(-1, 2, 7), rng.uniform(-.5, .5, 7), rng.uniform(-5, 5, 7),
                  rng.uniform(0, 150, 7), rng.uniform(0, 6, 7)]
        codec.round_trip(*values)
        expected.extend(codec.raw.T.tolist())
        q, dq, tau, kp, kd = [x.astype(np.float32) for x in values]
        q = (q - codec.offset) / codec.axis
        dq, tau = dq / codec.axis, tau / codec.axis
        for i in range(7):
            lines.append(' '.join(str(x) for x in [i, q[i], dq[i], tau[i], kp[i], kd[i]]))
    harness = r'''
#include "MotorProtocol.h"
#include <iostream>
#include <cstring>
int main() {
    MotorProtocol protocol;
    int id; float q, dq, tau, kp, kd;
    while (std::cin >> id >> q >> dq >> tau >> kp >> kd) {
        auto p = protocol.buildControlPacket(id, 1, true, tau, dq, q, kp, kd);
        int32_t qr; int16_t dqr, tr, kpr, kdr;
        std::memcpy(&tr, &p[4], 2); std::memcpy(&dqr, &p[6], 2);
        std::memcpy(&qr, &p[8], 4); std::memcpy(&kpr, &p[12], 2); std::memcpy(&kdr, &p[14], 2);
        std::cout << qr << ' ' << dqr << ' ' << tr << ' ' << kpr << ' ' << kdr << '\n';
    }
}
'''
    with TemporaryDirectory() as folder:
        folder = Path(folder)
        test = folder / 'sdk_check.cpp'
        test.write_text(harness)
        executable = folder / 'sdk_check'
        subprocess.run([compiler, '-std=c++17', '-I', str(source.parent.parent / 'include'),
                        str(test), str(source), '-o', str(executable)], check=True, capture_output=True)
        output = subprocess.run([str(executable)], input='\n'.join(lines), text=True,
                                check=True, capture_output=True).stdout
    actual = np.array([[int(x) for x in row.split()] for row in output.splitlines()])
    np.testing.assert_array_equal(actual, expected)
    return {'status': 'passed', 'joint_commands_compared': len(expected),
            'fields': ['q', 'dq', 'tau', 'kp', 'kd']}


def verify():
    env = ArmEnvironment(scene_path=build_scene())
    assert env.motors.command_period == .01 and env.motors.protocol is not None
    env.move_target(np.array([.01, 0, 0]))
    env.step(.002)
    held = env.motors.q.copy()
    for _ in range(4):
        env.step(.002)
        np.testing.assert_array_equal(env.motors.q, held)
    env.step(.002)
    assert not np.array_equal(env.motors.q, held), '100 Hz host command did not advance'
    env.reset()
    for _ in range(300):
        env.step()
    baseline = float(np.linalg.norm(env.tcp_position - env.target_position))
    nominal_gravity = env.gravity_torque(env.home_q)
    masses = {name: float(env.model.body_mass[env.model.body(name).id])
              for name in ['hand', 'moving_jaw', 'd435_camera']}
    assert abs(sum(masses.values()) - .265) < 1e-9
    # Hypothetical uncertainty check, not a claimed hardware identification:
    # increase physical end-effector mass while keeping the controller model.
    for name in masses:
        body = env.model.body(name).id
        env.model.body_mass[body] *= 1.2
        env.model.body_inertia[body] *= 1.2
    position = env.data.qpos.copy()
    mujoco.mj_setConst(env.model, env.data)
    env.data.qpos[:] = position
    np.testing.assert_array_equal(env.gravity_torque(env.home_q), nominal_gravity)
    env.reset()
    for _ in range(300):
        env.step()
    mismatch_error = float(np.linalg.norm(env.tcp_position - env.target_position))
    assert mismatch_error > baseline * 3, 'Controller should not know injected physical mass changes'
    assert np.isfinite(env.data.qpos).all() and not any(w.number for w in env.data.warning)
    trace_env = ArmEnvironment(scene_path=env.scene_path)
    with TemporaryDirectory() as folder:
        path = Path(folder) / 'trace.csv'
        with JointRecorder(trace_env, path):
            trace_env.step()
            trace_env.step()
            trace_env.reset()
            trace_env.step()
        rows = list(csv.DictReader(path.open()))
        metadata = json.loads(path.with_suffix('.metadata.json').read_text())
        assert len(rows) == 3 and [int(x['episode']) for x in rows] == [0, 0, 1]
        assert 'd435_camera' in metadata['end_effector_model']
        assert 'simulation' in metadata['source'] and trace_env.step_observer is None
    return {'host_command_hold': 'passed, 100 Hz', 'recording_and_reset': 'passed',
            'end_effector_model_mass_kg': masses,
            'nominal_hold_error_m': baseline,
            'hypothetical_20_percent_heavier_end_effector_error_m': mismatch_error,
            'hardware_accuracy': 'not measured; no hardware logs available'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root', type=Path, default=ROOT.parent / 'r1-7a_sdk')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = {'sdk_protocol': verify_sdk(args.sdk_root), **verify()}
    result = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(result)
    print(result)
