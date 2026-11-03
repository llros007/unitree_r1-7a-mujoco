"""Offline hardware-teleop regressions. Never opens a serial port."""
import os
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import glfw
import numpy as np

from environment import ArmEnvironment
from real_keyboard import (Bus, Session, TRACK_ABORT_RAD, check_command, check_health,
                           command_from_env, handle_key, hold_command, make_controller,
                           parse_args, parse_state, reanchor)


class FakeBus:
    def __init__(self, q=None):
        self.q = np.zeros(7) if q is None else q.copy()
        self.dq, self.tau = np.zeros(7), np.zeros(7)
        self.mode, self.err = np.ones(7, dtype=int), np.zeros(7, dtype=int)
        self.sent, self.times, self.stopped = [], [], False

    def send(self, *command):
        self.sent.append(tuple(x.copy() for x in command))
        self.times.append(time.monotonic())
        # Ideal joint response, only for command/loop tests.
        self.q, self.dq, self.tau = [x.copy() for x in command[:3]]

    def stop(self):
        self.stopped = True


def args(mode='sim', hardware=False):
    return SimpleNamespace(mode=mode, hardware=hardware, joint_speed=20.,
                           bridge=Path('/unused'), calib=Path('/unused'),
                           port='/unused', feedback_timeout=.04)


class TeleopChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env = ArmEnvironment(initial_pose='zero')

    def setUp(self):
        self.env.reset()

    def test_no_implicit_hardware(self):
        with patch('real_keyboard.os.path.exists', return_value=True):
            parsed = parse_args(['--mode', 'sim'])
        self.assertFalse(parsed.hardware)
        self.assertTrue(str(parsed.calib).endswith('sim_r1-7a/r1-7a_sdk/config/joint_calib.txt'))

    def test_reset_and_tab_rebase_on_hardware_in_both_modes(self):
        for mode in ('sim', 'sync'):
            for key in (glfw.KEY_R, glfw.KEY_TAB):
                self.env.reset()
                session = Session(self.env, args(mode))
                q = np.zeros(7); q[3] = np.deg2rad(10)
                bus = FakeBus(q)
                session.events.put((key, glfw.PRESS, 0))
                session.tick(bus)
                if key == glfw.KEY_R:
                    self.assertLess(np.max(np.abs(bus.sent[-1][0]-q)), np.deg2rad(.05))
                    self.assertLessEqual(np.max(np.abs(bus.sent[-1][1])), np.deg2rad(5))
                    self.assertLess(bus.sent[-1][0][3], q[3])
                else:
                    np.testing.assert_allclose(bus.sent[-1][0], q, atol=1e-9)
                    np.testing.assert_allclose(bus.sent[-1][1], 0, atol=1e-9)
                self.assertEqual(session.control.control_mode,
                                 'joint' if key == glfw.KEY_TAB else 'cartesian')

    def test_pause_resume_discards_pending_motion(self):
        for mode in ('sim', 'sync'):
            self.env.reset()
            session = Session(self.env, args(mode))
            bus = FakeBus()
            self.env.q_command[3] = np.deg2rad(5)
            command_from_env(self.env)
            self.assertGreater(self.env.reference.dq[3], 0)
            session.events.put((glfw.KEY_P, glfw.PRESS, 0))
            session.tick(bus)
            paused_target = bus.sent[-1][0].copy()
            np.testing.assert_array_equal(bus.sent[-1][1], np.zeros(7))
            np.testing.assert_array_equal(self.env.reference.dq, np.zeros(7))
            for _ in range(3):
                session.tick(bus)
                np.testing.assert_array_equal(bus.sent[-1][0], paused_target)
            session.events.put((glfw.KEY_P, glfw.PRESS, 0))
            session.tick(bus)
            np.testing.assert_allclose(bus.sent[-1][0], paused_target, atol=1e-9)

    def test_foc_exit_and_fault_block_next_command(self):
        for mode in ('sim', 'sync'):
            for attr in ('mode', 'err'):
                session = Session(self.env, args(mode))
                bus = FakeBus()
                getattr(bus, attr)[2] = 0 if attr == 'mode' else 8
                with self.assertRaises(RuntimeError):
                    session.tick(bus)
                self.assertEqual(len(bus.sent), 0)

    def test_detect_foc_exit_in_response(self):
        for mode in ('sim', 'sync'):
            bus = FakeBus()
            def send(*command):
                bus.mode[2] = 0
            bus.send = send
            with self.assertRaises(RuntimeError):
                Session(self.env, args(mode)).tick(bus)

    def test_tracking_and_estimated_torque_guards(self):
        bus = FakeBus()
        command = list(command_from_env(self.env))
        command[0] = np.full(7, TRACK_ABORT_RAD + .01)
        with self.assertRaisesRegex(RuntimeError, '跟踪误差'):
            check_command(self.env, bus, command)
        command[0] = np.zeros(7)
        command[2] = self.env.motors.tau_limit + 1
        with self.assertRaisesRegex(RuntimeError, '总力矩'):
            check_command(self.env, bus, command)

    def test_lost_ui_heartbeat_holds_and_pauses(self):
        session = Session(self.env, args())
        session.control.keys.add(glfw.KEY_UP)
        session.input_at = time.monotonic() - 1
        session.tick(FakeBus())
        self.assertTrue(self.env.paused)
        self.assertFalse(session.control.keys)
        np.testing.assert_array_equal(self.env.motors.host_command[1], np.zeros(7))

    def test_pure_sim_does_not_auto_pause_on_render_delay(self):
        session = Session(self.env, args())
        session.input_at = time.monotonic() - 10
        session.events.put((glfw.KEY_TAB, glfw.PRESS, 0))
        session.events.put((glfw.KEY_4, glfw.PRESS, 0))
        session.events.put((glfw.KEY_UP, glfw.PRESS, 0))
        before = self.env.q_command[3]
        for _ in range(20):
            session.tick(None)
        self.assertFalse(self.env.paused)
        self.assertGreater(self.env.q_command[3], before)
        self.assertGreater(self.env.data.qpos[self.env.qids[3]], before)

    def test_joint_start_mode(self):
        options = args()
        options.control_mode = 'joint'
        session = Session(self.env, options)
        self.assertEqual(session.control.control_mode, 'joint')

    def test_both_hardware_modes_require_stable_enable(self):
        for mode in ('sim', 'sync'):
            bus = FakeBus()
            session = Session(self.env, args(mode, hardware=True))
            reanchor(self.env, bus)
            session.enable(bus)
            self.assertEqual(len(bus.sent), 20)
            for command in bus.sent:
                np.testing.assert_allclose(command[0], 0, atol=1e-9)
                np.testing.assert_allclose(command[1], 0, atol=1e-9)

    def test_worker_continues_without_render_calls_and_stops_bus(self):
        bus = FakeBus()
        session = Session(self.env, args(hardware=True))
        with patch('real_keyboard.Bus', return_value=bus):
            session.thread.start()
            self.assertTrue(session.ready.wait(2))
            before = len(bus.times)
            # A render stall shorter than the UI watchdog must not stop TX.
            time.sleep(.16)
            session.stop_event.set()
            session.thread.join(2)
        self.assertIsNone(session.error)
        self.assertFalse(session.thread.is_alive())
        self.assertTrue(bus.stopped)
        self.assertGreaterEqual(len(bus.times)-before, 6)
        gaps = np.diff(bus.times[before:])
        self.assertTrue(np.all(gaps > .003), 'No catch-up command burst')

    def test_invalid_start_pose_stops_without_sending_a_clipped_target(self):
        bus = FakeBus()
        bus.q[1] = -.1
        session = Session(self.env, args(hardware=True))
        with patch('real_keyboard.Bus', return_value=bus):
            session.run()
        self.assertIn('启动实测角', session.error)
        self.assertEqual(bus.sent, [])
        self.assertTrue(bus.stopped)

    def test_fault_during_enable_stops_worker(self):
        bus = FakeBus()
        bus.err[1] = 8
        session = Session(self.env, args(hardware=True))
        with patch('real_keyboard.Bus', return_value=bus):
            session.run()
        self.assertIn('报错', session.error)
        self.assertTrue(bus.stopped)
        self.assertEqual(bus.sent, [])

    def test_calibration_stays_with_config_despite_sibling(self):
        with tempfile.TemporaryDirectory() as tmp:
            import json, shutil
            top = Path(tmp)
            config_dir = top/'project'; config_dir.mkdir()
            sibling = top/'r1-7a_sdk/config'; sibling.mkdir(parents=True)
            source = Path(__file__).parent
            shutil.copy(source/'r1-7a_sdk/config/joint_calib.txt', sibling/'joint_calib.txt')
            shutil.copy(source/'assets/joint_calib.txt', config_dir/'joint_calib.txt')
            config = json.loads((source/'motor_config.json').read_text())
            config['sdk_protocol_calibration'] = 'joint_calib.txt'
            path = config_dir/'motor.json'; path.write_text(json.dumps(config))
            env = ArmEnvironment(motor_config=path, initial_pose='zero')
            self.assertEqual(env.motors.protocol.path, config_dir/'joint_calib.txt')

    def test_bus_timeout_and_malformed_state(self):
        for value in ('nan', 'inf'):
            with self.assertRaises(RuntimeError):
                parse_state('STATE '+' '.join([value]*35))
        with tempfile.TemporaryDirectory() as tmp:
            bridge = Path(tmp)/'fake-bridge'
            bridge.write_text('#!/usr/bin/env python3\nimport sys,time\n'
                              'print("STATE " + " ".join(["0"]*21+["1"]*7+["0"]*7), flush=True)\n'
                              'for line in sys.stdin:\n'
                              ' if line.strip()=="STOP": break\n'
                              ' time.sleep(.3)\n')
            bridge.chmod(0o755)
            bus = Bus(bridge, '/unused', '/unused', timeout=.03)
            start = time.monotonic()
            try:
                with self.assertRaises(TimeoutError):
                    bus.send(*[np.zeros(7)]*5)
                self.assertLess(time.monotonic()-start, .2)
            finally:
                bus.stop()
            self.assertIsNotNone(bus.proc.poll())


if __name__ == '__main__':
    unittest.main(verbosity=2)
