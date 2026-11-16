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
                           parse_args, parse_state, reanchor, SDK_CALIB)


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
                           port='/unused', feedback_timeout=.04, home_speed_deg=5., home_accel_deg=10.)


class TeleopChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env = ArmEnvironment(initial_pose='zero')

    def setUp(self):
        self.env.reset()

    def test_leader_starts_with_small_encoder_offset_below_joint_limit(self):
        options = args('sync', hardware=True)
        options.leader_scale = .7
        options.joint_speed = 5.
        session = Session(self.env, options)
        bus = FakeBus(np.deg2rad([0, .5, 0, -.16, 0, 0, 0]))
        reanchor(self.env, bus)
        leader_angles = np.zeros(5)
        session.leader = SimpleNamespace(sample=lambda: leader_angles.copy())
        session.events.put((glfw.KEY_SPACE, glfw.PRESS, 0))
        session.tick(bus)
        self.assertIn(glfw.KEY_SPACE, session.control.keys)
        self.assertEqual(self.env.q_command[3], self.env.limits[3, 0])
        leader_angles[0] = np.deg2rad(10)
        session.tick(bus)
        self.assertGreater(self.env.q_command[0], 0)
        self.assertLessEqual(self.env.q_command[0], np.deg2rad(.05) + 1e-9)
        np.testing.assert_array_equal(self.env.q_command[[2, 6]], np.zeros(2))

    def test_leader_elbow_flexion_maps_to_r1_j4_in_both_directions(self):
        from so101_leader import joint_target

        q = np.deg2rad([0, 0, 0, 45, 0, 0, 0])
        show = FakeBus(q)
        reanchor(self.env, show)
        for sign in (-1, 1):
            delta = np.deg2rad([0, 0, 10 * sign, 0, 0])
            target = joint_target(self.env, q, delta, .7, np.deg2rad(20.))
            self.assertIsNotNone(target)
            self.assertGreater(-sign * (target[3] - q[3]), 0)
            np.testing.assert_array_equal(target[[0, 1, 2, 4, 5, 6]], q[[0, 1, 2, 4, 5, 6]])

    def test_leader_sim_pauses_when_ui_stalls(self):
        options = args()
        options.leader_scale = .7
        session = Session(self.env, options)
        session.leader = SimpleNamespace(sample=lambda: np.zeros(5))
        session.events.put((glfw.KEY_SPACE, glfw.PRESS, 0))
        session.input_at = time.monotonic() - 1.
        session.tick(None)
        self.assertTrue(self.env.paused)
        self.assertIsNone(session.leader_anchor)
        self.assertNotIn(glfw.KEY_SPACE, session.control.keys)

    def test_absolute_leader_reaches_fixed_elbow_target_after_each_engagement(self):
        from so101_leader import load_absolute_points

        options = args()
        options.joint_speed = 40.
        options.leader_absolute_points = load_absolute_points()
        session = Session(self.env, options)
        angles = np.zeros(5)
        session.leader = SimpleNamespace(sample=lambda: np.deg2rad(angles))

        def tick(count=1):
            for _ in range(count):
                session.input_at = time.monotonic()
                session.tick(None)

        session.events.put((glfw.KEY_SPACE, glfw.PRESS, 0))
        tick()
        self.assertAlmostEqual(np.rad2deg(self.env.q_command[3]), 0, places=3)
        angles[2] = -74
        tick(350)
        self.assertAlmostEqual(np.rad2deg(self.env.q_command[3]), 74, places=0)
        session.events.put((glfw.KEY_SPACE, glfw.RELEASE, 0))
        angles[2] = 0
        tick(20)
        self.assertAlmostEqual(np.rad2deg(self.env.q_command[3]), 74, places=3)
        session.events.put((glfw.KEY_SPACE, glfw.PRESS, 0))
        tick(350)
        np.testing.assert_allclose(np.rad2deg(self.env.data.qpos[self.env.qids]),
                                   [0, 0, 0, 0, 0, 0, 0], atol=.5)
        angles[2] = -74
        angles[3:] = 20
        tick(400)
        q = np.rad2deg(self.env.data.qpos[self.env.qids])
        np.testing.assert_allclose(q[[0, 1, 2, 3, 6]], [0, 0, 0, 74, 0], atol=.5)
        np.testing.assert_allclose(q[[4, 5]], [20, -20], atol=.5)

    def test_no_implicit_hardware(self):
        with patch('real_keyboard.os.path.exists', return_value=True):
            parsed = parse_args(['--mode', 'sim'])
        self.assertFalse(parsed.hardware)
        self.assertEqual(parsed.calib, SDK_CALIB.resolve())

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
                    # Planning is asynchronous: first command holds measured pose.
                    np.testing.assert_allclose(bus.sent[-1][0], q, atol=1e-9)
                    deadline = time.monotonic() + 2.
                    while session.home_results.empty() and time.monotonic() < deadline:
                        time.sleep(.001)
                    session.input_at = time.monotonic()
                    session.tick(bus)
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

    def test_enable_timeout_identifies_non_foc_joint(self):
        bus = FakeBus()
        bus.mode[2] = 0
        session = Session(self.env, args(hardware=True))
        reanchor(self.env, bus)
        clock = [0.]
        original = bus.send

        def send(*command):
            original(*command)
            clock[0] += .1

        bus.send = send
        with patch('real_keyboard.time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(session.stop_event, 'wait', return_value=False):
            with self.assertRaisesRegex(RuntimeError, r'J3\(mode=0,非FOC=20次\)') as caught:
                session.enable(bus)
        self.assertIn('最长连续FOC=0', str(caught.exception))
        self.assertIn('最大总线往返=100.0ms', str(caught.exception))

    def test_slow_control_cycle_reports_stage_and_stops(self):
        bus = FakeBus()
        session = Session(self.env, args(hardware=True))
        original = bus.send

        def send(*command):
            original(*command)
            if len(bus.sent) > 20:
                time.sleep(.13)

        bus.send = send
        with patch('real_keyboard.Bus', return_value=bus):
            session.run()
        self.assertIn('控制循环延迟超过100ms', session.error)
        self.assertIn('bus_roundtrip_ms=', session.error)
        self.assertTrue(bus.stopped)
        self.assertEqual(len(bus.sent), 21)

    def test_overdue_reference_does_not_send_a_motion_command(self):
        bus = FakeBus()
        session = Session(self.env, args('sync', hardware=True))
        clock = [time.monotonic()]

        def slow_reference(env):
            clock[0] += .134
            return command_from_env(env)

        with patch('real_keyboard.time.monotonic', side_effect=lambda: clock[0]), \
                patch('real_keyboard.command_from_env', side_effect=slow_reference):
            with self.assertRaisesRegex(TimeoutError, 'reference_and_checks_ms=134'):
                session.tick(bus, deadline=clock[0] + .02)
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

    def test_feedback_info_rejects_stale_or_hot_motors(self):
        bus = FakeBus()
        bus.info = dict(age_s=[.002]*7, driver_temperature_c=[25]*7, winding_temperature_c=[26]*7)
        check_health(bus)
        bus.info['age_s'][3] = .101
        with self.assertRaisesRegex(RuntimeError, 'older than 100ms'):
            check_health(bus)
        bus.info['age_s'][3] = .002
        bus.info['driver_temperature_c'][1] = 61
        with self.assertRaisesRegex(RuntimeError, 'temperature'):
            check_health(bus)

    def test_info_line_is_consumed_before_state(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            bridge = Path(tmp)/'fake-info-bridge'
            info = dict(packets=[1]*7, age_s=[.002]*7, driver_temperature_c=[25]*7,
                        winding_temperature_c=[26]*7, voltage_v=[48]*7)
            bridge.write_text('#!/usr/bin/env python3\nimport sys\n'
                + 'print(' + repr('INFO '+json.dumps(info)) + ', flush=True)\n'
                + 'print("STATE " + " ".join(["0"]*21+["1"]*7+["0"]*7), flush=True)\n'
                + 'for line in sys.stdin:\n if line.strip()=="STOP": break\n')
            bridge.chmod(0o755)
            bus = Bus(bridge, '/unused', '/unused')
            try:
                self.assertEqual(bus.info, info)
                check_health(bus)
            finally:
                bus.stop()


if __name__ == '__main__':
    unittest.main(verbosity=2)
