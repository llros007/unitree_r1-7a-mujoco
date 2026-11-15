"""Shared R-to-zero trajectory regressions. No hardware is opened."""
import unittest
import threading
import time
import io
from unittest.mock import patch

import glfw
import numpy as np

from environment import ArmEnvironment
from real_keyboard import Session, handle_key, reanchor, show_measured, terminal_exit, RealKeyboardViewer
from verify_real_keyboard import FakeBus, args


class ZeroReturnChecks(unittest.TestCase):
    def setUp(self):
        self.env = ArmEnvironment(initial_pose='zero')
        q = np.zeros(7); q[3] = np.deg2rad(20)
        show_measured(self.env, q, np.zeros(7))
        reanchor(self.env)

    def test_complete_return_has_bounded_speed_acceleration_and_no_teleport(self):
        for mode, hardware in [('sim', False), ('sync', True), ('sim', True)]:
            with self.subTest(mode=mode, hardware=hardware):
                self.setUp()
                e = self.env
                bus = FakeBus(e.data.qpos[e.qids]) if hardware else None
                session = Session(e, args(mode, hardware))
                initial = e.data.qpos.copy()
                handle_key(session.control, bus, glfw.KEY_R, glfw.PRESS, 0)
                np.testing.assert_array_equal(e.data.qpos, initial)
                points = [e.q_command.copy()]
                speeds = [np.zeros(7)]
                for _ in range(1500):
                    # Do not trigger the separate input-window watchdog.
                    import time
                    session.input_at = time.monotonic()
                    session.tick(bus)
                    command = e.motors.host_command if bus is None else bus.sent[-1]
                    points.append(command[0].copy()); speeds.append(command[1].copy())
                    if e.zero_return is None:
                        break
                self.assertIsNone(e.zero_return)
                np.testing.assert_array_equal(e.q_command, np.zeros(7))
                actual = e.data.qpos[e.qids] if bus is None else bus.q
                self.assertLess(np.max(np.abs(actual)), np.deg2rad(1))
                self.assertLessEqual(np.max(np.abs(np.diff(points, axis=0)))/.01,
                                     np.deg2rad(5)+1e-6)
                self.assertLessEqual(np.max(np.abs(speeds)), np.deg2rad(5)+1e-6)
                self.assertLessEqual(np.max(np.abs(np.diff(speeds, axis=0)))/.01,
                                     np.deg2rad(10)+1e-6)

    def test_pause_and_tab_cancel_without_replaying_home(self):
        for key in (glfw.KEY_P, glfw.KEY_TAB):
            self.setUp()
            session = Session(self.env, args())
            handle_key(session.control, None, glfw.KEY_R, glfw.PRESS, 0)
            for _ in range(30):
                session.tick(None)
            before = self.env.data.qpos.copy()
            handle_key(session.control, None, key, glfw.PRESS, 0)
            self.assertIsNone(self.env.zero_return)
            np.testing.assert_array_equal(self.env.data.qpos, before)
            if key == glfw.KEY_P:
                session.tick(None)
                np.testing.assert_array_equal(self.env.motors.host_command[1], np.zeros(7))
                handle_key(session.control, None, key, glfw.PRESS, 0)
                self.assertIsNone(self.env.zero_return)

    def test_blocked_path_refuses_home(self):
        before = self.env.data.qpos.copy()
        with patch.object(self.env, 'tool_motion_collision', return_value=('tool', 'arm', -.01)):
            message = self.env.start_zero_return()
        self.assertTrue(message.startswith('Blocked'))
        self.assertIsNone(self.env.zero_return)
        np.testing.assert_array_equal(self.env.data.qpos, before)

    def test_paused_r_does_not_move_or_enable_home(self):
        self.env.paused = True
        before = self.env.data.qpos.copy()
        self.assertTrue(self.env.start_zero_return().startswith('Paused'))
        self.assertIsNone(self.env.zero_return)
        np.testing.assert_array_equal(self.env.data.qpos, before)

    def test_repeated_r_does_not_restart_trajectory(self):
        session = Session(self.env, args())
        handle_key(session.control, None, glfw.KEY_R, glfw.PRESS, 0)
        session.tick(None)
        plan = self.env.zero_return
        elapsed = plan['elapsed']
        handle_key(session.control, None, glfw.KEY_R, glfw.PRESS, 0)
        self.assertIs(self.env.zero_return, plan)
        self.assertEqual(plan['elapsed'], elapsed)

    def test_arrival_uses_hardware_measurement_not_sim_prediction(self):
        self.env.start_zero_return()
        self.env.zero_return['index'] = len(self.env.zero_return['segments'])
        self.env.data.qpos[self.env.qids] = 0
        self.env.zero_return_feedback = (np.full(7, .1), np.zeros(7))
        for _ in range(30):
            self.env.advance_zero_return(.01)
        self.assertIsNotNone(self.env.zero_return)
        self.assertEqual(self.env.zero_return['stable'], 0)

    def test_background_planning_keeps_sending_and_can_be_cancelled(self):
        bus = FakeBus(self.env.data.qpos[self.env.qids])
        session = Session(self.env, args('sync', True))
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        original = ArmEnvironment.plan_zero_return

        def slow_plan(planner, *values):
            entered.set()
            release.wait(3)
            try:
                return original(planner, *values)
            finally:
                finished.set()

        with patch.object(ArmEnvironment, 'plan_zero_return', slow_plan):
            try:
                session.events.put((glfw.KEY_R, glfw.PRESS, 0))
                session.tick(bus)
                self.assertTrue(entered.wait(1))
                before = len(bus.sent)
                for _ in range(25):
                    session.input_at = time.monotonic()
                    session.tick(bus)
                self.assertEqual(len(bus.sent) - before, 25)
                self.assertIsNone(self.env.zero_return)
                session.events.put((glfw.KEY_P, glfw.PRESS, 0))
                session.tick(bus)
                self.assertTrue(self.env.paused)
            finally:
                release.set()
                self.assertTrue(finished.wait(2))
        # Wait for the worker's queue publication after computation.
        until = time.monotonic() + 1
        while session.home_results.empty() and time.monotonic() < until:
            time.sleep(.001)
        session.tick(bus)
        self.assertFalse(session.home_pending)
        self.assertIsNone(self.env.zero_return)

    def test_background_plan_is_accepted_and_returns_to_zero(self):
        bus = FakeBus(self.env.data.qpos[self.env.qids])
        session = Session(self.env, args('sync', True))
        session.events.put((glfw.KEY_R, glfw.PRESS, 0))
        session.tick(bus)
        until = time.monotonic() + 2
        while session.home_results.empty() and time.monotonic() < until:
            time.sleep(.001)
        for _ in range(1500):
            session.input_at = time.monotonic()
            session.tick(bus)
            if not session.home_pending and self.env.zero_return is None:
                break
        self.assertFalse(session.home_pending)
        self.assertIsNone(self.env.zero_return)
        np.testing.assert_allclose(bus.q, 0, atol=np.deg2rad(1))

    def test_terminal_and_window_exit_during_planning(self):
        session = Session(self.env, args())
        session.home_pending = True
        terminal_exit(session, io.StringIO('q\n'))
        self.assertTrue(session.stop_event.is_set())
        session.stop_event.clear()
        viewer = RealKeyboardViewer.__new__(RealKeyboardViewer)
        viewer.session = session
        with patch('real_keyboard.glfw.set_window_should_close') as close:
            viewer.on_key('window', glfw.KEY_ESCAPE, 0, glfw.PRESS, 0)
        close.assert_called_once_with('window', True)
        self.assertTrue(session.stop_event.is_set())

    def test_plan_is_rejected_when_measured_pose_changes(self):
        bus = FakeBus(self.env.data.qpos[self.env.qids])
        session = Session(self.env, args('sync', True))
        plan = self.env.plan_zero_return(bus.q)
        session.home_generation = 1
        session.home_pending = True
        session.home_results.put((1, plan, 5., 10.))
        bus.q[3] += np.deg2rad(1)
        session.accept_home_result(bus)
        self.assertFalse(session.home_pending)
        self.assertIsNone(self.env.zero_return)
        self.assertIn('pose moved', session.control.joint_status)


if __name__ == '__main__':
    unittest.main(verbosity=2)
