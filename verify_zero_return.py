"""Shared R-to-zero trajectory regressions. No hardware is opened."""
import unittest
from unittest.mock import patch

import glfw
import numpy as np

from environment import ArmEnvironment
from real_keyboard import Session, handle_key, reanchor, show_measured
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
