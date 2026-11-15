"""Offline bounded-integral and scheduling checks; no serial hardware."""
import unittest
import numpy as np
from position_bias import PositionBias
from auto_joint_test import command_wait_seconds


class BiasChecks(unittest.TestCase):
    def test_bounds_freeze_and_unwind(self):
        c = PositionBias(np.ones(7)*20, np.ones(7)*2, np.ones(7)*.3)
        for _ in range(1000):
            before = c.bias.copy()
            c.update(np.ones(7), .02)
            self.assertLessEqual(np.max(np.abs(c.bias-before)), .006000001)
        np.testing.assert_allclose(c.bias, 2)
        c.update(-np.ones(7), .02, integrate=False)
        np.testing.assert_allclose(c.bias, 2)
        c.update(-np.ones(7), .02)
        self.assertTrue(np.all(c.bias < 2))
        before = c.bias.copy()
        c.update(np.ones(7), .02, integrate=False, unwind=True)
        np.testing.assert_allclose(c.bias, before-.006)

    def test_constant_load_error_decreases_without_gain_increase(self):
        c = PositionBias(np.ones(7)*20, np.ones(7)*2, np.ones(7)*.3)
        for _ in range(600):
            error = (1.7-c.bias)/60
            c.update(error, .02)
        self.assertLess(np.max(np.rad2deg((1.7-c.bias)/60)), .1)

    def test_invalid_or_late_feedback_is_rejected(self):
        c = PositionBias(np.ones(7), np.ones(7), np.ones(7)*.1)
        for dt in (0, -.1, .101, float('nan')):
            with self.assertRaises(ValueError):
                c.update(np.ones(7), dt)
        with self.assertRaises(ValueError):
            c.update(np.full(7, np.nan), .02)

    def test_no_added_period_after_overrun(self):
        self.assertAlmostEqual(command_wait_seconds(1,1.004,.01,True), .006)
        self.assertEqual(command_wait_seconds(1,1.015,.01,True), 0)
        with self.assertRaises(TimeoutError):
            command_wait_seconds(1,1.111,.01,True)

    def test_near_limit_bias_unwinds_without_affecting_other_joint(self):
        from motor_control import ArmMotorController
        c = ArmMotorController.__new__(ArmMotorController)
        c.position_bias = PositionBias(np.ones(7)*20, np.ones(7), np.ones(7)*.3)
        c.joint_limits = np.tile([0., np.pi], (7,1))
        measured = np.full(7, .5); measured[1] = np.deg2rad(.2)
        c.position_bias.bias[[1,3]] = -.5
        command = (measured.copy(), np.zeros(7), np.zeros(7), np.ones(7)*60, np.ones(7))
        c.compensate_hold(command, measured, .02)
        self.assertAlmostEqual(c.position_bias.bias[1], -.494)
        self.assertAlmostEqual(c.position_bias.bias[3], -.5)
        command[1][3] = .01
        c.compensate_hold(command, measured, .02, integrate=False, unwind=True)
        self.assertAlmostEqual(c.position_bias.bias[3], -.494)

    def test_mujoco_disturbance_hold(self):
        from environment import ArmEnvironment
        import mujoco
        env = ArmEnvironment(initial_pose='clearance')
        q = env.data.qpos[env.qids].copy(); q[1] = np.deg2rad(5)
        env.data.qpos[env.qids] = q
        env.data.qvel[:] = 0
        mujoco.mj_forward(env.model, env.data)
        ki, limit, slew = np.zeros((3,7))
        ki[[1,3]], limit[[1,3]], slew[[1,3]] = [20,15], [2,1.2], [.3,.2]
        bias = PositionBias(ki, limit, slew)
        env.data.qfrc_applied[env.dofs[1]] = -1.7
        env.data.qfrc_applied[env.dofs[3]] = .9
        for _ in range(650):
            correction = bias.update(q-env.data.qpos[env.qids], .02)
            env.set_joint_command(q, 0, env.gravity_feedforward(q)+correction)
            env.step(.02)
        self.assertLess(np.max(np.abs(np.rad2deg((q-env.data.qpos[env.qids])[[1,3]]))), .2)


if __name__ == '__main__':
    unittest.main()
