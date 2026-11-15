"""Offline checks for the automatic benchmark; never opens a serial port."""
import json
import csv
import warnings
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from auto_joint_test import SimulationBus, ShadowSimulation, metrics, parse_args, plan, reference, run


class AutoTestChecks(unittest.TestCase):
    def test_startup_only_and_empty_trace_are_not_precision_failures(self):
        from joint_test_report import analyze
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            report = dict(source='hardware_feedback', status='aborted', error='FOC not ready', segments=[],
                          arguments=dict(joints=[3], move_seconds=1., hold_seconds=1.,
                                         tolerance_deg=.5, velocity_tolerance_deg_s=.5),
                          motor_config=dict(command_period_s=.01))
            (output/'report.json').write_text(json.dumps(report))
            fields = ['segment', 'receive_time_s'] + [f'{field}_J{j}'
                for field in ('q_cmd_rad', 'q_feedback_rad', 'dq_feedback_rad_s', 'q_sim_rad') for j in range(1,8)]
            for count in (0, 1):
                with (output/'trace.csv').open('w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
                    if count:
                        writer.writerow({k: (-1 if k == 'segment' else 0) for k in fields})
                with warnings.catch_warnings():
                    warnings.simplefilter('error')
                    result = analyze(output)
                self.assertEqual(result['assessment'], 'not_tested')
                self.assertIsNone(result['tested_joints_passed'])
                self.assertIsNone(result['real_minus_sim'])
                self.assertIn('NOT TESTED', (output/'comparison.html').read_text())

    def test_quintic_endpoints_and_speed(self):
        start, end = np.zeros(7), np.ones(7) * np.deg2rad(1)
        for t, expected in ((0, start), (2, end), (3, end)):
            q, dq = reference(start, end, t, 2)
            np.testing.assert_allclose(q, expected)
            np.testing.assert_allclose(dq, 0)
        self.assertLess(np.max(np.rad2deg(reference(start, end, 1, 2)[1])), 5)

    def test_no_implicit_hardware_and_speed_rejection(self):
        self.assertFalse(parse_args([]).hardware)
        with self.assertRaises(SystemExit):
            parse_args(['--amplitude-deg', '5', '--move-seconds', '.1'])

    def test_metrics_distinguish_bias_and_settling(self):
        args = parse_args(['--move-seconds', '1', '--hold-seconds', '1'])
        q = np.zeros(7)
        samples = [(t, q, q + np.deg2rad(.1), q) for t in np.linspace(0, 2, 201)]
        result = metrics(samples, args)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['settling_seconds'], 0)
        np.testing.assert_allclose(result['steady_bias_deg'], .1)
        samples[-1] = (2., q, q + np.deg2rad(1), q)
        self.assertFalse(metrics(samples, args)['passed'])

    def test_plan_rejects_limit_and_collision(self):
        args = parse_args([])
        env = SimpleNamespace(limits=np.tile([-1., 1.], (7, 1)), tool_motion_collision=lambda a, b: None)
        start = np.zeros(7); start[3] = 1.
        with self.assertRaises(ValueError):
            plan(env, start, args)
        env.tool_motion_collision = lambda a, b: ('tool', 'arm', 0.)
        with self.assertRaises(ValueError):
            plan(env, np.zeros(7), args)

    def test_auto_direction_preserves_amplitude_and_rejects_no_path(self):
        args = parse_args(['--joints', '2', '--cycles', '1', '--amplitude-deg', '5',
                           '--move-seconds', '3', '--direction', 'auto'])
        env = SimpleNamespace(limits=np.tile([0., 1.], (7, 1)), tool_motion_collision=lambda a,b: None)
        segments = plan(env, np.zeros(7), args)
        self.assertEqual(len(segments), 2)
        self.assertEqual({s['sign'] for s in segments}, {1})
        self.assertAlmostEqual(np.rad2deg(segments[0]['target'][1]), 5.)
        np.testing.assert_allclose(segments[-1]['target'], 0.)
        env.tool_motion_collision = lambda a,b: ('tool', 'arm', 0.)
        with self.assertRaisesRegex(ValueError, 'no checked direction'):
            plan(env, np.zeros(7), args)

    def test_staged_large_angle_is_explicit_and_bounded(self):
        args=parse_args(['--joints','2','4','--amplitude-deg','60','--large-angle','--direction','positive',
                         '--stages-deg','15','30','60','--move-seconds','40','--cycles','1'])
        env=SimpleNamespace(limits=np.tile([0.,np.pi],(7,1)),tool_motion_collision=lambda a,b:None)
        stages=plan(env,np.zeros(7),args)
        self.assertEqual(len(stages),8)
        self.assertEqual([s['relative_deg'] for s in stages[:4]],[15,30,60,0])
        for s in stages:
            delta=np.max(np.abs(np.rad2deg(s['target']-s['start'])))
            self.assertLessEqual(1.875*delta/s['move_seconds'],5)
            self.assertLessEqual(5.774*delta/s['move_seconds']**2,10)
        np.testing.assert_allclose(stages[-1]['target'],0)

    def test_selected_joint_is_separate_from_holding_and_raw_velocity(self):
        args = parse_args(['--move-seconds', '1', '--hold-seconds', '1'])
        q = np.zeros(7)
        measured = q.copy(); measured[3] = np.deg2rad(.8)
        samples = [(t, q, measured, q) for t in np.linspace(0, 2, 201)]
        result = metrics(samples, args, 3)
        self.assertTrue(result['passed'])
        self.assertFalse(result['holding_joints_passed'])
        self.assertFalse(result['position_passed'][3])
        samples[-1] = (2., q, measured, np.ones(7) * np.deg2rad(1))
        result = metrics(samples, args, 3)
        self.assertFalse(result['passed'])
        self.assertTrue(result['position_passed'][2])
        self.assertFalse(result['velocity_passed'][2])

    def test_shadow_preserves_elapsed_time_and_command_hold(self):
        class Plant:
            def __init__(self):
                self.data = SimpleNamespace(qpos=np.zeros(7), qvel=np.zeros(7))
                self.qids = self.dofs = np.arange(7)
                self.model = SimpleNamespace(opt=SimpleNamespace(timestep=.002))
                self.calls = []
            def set_joint_command(self, *command):
                self.calls.append(('command', float(np.asarray(command[0])[0])))
            def step(self, duration):
                self.calls.append(('step', round(duration, 6)))
        plant = Plant()
        shadow = ShadowSimulation(plant, np.ones(7), np.zeros(7))
        shadow.sample((np.ones(7)*2, 0, 0), 10., 10.003)
        shadow.sample((np.ones(7)*3, 0, 0), 10.011, 10.014)
        self.assertEqual(shadow.steps, 7)
        self.assertEqual(plant.calls, [('command', 1.), ('command', 2.), ('step', .002),
                                       ('step', .008), ('command', 3.), ('step', .004)])

    def test_stop_request_prevents_commands(self):
        import threading
        event = threading.Event(); event.set()
        class StopBus(SimulationBus):
            sent = 0
            def send(self, *command):
                StopBus.sent += 1
                super().send(*command)
        with tempfile.TemporaryDirectory() as temp:
            args = parse_args(['--output', str(Path(temp)/'cancel')])
            with patch('auto_joint_test.SimulationBus', StopBus):
                self.assertEqual(run(args, stop_event=event), 1)
            self.assertEqual(StopBus.sent, 0)

    def test_mock_hardware_uses_independent_timestamped_simulation(self):
        class Hardware:
            def __init__(self):
                self.q = np.deg2rad([0., 0., 0., 45., 0., 0., 0.])
                self.dq = self.tau = np.zeros(7)
                self.mode = np.ones(7, dtype=int)
                self.err = np.zeros(7, dtype=int)
                self.stopped = False
            def send(self, q, dq, tau, kp, kd):
                self.q, self.dq = q.copy(), dq.copy()
            def stop(self):
                self.stopped = True
        bus = Hardware()
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'mock_hardware'
            args = parse_args(['--hardware', '--sim-compare', '--cycles', '1', '--move-seconds', '1',
                               '--hold-seconds', '.3', '--output', str(output)])
            with patch('auto_joint_test.Bus', return_value=bus):
                self.assertEqual(run(args), 0)
            self.assertTrue(bus.stopped)
            report = json.loads((output/'analysis.json').read_text())
            self.assertIn('real_minus_sim', report)
            self.assertGreater(max(report['real_minus_sim']['max_abs_deg']), .0001)
            self.assertTrue((output/'comparison.html').is_file())

    def test_fault_stops_and_preserves_report(self):
        class FaultBus(SimulationBus):
            stopped = False

            def send(self, *command):
                self.err[0] = 8

            def stop(self):
                FaultBus.stopped = True

        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / 'fault'
            args = parse_args(['--output', str(output)])
            with patch('auto_joint_test.SimulationBus', FaultBus):
                self.assertEqual(run(args), 1)
            report = json.loads((output / 'report.json').read_text())
            self.assertEqual(report['status'], 'aborted')
            self.assertIn('FOC', report['error'])
            self.assertTrue(FaultBus.stopped)
            self.assertTrue((output / 'trace.csv').exists())


if __name__ == '__main__':
    unittest.main()
