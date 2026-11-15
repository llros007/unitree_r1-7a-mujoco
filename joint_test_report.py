"""Postprocess raw joint traces without hardware access; self-contained HTML/SVG plots."""
import csv
import html
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def analyze(directory):
    from auto_joint_test import metrics
    directory = Path(directory)
    report = json.loads((directory / 'report.json').read_text())
    with (directory / 'trace.csv').open() as stream:
        reader = csv.reader(stream)
        columns = next(reader, [])
        rows = [dict(zip(columns, row)) for row in reader if len(row) == len(columns)]
    if not rows:
        result = dict(source=report['source'], assessment='not_tested', segments=[],
                      error=report.get('error'), sample_count=0, real_minus_sim=None,
                      tested_joints_passed=None, holding_joints_passed=None)
        (directory / 'analysis.json').write_text(json.dumps(result, indent=2) + '\n')
        (directory / 'comparison.html').write_text('<!doctype html><meta charset="utf-8"><h1>NOT TESTED</h1><p>' +
            html.escape(report.get('error', 'No samples recorded.')) + '</p>')
        return result
    args = SimpleNamespace(**report['arguments'])
    def values(field):
        return np.array([[float(r[f'{field}_J{j}']) for j in range(1, 8)] for r in rows])

    target, measured, velocity = values('q_cmd_rad'), values('q_feedback_rad'), values('dq_feedback_rad_s')
    has_sim = 'q_sim_rad_J1' in rows[0]
    sim = values('q_sim_rad') if has_sim else None
    tcp = {}
    if 'tcp_target_x_m' in rows[0]:
        tcp = {kind: np.array([[float(r[f'tcp_{kind}_{axis}_m']) * 1000 for axis in 'xyz'] for r in rows])
               for kind in ('target', 'command', 'feedback', 'sim')}
    result = dict(source=report['source'], segments=[],
                  note='Raw velocity threshold kept unchanged. FK/model comparison is not external TCP accuracy.')
    segment_ids = np.array([int(r['segment']) for r in rows])
    if len(rows) > 1 and 'send_time_s' in rows[0]:
        sent = np.array([float(r['send_time_s']) for r in rows])
        intervals = np.diff(sent)
        result['acquisition'] = dict(
            configured_hz=1 / report['motor_config']['command_period_s'],
            achieved_host_hz=float((len(sent)-1)/(sent[-1]-sent[0])) if sent[-1] > sent[0] else None,
            interval_median_ms=float(np.median(intervals)*1000),
            interval_p95_ms=float(np.percentile(intervals, 95)*1000),
            interval_max_ms=float(np.max(intervals)*1000),
            note='Host send timing, not motor sensor acquisition frequency. Offline simulation wall time is computation time.')
    for index, segment in enumerate(report['segments']):
        ids = np.flatnonzero(segment_ids == index)
        if not len(ids):
            continue
        samples = [(float(rows[i]['reference_time_s']), target[i], measured[i], velocity[i]) for i in ids]
        entry = {k: segment[k] for k in ('joint', 'axis', 'cycle', 'sign', 'kind') if k in segment}
        for key in ('relative_deg','move_seconds'):
            if key in segment:entry[key]=segment[key]
        segment_args = SimpleNamespace(**{**vars(args), 'move_seconds': segment.get('move_seconds', args.move_seconds)})
        entry.update(metrics(samples, segment_args, segment.get('joint')))
        if tcp:
            tail = ids[np.array([rows[i]['phase'] == 'hold' and
                                float(rows[i]['reference_time_s']) >= segment_args.move_seconds + args.hold_seconds - .5
                                for i in ids])]
            entry['tcp_mm'] = {}
            for first, second in [('command', 'target'), ('feedback', 'target'),
                                  ('sim', 'target'), ('feedback', 'sim')]:
                delta = tcp[first] - tcp[second]
                error = np.linalg.norm(delta[ids], axis=1)
                entry['tcp_mm'][f'{first}_minus_{second}'] = dict(
                    rmse=float(np.sqrt(np.mean(error**2))), max=float(np.max(error)),
                    steady_bias_norm=float(np.linalg.norm(delta[tail].mean(axis=0))) if len(tail) else None)
            entry['steady_orientation_feedback_minus_sim_deg'] = (
                float(np.mean([float(rows[i]['orientation_feedback_minus_sim_deg']) for i in tail]))
                if len(tail) else None)
        result['segments'].append(entry)
    result['assessment'] = 'completed_segments' if result['segments'] else 'not_tested'
    result['error'] = report.get('error')
    result['sample_count'] = len(rows)
    result['tested_joints_passed'] = all(s['passed'] for s in result['segments']) if result['segments'] else None
    result['holding_joints_passed'] = all(s['holding_joints_passed'] for s in result['segments']) if result['segments'] else None
    result['all_positions_passed'] = all(all(s['position_passed']) for s in result['segments']) if result['segments'] else None
    result['all_raw_velocities_passed'] = all(all(s['velocity_passed']) for s in result['segments']) if result['segments'] else None
    result['directional_returns'] = {}
    for joint in ([] if tcp else args.joints):
        groups = {}
        for sign in (1, -1):
            ends = [s['endpoint_mean_deg'][joint-1] for s in result['segments']
                    if s['joint'] == joint and s['kind'] == 'return' and s['sign'] == sign]
            if ends:
                groups[str(sign)] = dict(count=len(ends), mean_deg=float(np.mean(ends)), std_deg=float(np.std(ends)))
        if len(groups) == 2:
            groups['direction_difference_deg'] = groups['1']['mean_deg'] - groups['-1']['mean_deg']
        result['directional_returns'][f'J{joint}'] = groups
    mask = segment_ids >= 0
    result['real_minus_sim'] = None
    if has_sim and np.any(mask):
        delta = np.rad2deg(measured[mask] - sim[mask])
        result['real_minus_sim'] = dict(rmse_deg=np.sqrt(np.mean(delta**2, axis=0)).tolist(),
                                       max_abs_deg=np.max(np.abs(delta), axis=0).tolist(),
                                       mean_deg=np.mean(delta, axis=0).tolist())
    (directory / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    def verdict(value):
        return 'NOT TESTED' if value is None else ('PASS' if value else 'FAIL')

    heading = 'Cartesian IK test comparison' if tcp else 'Joint test comparison'
    parts = [f'<!doctype html><meta charset="utf-8"><title>{heading}</title>',
             '<style>body{font:15px system-ui;background:#101721;color:#e4edf7;max-width:1150px;margin:30px auto;padding:16px}svg{width:100%;background:#172231;border-radius:8px}table{border-collapse:collapse;width:100%;margin:20px 0}td,th{border:1px solid #455268;padding:8px}h2{margin-top:30px}</style>',
             f'<h1>{heading}</h1>',
             f'<p>Source: {html.escape(report["source"])} | Run: {html.escape(report["status"])}</p>',
             '<p>Joint encoder feedback only. Left viewer pose is model FK, not externally measured TCP. '
             'Simulation is an independent model driven by host commands; timestamps are not synchronized motor acquisition times.</p>',
             '<p style="color:#58b8ff">Blue: command <span style="color:#ffb85c">Orange: feedback</span> '
             '<span style="color:#71ddaf">Green: independent simulation</span></p>',
             '<p>' + html.escape(report.get('error', '')) + '</p>',
             ('' if tcp else f'<p>Tested joints: {verdict(result["tested_joints_passed"])} | '
              f'Holding joints: {verdict(result["holding_joints_passed"])}</p>'),
             f'<p>All position conditions: {verdict(result["all_positions_passed"])} | '
             f'All raw velocity conditions: {verdict(result["all_raw_velocities_passed"])}</p>',
             '<table><tr><th>Segment</th><th>Combined test</th><th>Steady bias (tested, deg)</th><th>Position failures</th><th>Raw speed failures</th></tr>']
    for index, s in enumerate(result['segments']):
        failures = lambda field: ', '.join(f'J{j+1}' for j, ok in enumerate(s[field]) if not ok) or 'none'
        stage = f' relative {s["relative_deg"]:g} deg' if 'relative_deg' in s else f' direction {s["sign"]:+}'
        label = s['axis'] if 'axis' in s else f'J{s["joint"]}'
        bias = (max(abs(x) for x in s['steady_bias_deg']) if tcp else
                s['steady_bias_deg'][s['joint']-1])
        parts.append(f'<tr><td>{index+1}: {label} {s["kind"]}{stage}</td>'
                     f'<td>{"PASS" if s["passed"] else "FAIL"}</td>'
                     f'<td>{bias:+.5f}</td><td>{failures("position_passed")}</td>'
                     f'<td>{failures("velocity_passed")}</td></tr>')
    parts.append('</table>')
    if tcp:
        parts.append('<h2>TCP position error in mm</h2><table><tr><th>Segment</th>'
                     '<th>IK command vs target steady</th><th>Feedback FK vs target steady</th>'
                     '<th>Simulation vs target steady</th><th>Feedback FK vs simulation steady</th>'
                     '<th>Feedback FK vs simulation peak</th></tr>')
        for s in result['segments']:
            errors = s['tcp_mm']
            parts.append(f'<tr><td>{s["axis"]}{s["sign"]:+} {s["kind"]}</td>' +
                         ''.join(f'<td>{errors[key]["steady_bias_norm"]:.3f}</td>'
                                 for key in ('command_minus_target', 'feedback_minus_target',
                                             'sim_minus_target', 'feedback_minus_sim')) +
                         f'<td>{errors["feedback_minus_sim"]["max"]:.3f}</td></tr>')
        parts.append('</table>')
    if report['source'] == 'hardware_feedback' and result.get('acquisition'):
        timing = result['acquisition']
        parts.append(f'<p>Configured host rate: {timing["configured_hz"]:.1f} Hz; achieved: '
                     f'{timing["achieved_host_hz"]:.1f} Hz. Send interval median/P95/max: '
                     f'{timing["interval_median_ms"]:.2f} / {timing["interval_p95_ms"]:.2f} / '
                     f'{timing["interval_max_ms"]:.2f} ms. Not motor sensor acquisition frequency.</p>')
    if result['real_minus_sim'] is not None:
        parts.append('<p>Feedback minus simulation RMSE (degrees, J1–J7): ' +
                     ', '.join(f'{x:.4f}' for x in result['real_minus_sim']['rmse_deg']) + '</p>')
    t = np.array([float(r['receive_time_s']) for r in rows])
    if report['source'] == 'simulation':
        # Wall clock represents computation time in offline mode, not physical time.
        t = np.arange(len(rows)) * report['motor_config']['command_period_s']
    t -= t[0]
    x = 65 + 980 * t / max(t[-1], 1e-9)
    # Bound chart size while retaining each bin's extrema, including spikes.
    charts = []
    for j in range(7):
        curves = [np.rad2deg(target[:, j]), np.rad2deg(measured[:, j])]
        if has_sim:
            curves.append(np.rad2deg(sim[:, j]))
        charts.append((f'J{j+1} — joint angle (degrees)', curves))
    if tcp:
        charts = [(f'TCP {axis.upper()} (mm)', [tcp[k][:, i] for k in ('target', 'feedback', 'sim')])
                  for i, axis in enumerate('xyz')] + charts
    for title, curves in charts:
        lo, hi = min(c.min() for c in curves), max(c.max() for c in curves)
        pad = max(.02, (hi-lo)*.1)
        lo, hi = lo-pad, hi+pad
        parts.append(f'<h2>{title}</h2><svg viewBox="0 0 1100 220" role="img" aria-label="{title} over time">')
        for value, yy in ((hi, 25), ((lo+hi)/2, 100), (lo, 175)):
            parts.append(f'<path d="M65 {yy}H1045" stroke="#354355"/><text x="2" y="{yy+5}" fill="#bbc9dc" font-size="12">{value:.3f}</text>')
        for c, color in zip(curves, ('#58b8ff', '#ffb85c', '#71ddaf')):
            bins = np.array_split(np.arange(len(c)), min(800, len(c)))
            ids = sorted({int(i) for b in bins for i in (b[0], b[-1], b[np.argmin(c[b])], b[np.argmax(c[b])])})
            points = ' '.join(f'{x[i]:.2f},{175-150*(c[i]-lo)/(hi-lo):.2f}' for i in ids)
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>')
        parts.append(f'<text x="65" y="205" fill="#bbc9dc">0 s</text><text x="970" y="205" fill="#bbc9dc">{t[-1]:.2f} s</text></svg>')
    (directory / 'comparison.html').write_text('\n'.join(parts))
    print(f'Analysis: {directory.resolve() / "analysis.json"}\nCharts: {directory.resolve() / "comparison.html"}', flush=True)
    return result
