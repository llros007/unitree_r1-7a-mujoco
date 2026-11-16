"""Record a hand-guided SO-101 motion, then play it back.

Torque stays off while recording, so move the leader by hand. Playback turns
torque on, moves to the start pose, and follows the recorded timing.

    .venv-leader\\Scripts\\python.exe teach_leader.py --port COM3
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

from keyboard_leader import (
    MOTORS, draw, motor_limits, present_positions, read_keys,
)
from calibrate_leader import DEFAULT_ID, choose_port, leader_classes


DEFAULT_PATH = Path('outputs/leader_motion.json')
PERIOD = 0.02


def empty_motion():
    return {'motors': list(MOTORS), 'samples': []}


def save_motion(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    motion = {'motors': list(MOTORS), 'samples': [{'t': t, 'q': q} for t, q in samples]}
    path.write_text(json.dumps(motion, indent=2), encoding='utf-8')
    return path


def load_motion(path):
    motion = json.loads(path.read_text(encoding='utf-8'))
    if motion.get('motors') != list(MOTORS):
        raise ValueError('动作文件不是这台 SO-101 主臂的六关节记录')
    samples = []
    for row in motion['samples']:
        q = [float(value) for value in row['q']]
        if len(q) != len(MOTORS) or not all(math.isfinite(value) for value in q):
            raise ValueError('动作文件里有无效关节角')
        samples.append((float(row['t']), q))
    if len(samples) < 2 or samples[-1][0] <= samples[0][0]:
        raise ValueError('动作太短，至少要记录两个时刻')
    samples.sort(key=lambda item: item[0])
    origin = samples[0][0]
    return [(t - origin, q) for t, q in samples]


def pose(values):
    return {name: values[index] for index, name in enumerate(MOTORS)}


def clamp_goal(goals, measured, limits, lead):
    limited = {}
    for name in MOTORS:
        low, high = limits[name]
        target = min(high, max(low, goals[name]))
        target = min(measured[name] + lead, max(measured[name] - lead, target))
        limited[name] = target
    return limited


def send(leader, goals):
    leader.send_feedback({f'{name}.pos': goals[name] for name in MOTORS})


def arrived(measured, target, tolerance):
    return all(abs(measured[name] - target[name]) <= tolerance for name in MOTORS)


def screen(mode, measured, samples, path, message):
    rows = [f'SO-101 示教回放    {path}', '']
    for name, label in (
        ('shoulder_pan', '肩旋转'), ('shoulder_lift', '肩抬升'), ('elbow_flex', '肘弯曲'),
        ('wrist_flex', '腕弯曲'), ('wrist_roll', '腕旋转'), ('gripper', '夹爪'),
    ):
        unit = '%' if name == 'gripper' else '°'
        rows.append(f'{label:<4} {name:<14} {measured[name]:+7.1f}{unit}')
    duration = samples[-1][0] if samples else 0.
    rows.extend([
        '',
        f'状态 {mode}    已记录 {len(samples)} 点 / {duration:.1f} 秒',
        message,
        '',
        'R 开始或结束记录。记录时力矩关闭，用手带动主臂。',
        'P 回放一次。回放时力矩打开，主臂会自己动。',
        'L 循环回放    Space 停止回放并卸力',
        'Q 或 Esc 退出并卸力',
    ])
    return rows


def approach(leader, limits, target, speed, lead, redraw):
    tolerance = 1.5
    while True:
        for key in read_keys():
            if key in ('q', 'Q', '\x1b', ' '):
                return False
        measured = present_positions(leader)
        redraw(measured, '正在移到动作起点')
        if arrived(measured, target, tolerance):
            return True
        step = {}
        for name in MOTORS:
            delta = target[name] - measured[name]
            step[name] = measured[name] + max(-speed * PERIOD, min(speed * PERIOD, delta))
        send(leader, clamp_goal(step, measured, limits, lead))
        time.sleep(PERIOD)


def play(leader, limits, samples, speed, rate, lead, repeat, status):
    def redraw(measured, message):
        status(message)
        draw(screen('回放', measured, samples, status.path, status.message))

    leader.enable_torque()
    read_keys()
    try:
        target = pose(samples[0][1])
        if not approach(leader, limits, target, speed, lead, redraw):
            return False
        loops = 0
        while True:
            started = time.monotonic()
            index = 0
            while index < len(samples) - 1:
                for key in read_keys():
                    if key in ('q', 'Q', '\x1b', ' '):
                        return False
                elapsed = (time.monotonic() - started) * rate
                while index < len(samples) - 1 and samples[index + 1][0] <= elapsed:
                    index += 1
                measured = present_positions(leader)
                desired = pose(samples[index][1])
                paced = {}
                for name in MOTORS:
                    delta = desired[name] - measured[name]
                    paced[name] = measured[name] + max(-speed * PERIOD, min(speed * PERIOD, delta))
                send(leader, clamp_goal(paced, measured, limits, lead))
                redraw(measured, f'回放 {loops + 1}    {min(elapsed, samples[-1][0]):.1f}/{samples[-1][0]:.1f} 秒')
                time.sleep(PERIOD)
            loops += 1
            if loops >= repeat:
                return True
    finally:
        leader.disable_torque()


class Status:
    def __init__(self, path):
        self.path = path
        self.message = '力矩关闭。按 R 开始记录。'

    def __call__(self, message):
        self.message = message


def session(leader, path, speed, rate, lead, repeat):
    limits = motor_limits(leader)
    leader.configure()
    samples = load_motion(path) if path.is_file() else []
    recording = False
    started = 0.
    status = Status(path)
    if samples:
        status('已载入动作。按 P 回放，按 R 重新记录。')
    sys.stdout.write('\x1b[2J')
    while True:
        stop_play = False
        for key in read_keys():
            if key in ('q', 'Q', '\x1b'):
                return
            if key in ('r', 'R') and not recording:
                recording = True
                samples = []
                started = time.monotonic()
                status('正在记录，用手带动主臂。再按 R 结束。')
            elif key in ('r', 'R') and recording:
                recording = False
                if len(samples) < 2:
                    samples = []
                    status('记录太短，没有保存。')
                else:
                    save_motion(path, samples)
                    status(f'已保存 {samples[-1][0]:.1f} 秒。按 P 回放。')
            elif key in ('p', 'P', 'l', 'L') and not recording:
                if len(samples) < 2:
                    status('还没有动作。先按 R 记录。')
                else:
                    times = repeat if key in ('l', 'L') else 1
                    finished = play(leader, limits, samples, speed, rate, lead, times, status)
                    leader.configure()
                    status('回放完成，力矩已关闭。' if finished else '回放已停止，力矩已关闭。')
                    stop_play = True
        measured = present_positions(leader)
        if recording:
            samples.append((time.monotonic() - started, [measured[name] for name in MOTORS]))
            if samples[-1][0] >= 120.:
                recording = False
                save_motion(path, samples)
                status('已记录 120 秒并保存。按 P 回放。')
        if not stop_play:
            draw(screen('记录中' if recording else '等待', measured, samples, path, status.message))
        time.sleep(PERIOD)


def main(argv=None):
    parser = argparse.ArgumentParser(description='用手示教 SO-101 主臂动作，再按记录回放')
    parser.add_argument('--port', help='主臂串口，例如 COM3；只连着一个串口时可以省略')
    parser.add_argument('--id', default=DEFAULT_ID, help=f'标定编号，默认 {DEFAULT_ID}')
    parser.add_argument('--motion', type=Path, default=DEFAULT_PATH, help='动作文件路径')
    parser.add_argument('--speed', type=float, default=20., help='回放时单个关节的最大速度，度/秒或百分比/秒')
    parser.add_argument('--rate', type=float, default=1., help='相对记录速度的倍率，1 为原速')
    parser.add_argument('--lead', type=float, default=8., help='目标最多领先反馈的角度或百分比')
    parser.add_argument('--repeat', type=int, default=0, help='L 循环次数；0 表示一直循环，直到按 Space')
    args = parser.parse_args(argv)
    if not 0 < args.speed <= 60 or not 0 < args.rate <= 3 or not 0 < args.lead <= 30 or args.repeat < 0:
        parser.error('--speed 范围为 (0, 60]，--rate 范围为 (0, 3]，--lead 范围为 (0, 30]，--repeat 不能为负')
    loops = 10 ** 9 if args.repeat == 0 else args.repeat

    SO101Leader, SO101LeaderConfig = leader_classes()
    port = choose_port(args.port)
    leader = SO101Leader(SO101LeaderConfig(port=port, id=args.id))
    if not Path(leader.calibration_fpath).is_file():
        raise SystemExit(f'没有标定文件：{leader.calibration_fpath}')
    leader.connect(calibrate=False)
    try:
        if not leader.is_calibrated:
            raise SystemExit('电机里的标定和文件不一致。请先运行 calibrate_leader.py')
        session(leader, args.motion, args.speed, args.rate, args.lead, loops)
    finally:
        sys.stdout.write('\x1b[2J\x1b[H')
        sys.stdout.flush()
        if leader.is_connected:
            leader.disconnect()
        print('已退出，力矩已关闭。', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
