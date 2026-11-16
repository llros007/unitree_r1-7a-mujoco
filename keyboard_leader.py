"""Keyboard jog for the SO-101 leader from a Windows terminal.

    .venv-leader\\Scripts\\python.exe keyboard_leader.py --port COM3

Torque turns on after the current pose is read, so the arm holds where it is.
Exit and Ctrl+C turn torque off.
"""
import argparse
import msvcrt
import sys
import time
from pathlib import Path

from calibrate_leader import DEFAULT_ID, MOTORS, choose_port, leader_classes


LABELS = {
    'shoulder_pan': '肩旋转',
    'shoulder_lift': '肩抬升',
    'elbow_flex': '肘弯曲',
    'wrist_flex': '腕弯曲',
    'wrist_roll': '腕旋转',
    'gripper': '夹爪',
}


def motor_limits(leader):
    from lerobot.motors import MotorNormMode

    bus = leader.bus
    limits = {}
    for name, motor in bus.motors.items():
        if motor.norm_mode is MotorNormMode.RANGE_0_100:
            limits[name] = (0., 100.)
            continue
        if motor.norm_mode is not MotorNormMode.DEGREES:
            raise RuntimeError(f'{name} 的角度单位不是度')
        cal = bus.calibration[name]
        span = bus.resolution(motor.model) - 1
        mid = (cal.range_min + cal.range_max) / 2
        low = (cal.range_min - mid) * 360 / span
        high = (cal.range_max - mid) * 360 / span
        limits[name] = (min(low, high), max(low, high))
    return limits


def enable_console():
    """Windows ignores cursor-home escapes until this console mode is turned on."""
    import ctypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    handle = kernel.GetStdHandle(-11)
    mode = ctypes.c_uint()
    if kernel.GetConsoleMode(handle, ctypes.byref(mode)):
        kernel.SetConsoleMode(handle, mode.value | 0x0004)


def read_keys():
    keys = []
    while msvcrt.kbhit():
        key = msvcrt.getwch()
        if key in ('\x00', '\xe0'):
            keys.append(key + msvcrt.getwch())
            continue
        if key != '\x1b':
            keys.append(key)
            continue
        time.sleep(0.02)
        sequence = ''
        while msvcrt.kbhit():
            sequence += msvcrt.getwch()
        arrows = {'[D': '\xe0K', '[C': '\xe0M', '[A': '\xe0H', '[B': '\xe0P'}
        if sequence in arrows:
            keys.append(arrows[sequence])
        elif sequence == '':
            keys.append('\x1b')
    return keys


def present_positions(leader):
    action = leader.get_action()
    return {name: float(action[f'{name}.pos']) for name in MOTORS}


def format_screen(selected, goals, measured, limits, speed, fine, torque, message):
    rows = ['SO-101 主臂键盘控制', '']
    for index, name in enumerate(MOTORS):
        mark = '>' if index == selected else ' '
        low, high = limits[name]
        unit = '%' if name == 'gripper' else '°'
        rows.append(
            f'{mark} {index + 1} {LABELS[name]:<4} {name:<14} '
            f'当前 {measured[name]:+7.1f}{unit}  目标 {goals[name]:+7.1f}{unit}  '
            f'[{low:+.0f}, {high:+.0f}]'
        )
    rows.extend([
        '',
        f'速度 {speed:g} {"°" if MOTORS[selected] != "gripper" else "%"}/秒'
        f'{"  微调" if fine else ""}   力矩 {"开" if torque else "关"}',
        message,
        '',
        '1-6 选择关节    ←/→ 或 -/+ 点动，松开即停    F 微调',
        'T 开关力矩    Q 或 Esc 退出并卸力',
    ])
    return rows


def draw(rows):
    enable_console()
    sys.stdout.write('\x1b[H')
    for row in rows:
        sys.stdout.write(row + '\x1b[K\n')
    sys.stdout.write('\x1b[J')
    sys.stdout.flush()


def run(leader, speed, fine_scale, lead):
    limits = motor_limits(leader)
    leader.configure()
    measured = present_positions(leader)
    goals = dict(measured)
    print('已读到当前姿态。2 秒后使能，主臂会保持不动。请扶住。', flush=True)
    time.sleep(2)
    leader.enable_torque()
    torque = True
    leader.send_feedback({f'{name}.pos': goals[name] for name in MOTORS})

    selected = 0
    fine = False
    direction = 0
    direction_until = 0.
    message = '力矩已打开，正在保持当前姿态。'
    sys.stdout.write('\x1b[2J')
    period = 0.02
    previous = time.monotonic()
    while True:
        now = time.monotonic()
        dt = min(now - previous, 0.1)
        previous = now
        for key in read_keys():
            if key in ('q', 'Q', '\x1b'):
                return
            if key in ('t', 'T'):
                if torque:
                    leader.disable_torque()
                    torque = False
                    direction = 0
                    message = '力矩已关闭，可以用手掰动。按 T 重新保持。'
                else:
                    measured = present_positions(leader)
                    goals = dict(measured)
                    leader.enable_torque()
                    torque = True
                    message = '力矩已打开，从当前姿态保持。'
            elif key in ('f', 'F'):
                fine = not fine
            elif key in '123456':
                selected = int(key) - 1
                direction = 0
            elif key in ('-', '_', '\xe0K'):
                direction = -1
                direction_until = now + 0.12
            elif key in ('+', '=', '\xe0M'):
                direction = 1
                direction_until = now + 0.12
        if now > direction_until:
            direction = 0
        name = MOTORS[selected]
        rate = speed * (fine_scale if fine else 1.)
        if name == 'gripper':
            rate = min(rate, 40.)
        if torque and direction:
            goals[name] += direction * rate * dt
            low, high = limits[name]
            goals[name] = min(high, max(low, goals[name]))
            goals[name] = min(measured[name] + lead, max(measured[name] - lead, goals[name]))
        if torque:
            leader.send_feedback({f'{joint}.pos': goals[joint] for joint in MOTORS})
        measured = present_positions(leader)
        draw(format_screen(selected, goals, measured, limits, rate if torque else speed, fine, torque, message))
        time.sleep(max(0., period - (time.monotonic() - now)))


def main(argv=None):
    parser = argparse.ArgumentParser(description='在终端用键盘点动 SO-101 主臂')
    parser.add_argument('--port', help='主臂串口，例如 COM3；只连着一个串口时可以省略')
    parser.add_argument('--id', default=DEFAULT_ID, help=f'标定编号，默认 {DEFAULT_ID}')
    parser.add_argument('--speed', type=float, default=12., help='点动速度，度/秒，夹爪为百分比/秒')
    parser.add_argument('--lead', type=float, default=8., help='目标最多领先反馈的角度或百分比')
    args = parser.parse_args(argv)
    if not 0 < args.speed <= 60 or not 0 < args.lead <= 30:
        parser.error('--speed 范围为 (0, 60]，--lead 范围为 (0, 30]')

    SO101Leader, SO101LeaderConfig = leader_classes()
    port = choose_port(args.port)
    leader = SO101Leader(SO101LeaderConfig(port=port, id=args.id))
    if not Path(leader.calibration_fpath).is_file():
        raise SystemExit(f'没有标定文件：{leader.calibration_fpath}')
    leader.connect(calibrate=False)
    try:
        if not leader.is_calibrated:
            raise SystemExit('电机里的标定和文件不一致。请先运行 calibrate_leader.py')
        run(leader, args.speed, 0.25, args.lead)
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
