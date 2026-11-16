"""Record an SO-101 leader calibration and save the file used by real_keyboard.py.

Run with the leader environment, while the arm is plugged in and supported:

    .venv-leader\\Scripts\\python.exe calibrate_leader.py

Torque is turned off. Move the arm by hand. The result is written into the motors
and to the LeRobot calibration file named by --id.
"""
import argparse
import sys
from pathlib import Path


DEFAULT_ID = '7vb_leader_arm'
MOTORS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
# Tick span that still fits in one encoder turn. A larger span means the joint wrapped.
SPAN_LIMITS = {
    'shoulder_pan': (700, 3400),
    'shoulder_lift': (700, 3400),
    'elbow_flex': (700, 3400),
    'wrist_flex': (700, 3400),
    'gripper': (500, 2200),
}


def leader_classes():
    try:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    except ImportError as exc:
        raise SystemExit(
            '当前解释器没有 LeRobot。请用主臂环境运行：\n'
            r'.venv-leader\Scripts\python.exe calibrate_leader.py'
        ) from exc
    return SO101Leader, SO101LeaderConfig


def serial_ports():
    from serial.tools import list_ports
    return list(list_ports.comports())


def choose_port(requested):
    ports = serial_ports()
    if requested:
        known = {port.device.upper() for port in ports}
        if known and requested.upper() not in known:
            listed = ', '.join(port.device for port in ports) or '无'
            raise SystemExit(f'找不到串口 {requested}。当前可见串口：{listed}')
        return requested
    if len(ports) == 1:
        port = ports[0]
        print(f'使用唯一串口 {port.device}（{port.description}）', flush=True)
        return port.device
    if not ports:
        raise SystemExit('没有发现串口。请确认主臂已连接并安装了 USB 串口驱动。')
    lines = '\n'.join(f'  {port.device}  {port.description}' for port in ports)
    raise SystemExit(f'发现多个串口，请用 --port 指定主臂：\n{lines}')


def enable_console():
    if sys.platform != 'win32':
        return
    import ctypes
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    handle = kernel.GetStdHandle(-11)
    mode = ctypes.c_uint()
    if kernel.GetConsoleMode(handle, ctypes.byref(mode)):
        kernel.SetConsoleMode(handle, mode.value | 0x0004)


def print_steps(path):
    print(
        '主臂力矩会关闭，请用手扶住，不要让它自由落下。\n'
        '一次只校准一个电机，其他电机不要动。这个电机合格后才写入。\n'
        f'结果保存到：\n     {path}\n'
        '每个电机：\n'
        '  1. 只把这一个关节摆到行程中间，夹爪则半开，然后按回车。\n'
        '  2. 只转动这一个关节：肩、肘、腕转到两个机械尽头，不要转过一整圈。\n'
        '     夹爪先完全合拢，再完全张开。wrist_roll 不用来回转。\n'
        '  3. 转完后按回车。不合格会要求重做这一个，不会动其他电机。\n',
        flush=True,
    )


def span_problems(calibration):
    problems = []
    for name in MOTORS:
        if name == 'wrist_roll':
            continue
        item = calibration[name]
        span = int(item.range_max) - int(item.range_min)
        low, high = SPAN_LIMITS[name]
        if span < low:
            problems.append(f'{name} 只有 {span} 个刻度，没有转到两头')
        elif span > high:
            problems.append(f'{name} 有 {span} 个刻度，转过了一圈，零点会错')
    return problems


def print_spans(calibration):
    print('这次记录到的行程：', flush=True)
    for name in MOTORS:
        item = calibration[name]
        span = int(item.range_max) - int(item.range_min)
        print(f'  {name:<14} {item.range_min:>5} .. {item.range_max:<5} 共 {span} 个刻度', flush=True)


def save_motor(leader, name, item):
    leader.calibration[name] = item
    leader.bus.write_calibration({name: item}, cache=False)
    leader.bus.calibration = dict(leader.calibration)
    leader._save_calibration()


def motor_problem(name, item):
    if name == 'wrist_roll':
        return None
    span = int(item.range_max) - int(item.range_min)
    low, high = SPAN_LIMITS[name]
    if span < low:
        return f'{name} 只有 {span} 个刻度，没有转到两头'
    if span > high:
        return f'{name} 有 {span} 个刻度，转过了一圈，零点会错'
    return None


def record_one(leader, name):
    full_turn = [name] if name == 'wrist_roll' else []
    while True:
        print(f'\n===== 只校准 {name}，其他关节不要动 =====', flush=True)
        try:
            recorded = leader.bus.record_calibration(
                'position', full_turn_motors=full_turn, motors=[name])
        except ValueError as exc:
            print(f'记录失败：{exc}', flush=True)
            print('这个关节没有形成行程。请只重做这一个。', flush=True)
            continue
        item = recorded[name]
        span = int(item.range_max) - int(item.range_min)
        print(f'{name}: {item.range_min} .. {item.range_max}，共 {span} 个刻度', flush=True)
        problem = motor_problem(name, item)
        if problem is None:
            save_motor(leader, name, item)
            print(f'{name} 已保存。', flush=True)
            return
        print(f'这次没有保存：{problem}', flush=True)
        print('请只重做这一个电机。', flush=True)


def record_each(leader):
    print('按回车开始下一个电机，输入 q 后回车可以停。', flush=True)
    for index, name in enumerate(MOTORS, 1):
        answer = input(f'[{index}/6] 准备校准 {name}。回车开始，q 结束：').strip().lower()
        if answer == 'q':
            print('已停在当前电机，前面保存过的电机仍然有效。', flush=True)
            return
        record_one(leader, name)
    print('六个电机都已逐个校准。', flush=True)


def record_until_valid(leader):
    while True:
        try:
            calibration = leader.bus.record_calibration('position', full_turn_motors=['wrist_roll'])
        except ValueError as exc:
            print(f'记录失败：{exc}', flush=True)
            print('有关节完全没动。请再做一遍。', flush=True)
            continue
        print_spans(calibration)
        problems = span_problems(calibration)
        if not problems:
            leader.calibration = calibration
            leader.bus.write_calibration(calibration)
            leader._save_calibration()
            print('行程合格，已保存。', flush=True)
            return
        print('这次没有保存：', flush=True)
        for line in problems:
            print(f'  {line}', flush=True)
        print('请再做一遍。肩和肘只转到机械尽头，不要连续转过一整圈。夹爪要完全合拢再完全张开。', flush=True)


def print_angles(leader):
    action = leader.get_action()
    print('当前角度（度）：', flush=True)
    for name in MOTORS:
        print(f'  {name:<14} {action[f"{name}.pos"]:+8.2f}', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='校准已连接的 SO-101 主臂')
    parser.add_argument('--port', help='主臂串口，例如 COM5；只连着一个串口时可以省略')
    parser.add_argument('--id', default=DEFAULT_ID,
                        help=f'标定文件名，默认 {DEFAULT_ID}，与 real_keyboard.py 一致')
    parser.add_argument('--keep', action='store_true',
                        help='已有同名文件时写回电机，不重新记录行程')
    parser.add_argument('--each', action='store_true',
                        help='一次只校准一个电机，保留其他电机已有的标定')
    parser.add_argument('--motor',
                        help='只校准一个电机，可用 1–6 或名称，例如 2 或 shoulder_lift')
    args = parser.parse_args(argv)
    if args.motor:
        if args.motor.isdigit():
            index = int(args.motor)
            if index not in range(1, 7):
                parser.error('--motor 编号必须是 1 到 6')
            args.motor = MOTORS[index - 1]
        elif args.motor not in MOTORS:
            parser.error('--motor 必须是 1–6 或 ' + ', '.join(MOTORS))

    SO101Leader, SO101LeaderConfig = leader_classes()
    port = choose_port(args.port)
    leader = SO101Leader(SO101LeaderConfig(port=port, id=args.id))
    if not args.keep and not args.each and not args.motor:
        leader.calibration = {}
    enable_console()
    print_steps(leader.calibration_fpath)
    leader.connect(calibrate=False)
    try:
        if args.keep:
            leader.calibrate()
        elif args.motor:
            print(f'只校准电机 {MOTORS.index(args.motor) + 1}：{args.motor}。其他电机保持不动。', flush=True)
            record_one(leader, args.motor)
        elif args.each:
            record_each(leader)
        else:
            record_until_valid(leader)
        if not leader.calibration_fpath.is_file():
            raise RuntimeError(f'标定没有写入 {leader.calibration_fpath}')
        print_angles(leader)
    finally:
        if leader.is_connected:
            leader.disconnect()
    print(f'标定完成：{Path(leader.calibration_fpath).resolve()}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
