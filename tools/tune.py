#!/usr/bin/env python3
"""平衡车在线调参工具（配 firmware/baseline-replica 的 [PATCH P7] 串口命令）。

用法:
    /home/DontRain/.platformio/penv/bin/python tools/tune.py

特性:
    - 自动探测 /dev/ttyUSB*，用 stty -hupcl 压制 open 复位（车可以边站边调）。
      例外：USB 线刚插上后的第一次连接仍会复位一次板子（参数回默认）
    - 后台线程持续打印遥测（Ang/VL/VR/Bat/PWM/Stop），t 键可关掉显示（仍写日志）
    - 所有收发都带时间戳记录到日志文件
    - 拔线自动重连
    ⚠️ 不要和后台遥测记录器同时开，串口只能被一个进程占用

命令（回车发送）:
    p36        直立环 KP = 36            p+ / p-   步进 ±1.0
    d0.9       直立环 KD                 d+ / d-   步进 ±0.05
    v1.5       速度环 KP                 v+ / v-   步进 ±0.1
    i0.0075    速度环 KI                 i+ / i-   步进 ±0.0005
    a0.2       平衡角（°，同遥测 Ang 符号） a+ / a-   步进 ±0.1
    z12        死区补偿 PWM              z+ / z-   步进 ±1
    ?          让固件报当前参数
    g / x      启动 / 停止电机
    t          开关遥测显示
    q          退出
注意: 参数只存在 RAM，断电/复位后回到固件默认值。调好后把数值抄下来固化进代码。
"""
import glob
import subprocess
import sys
import time
import threading
from datetime import datetime

import serial

STEPS = {'p': 1.0, 'd': 0.05, 'v': 0.1, 'i': 0.0005, 'a': 0.1, 'z': 1}
DECIMALS = {'p': 1, 'd': 2, 'v': 2, 'i': 4, 'a': 2, 'z': 0}
# [PARAM] 行字段名 → 命令字母
PARAM_KEYS = {'KP': 'p', 'KD': 'd', 'VKP': 'v', 'VKI': 'i', 'BAL_ANG': 'a', 'DZ': 'z'}

LOG_PATH = ('/tmp/claude-1000/-home-DontRain-Projects-SummerTask/'
            '8370d653-c04c-4d59-86b4-e7ad8a4f48df/scratchpad/'
            f'tune_{datetime.now():%Y%m%d_%H%M%S}.log')

show_telemetry = True
params = {}          # 命令字母 → 最近一次固件回报的值
params_lock = threading.Lock()
stop_flag = False


def log(fp, direction, text):
    fp.write(f'{datetime.now():%H:%M:%S.%f} {direction} {text}\n')
    fp.flush()


def find_port():
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    return ports[0] if ports else None


def open_port(path):
    # 本车适配器 dtr=False 也压不住 open 时的复位脉冲，实测要靠 -hupcl：
    # 清掉 hupcl 后关闭串口不再拉低 DTR，后续 open 就没有复位沿。
    # stty 自身开关串口会触发最后一次复位（板子重启、参数回默认，约 3 秒），
    # 之后本进程内的重连都是无复位热接入。
    subprocess.run(['stty', '-F', path, '-hupcl', '9600'], check=False)
    time.sleep(0.1)
    s = serial.Serial()
    s.port = path
    s.baudrate = 9600
    s.timeout = 0.5
    s.dtr = False
    s.rts = False
    s.open()
    return s


def parse_param_line(line):
    """从 '[PARAM] KP=36.00 KD=0.90 ...' 更新本地参数表"""
    with params_lock:
        for field in line.replace('[PARAM]', '').split():
            if '=' not in field:
                continue
            k, _, v = field.partition('=')
            if k in PARAM_KEYS:
                try:
                    params[PARAM_KEYS[k]] = float(v)
                except ValueError:
                    pass


def reader(ser_ref, fp):
    restore = {}   # 板子复位时的参数快照，READY 后回灌
    while not stop_flag:
        ser = ser_ref[0]
        if ser is None:
            time.sleep(0.3)
            continue
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError):
            print('\n[tune] 串口断开，等待重连……')
            try:
                ser.close()
            except Exception:
                pass
            ser_ref[0] = None
            while not stop_flag and ser_ref[0] is None:
                port = find_port()
                if port:
                    try:
                        ser_ref[0] = open_port(port)
                        print(f'[tune] 已重连 {port}')
                        ser_ref[0].write(b'?\n')
                    except (serial.SerialException, OSError):
                        time.sleep(0.5)
                else:
                    time.sleep(0.5)
            continue
        if not raw:
            continue
        line = raw.decode('ascii', errors='replace').strip()
        if not line:
            continue
        log(fp, '<', line)
        # 板子意外复位（USB 掉线重连等）→ 参数回默认 + 电机停机。
        # 在启动横幅时抢先快照调好的参数，READY 后自动发回去。
        if 'MiniBalance Starting' in line:
            with params_lock:
                restore = dict(params)
            if restore:
                print('\n[tune] ⚠️ 板子复位了！等它启动完自动恢复参数……')
        if 'SYSTEM READY' in line and ser_ref[0] and restore:
            for letter, val in restore.items():
                fmt = f'{letter}{val:.{DECIMALS[letter]}f}\n'
                try:
                    ser_ref[0].write(fmt.encode('ascii'))
                    log(fp, '>', fmt.strip() + ' (auto-restore)')
                except (serial.SerialException, OSError):
                    break
            print('[tune] ⚠️ 参数已自动恢复，但电机是停的：车扶直后发 g 启动')
            restore = {}
        if line.startswith('[PARAM]'):
            parse_param_line(line)
            print(f'\r{line}')
        elif show_telemetry or not line.startswith('Ang:'):
            print(f'\r{line}')


def main():
    global show_telemetry, stop_flag

    port = find_port()
    if not port:
        print('[tune] 没找到 /dev/ttyUSB*，先插好 USB 线再运行')
        sys.exit(1)

    fp = open(LOG_PATH, 'a')
    ser_ref = [open_port(port)]
    print(f'[tune] 已连接 {port}（无复位打开），日志: {LOG_PATH}')
    print('[tune] 输入 help 看命令，q 退出')

    t = threading.Thread(target=reader, args=(ser_ref, fp), daemon=True)
    t.start()

    time.sleep(0.3)
    ser_ref[0].write(b'?\n')   # 拉取当前参数

    while True:
        try:
            cmd = input().strip().replace(' ', '')
        except (EOFError, KeyboardInterrupt):
            cmd = 'q'

        if not cmd:
            continue
        if cmd == 'q':
            break
        if cmd == 'help':
            print(__doc__)
            continue
        if cmd == 't':
            show_telemetry = not show_telemetry
            print(f'[tune] 遥测显示: {"开" if show_telemetry else "关（仍在写日志）"}')
            continue

        # 步进命令 p+ / p- 等：基于固件最近回报的值加减
        if len(cmd) == 2 and cmd[0] in STEPS and cmd[1] in '+-':
            key = cmd[0]
            with params_lock:
                cur = params.get(key)
            if cur is None:
                print('[tune] 还没拿到当前参数，先发 ? 等 [PARAM] 行出来再步进')
                continue
            new = cur + STEPS[key] * (1 if cmd[1] == '+' else -1)
            cmd = f'{key}{new:.{DECIMALS[key]}f}'
            print(f'[tune] {key}: {cur} -> {cmd[1:]}')

        if ser_ref[0] is None:
            print('[tune] 串口未连接，命令没发出去')
            continue
        try:
            ser_ref[0].write((cmd + '\n').encode('ascii'))
            log(fp, '>', cmd)
        except (serial.SerialException, OSError):
            print('[tune] 发送失败（串口断开？），等重连后重试')

    stop_flag = True
    time.sleep(0.2)
    if ser_ref[0]:
        ser_ref[0].close()
    fp.close()
    # 退出时把最后已知参数打出来，方便抄回固件
    if params:
        print('[tune] 最后已知参数（记得固化进 main.cpp 初值）:')
        with params_lock:
            for k, letter in [('KP', 'p'), ('KD', 'd'), ('VKP', 'v'),
                              ('VKI', 'i'), ('BAL_ANG', 'a'), ('DZ', 'z')]:
                if letter in params:
                    print(f'    {k} = {params[letter]}')


if __name__ == '__main__':
    main()
