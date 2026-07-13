#!/usr/bin/env python3
"""平衡车手动烧录脚本：交互填参数 → 写入 main.cpp → 编译烧录。

用法:
    /home/DontRain/.platformio/penv/bin/python3 tools/flash.py
    /home/DontRain/.platformio/penv/bin/python3 tools/flash.py --kp 35 --dz 12   # 也可命令行直接给

交互模式下每项显示当前固化值，直接回车 = 不改。
平衡角与遥测 Ang 同符号输入（脚本自动取负写入 target_angle，不用管内部符号）。
"""
import argparse
import glob
import re
import subprocess
import sys
from pathlib import Path

REPO = Path('/home/DontRain/Projects/SummerTask')
SRC = REPO / 'firmware/baseline-replica/src/main.cpp'
PIO = '/home/DontRain/.platformio/penv/bin/pio'

# 名称 → (源码变量, 正则, 显示时是否取负=平衡角符号转换)
FIELDS = {
    'kp':  ('balance_kp',   r'(float\s+balance_kp\s*=\s*)(-?[\d.]+)',   False),
    'kd':  ('balance_kd',   r'(float\s+balance_kd\s*=\s*)(-?[\d.]+)',   False),
    'vkp': ('velocity_kp',  r'(float\s+velocity_kp\s*=\s*)(-?[\d.]+)',  False),
    'vki': ('velocity_ki',  r'(float\s+velocity_ki\s*=\s*)(-?[\d.]+)',  False),
    'ang': ('target_angle', r'(float\s+target_angle\s*=\s*)(-?[\d.]+)', True),
    'dz':  ('dead_zone',    r'(int\s+dead_zone\s*=\s*)(-?[\d.]+)',      False),
}
LABELS = {
    'kp': '直立环 KP', 'kd': '直立环 KD', 'vkp': '速度环 KP',
    'vki': '速度环 KI', 'ang': '平衡角(°, 同遥测 Ang 符号)', 'dz': '死区补偿 PWM',
}


def read_current(src_text):
    cur = {}
    for key, (_, pat, neg) in FIELDS.items():
        m = re.search(pat, src_text)
        if not m:
            sys.exit(f'错误: 在 main.cpp 里找不到 {FIELDS[key][0]}，正则需要更新')
        val = float(m.group(2))
        cur[key] = -val if neg else val
    return cur


def main():
    ap = argparse.ArgumentParser()
    for key in FIELDS:
        ap.add_argument(f'--{key}', type=float, default=None, help=LABELS[key])
    ap.add_argument('--build-only', action='store_true', help='只编译不烧录')
    args = ap.parse_args()

    src_text = SRC.read_text()
    cur = read_current(src_text)

    cli_given = any(getattr(args, k) is not None for k in FIELDS)
    new = {}
    if cli_given:
        for key in FIELDS:
            v = getattr(args, key)
            new[key] = cur[key] if v is None else v
    else:
        print('回车 = 保留当前值\n')
        for key in FIELDS:
            raw = input(f'  {LABELS[key]} [{cur[key]:g}]: ').strip()
            if not raw:
                new[key] = cur[key]
            else:
                try:
                    new[key] = float(raw)
                except ValueError:
                    sys.exit(f'非法数值: {raw!r}')

    # 写回源码
    for key, (_, pat, neg) in FIELDS.items():
        val = -new[key] if neg else new[key]
        if key == 'dz':
            rep = rf'\g<1>{int(val)}'
        elif key == 'vki':
            rep = rf'\g<1>{val:.4f}'
        else:
            rep = rf'\g<1>{val:g}'
        src_text = re.sub(pat, rep, src_text, count=1)
    SRC.write_text(src_text)

    print('\n将固化的参数:')
    for key in FIELDS:
        mark = ' *' if new[key] != cur[key] else ''
        print(f'  {LABELS[key]}: {new[key]:g}{mark}')

    # 编译 + 烧录
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    cmd = [PIO, 'run']
    if not args.build_only:
        if not ports:
            sys.exit('\n没找到 /dev/ttyUSB*，插上 USB 线再跑（参数已写入源码，重跑直接回车全保留即可）')
        cmd += ['-t', 'upload', '--upload-port', ports[0]]
        print(f'\n烧录到 {ports[0]} ...')
    r = subprocess.run(cmd, cwd=SRC.parent.parent)
    if r.returncode == 0:
        print('\n完成 ✔  （板子已复位，参数即当前固化值）')
    else:
        sys.exit('\n编译或烧录失败，见上方输出')


if __name__ == '__main__':
    main()
