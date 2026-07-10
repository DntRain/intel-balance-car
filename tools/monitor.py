#!/usr/bin/env python3
"""平衡车串口协议联调工具（阶段 1 验收用）。

解析 UNO 上行遥测帧并打印，同时按 20Hz 下发 cmd_vel。
协议见 docs/serial-protocol.md v1.0。

用法:
    python3 monitor.py /dev/ttyUSB0                 # 只看遥测
    python3 monitor.py /dev/ttyUSB0 --v 100         # 前进 100mm/s
    python3 monitor.py /dev/ttyUSB0 --w 0.5         # 原地转 0.5rad/s
依赖: pip install pyserial
"""
import argparse
import struct
import sys
import time

import serial

FRAME_HEADER = b"\xaa\x55"
TYPE_TELEMETRY = 0x01
TYPE_CMD_VEL = 0x10
TELEMETRY_LEN = 28
# 与 docs/serial-protocol.md 上行帧字段一一对应
TELEMETRY_FMT = "<hhhhhhhhhhhhhH"


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_cmd_vel(v_mms: int, w_mrads: int) -> bytes:
    payload = struct.pack("<hh", v_mms, w_mrads)
    body = bytes([TYPE_CMD_VEL, len(payload)]) + payload
    return FRAME_HEADER + body + bytes([checksum(body)])


def parse_stream(buf: bytearray):
    """从缓冲区提取完整遥测帧，返回 (剩余缓冲, 解析出的字段元组列表)。"""
    frames = []
    while True:
        i = buf.find(FRAME_HEADER)
        if i < 0:
            del buf[:-1]  # 保留可能的半个帧头
            return frames
        if len(buf) < i + 4 + TELEMETRY_LEN + 1:
            del buf[:i]
            return frames
        ftype, flen = buf[i + 2], buf[i + 3]
        if ftype != TYPE_TELEMETRY or flen != TELEMETRY_LEN:
            del buf[: i + 2]
            continue
        body = bytes(buf[i + 2 : i + 4 + TELEMETRY_LEN])
        if buf[i + 4 + TELEMETRY_LEN] != checksum(body):
            del buf[: i + 2]
            continue
        frames.append(struct.unpack(TELEMETRY_FMT, body[2:]))
        del buf[: i + 5 + TELEMETRY_LEN]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--v", type=float, default=0.0, help="线速度 mm/s")
    ap.add_argument("--w", type=float, default=0.0, help="角速度 rad/s")
    args = ap.parse_args()

    cmd = build_cmd_vel(int(args.v), int(args.w * 1000))
    send_cmd = args.v != 0 or args.w != 0

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    buf = bytearray()
    n_frames, n_start = 0, time.monotonic()
    last_cmd = 0.0
    print(f"监听 {args.port} @{args.baud}"
          + (f"，下发 v={args.v}mm/s w={args.w}rad/s" if send_cmd else "（只读）"))
    try:
        while True:
            now = time.monotonic()
            if send_cmd and now - last_cmd >= 0.05:  # 20Hz，避开 UNO 端 500ms 超时
                ser.write(cmd)
                last_cmd = now
            buf.extend(ser.read(256))
            for f in parse_stream(buf):
                n_frames += 1
                if n_frames % 10 == 0:  # 5Hz 打印
                    (dl, dr, sl, sr, roll, pitch, yaw,
                     gx, gy, gz, ax, ay, az, vbat) = f
                    rate = n_frames / (now - n_start)
                    state = "停" if roll else "行"  # roll 字段临时复用为 Flag_Stop
                    print(f"[{rate:5.1f}Hz|{state}] dEnc L/R {dl:+4d}/{dr:+4d}  "
                          f"v L/R {sl:+5d}/{sr:+5d}mm/s  "
                          f"pitch {pitch/100:+6.2f}°  yaw {yaw/100:+7.2f}°  "
                          f"gz {gz/10:+6.1f}°/s  bat {vbat/1000:.2f}V")
    except KeyboardInterrupt:
        if send_cmd:
            ser.write(build_cmd_vel(0, 0))  # 退出前停车
        print(f"\n共收到 {n_frames} 帧，"
              f"平均 {n_frames / (time.monotonic() - n_start):.1f}Hz（应≈50Hz）")


if __name__ == "__main__":
    sys.exit(main())
