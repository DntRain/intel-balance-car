# 平衡车固件（Arduino UNO）—— 阶段 1

基于同款车实测项目 [lxbme/arduino-balance-car](https://github.com/lxbme/arduino-balance-car)
改造：启用右编码器（原生 PCINT）、新增转向环、通信改 UART 协议、支持 cmd_vel 下发与超时保护。
引脚见 [../../docs/hardware.md](../../docs/hardware.md)，协议见 [../../docs/serial-protocol.md](../../docs/serial-protocol.md)。

## 编译与烧录

```bash
pio run                 # 编译
pio run -t upload       # 烧录（⚠️ 烧录前断开 D0/D1 与 RDK X5 的连线）
```

依赖：MsTimer2 自动拉取；KalmanFilter 已随仓库放在 `lib/`（来自基线项目）。

## 调试模式

`src/main.cpp` 顶部 `#define DEBUG_PRINT 1`：串口输出人类可读状态（5Hz），关闭二进制遥测。
联调/上车前必须改回 `0`（两者共用串口）。

## 上电行为

- 默认 `Flag_Stop=1`（电机不动），按 D3 按键或「放下检测」触发后开始平衡
- 倾角超 ±40°、电池低于 10V、被拿起 → 自动停机
- 500ms 未收到 cmd_vel 帧 → 目标速度清零，原地平衡

## 调参流程（首次上车）

1. **机械中值**：DEBUG 模式读静置角度，更新 `TARGET_ANGLE`（基线值 -2.3°）
2. **方向校验**（重要，装车后必做）：
   - 手推小车前进，DEBUG 看 VL/VR 均应为正 → 反了就在对应编码器 ISR 里调换 ± 号
   - 轻推车顶使前倾，车轮应向前追 → 反了检查电机接线/`setPwm` 极性
3. **直立环 PD**：从 KP=15/KD=0.4 起调；振荡加 KD，绵软加 KP
4. **速度环 PI**：KP=2/KI=0.01 起调，目标 0 时应能抵抗缓慢漂移
5. **转向环**：`TURN_KP/TURN_KD` 从当前初值起调（基线无此环，纯新增）
6. 用 `tools/monitor.py` 验证协议与运动指令

## 联调（阶段 1 验收）

```bash
# PC 经 USB 连 UNO（此时 D0/D1 别接 RDK X5）
python3 ../../tools/monitor.py /dev/ttyUSB0            # 应稳定收到 ≈50Hz 遥测
python3 ../../tools/monitor.py /dev/ttyUSB0 --v 100    # 小车前进 100mm/s
python3 ../../tools/monitor.py /dev/ttyUSB0 --w 0.5    # 原地转 0.5rad/s
# Ctrl+C 退出（自动发停车指令）；中断下发后 500ms 内小车应自行停住
```

验收标准（roadmap 阶段 1）：推不倒、可遥控前进转向、遥测帧可被脚本正确解析（≈50Hz、无坏帧）。
