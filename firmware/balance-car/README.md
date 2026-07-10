# 平衡车固件（Arduino UNO）

PlatformIO 工程。引脚分配见 [../../docs/hardware.md](../../docs/hardware.md)，
串口协议见 [../../docs/serial-protocol.md](../../docs/serial-protocol.md)。

## 编译与烧录

```bash
pio run                 # 编译
pio run -t upload       # 烧录（⚠️ 烧录前断开 D0/D1 与 RDK X5 的连线）
pio device monitor      # 串口监视 115200
```

## 调参流程（阶段 1）

1. **先调直立环 PD**：速度环/转向环置零。加大 Kp 直到小车来回振荡，再加 Kd 抑制振荡。
   方向反了（越推越倒）说明 PWM 或 MPU 安装方向取反。
2. **再调速度环 PI**：目标速度 0，Kp 由小到大，小车应能抵抗缓慢漂移；Ki 消除稳态偏移。
3. **最后转向环 PD**：给目标 ω，观察原地转向是否平稳。
4. 倾角保护阈值（如 ±40°）触发时必须立即关 PWM，防止摔倒后轮子狂转。

## 上位机联调

PC 上用 Python + pyserial 按协议解析遥测帧、发送速度帧，验证协议正确后再接 RDK X5。
（联调脚本放 `tools/`，阶段 1 编写。）
