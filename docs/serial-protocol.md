# UNO ↔ RDK X5 串口协议 v1.0

- 物理层：UART **115200 8N1**。UNO 为 5V 电平、RDK X5 为 3.3V：
  UNO TX → X5 RX 必须经分压（如 1kΩ/2kΩ）；X5 TX(3.3V) → UNO RX 可直连。
- 字节序：**小端**（little-endian）
- 帧格式：`帧头(2) + 类型(1) + 长度(1) + 载荷(N) + 校验(1)`
  - 帧头固定 `0xAA 0x55`
  - 长度 = 载荷字节数 N
  - 校验 = 类型、长度、载荷所有字节求和的低 8 位（sum & 0xFF）

## 上行帧：遥测（UNO → X5，50Hz，类型 0x01，载荷 28 字节）

| 偏移 | 字段 | 类型 | 单位/量纲 |
|---|---|---|---|
| 0 | 左轮编码器增量 | int16 | counts（自上帧累计，溢出前发送） |
| 2 | 右轮编码器增量 | int16 | counts |
| 4 | 左轮速度 | int16 | mm/s |
| 6 | 右轮速度 | int16 | mm/s |
| 8 | roll | int16 | 0.01° |
| 10 | pitch | int16 | 0.01° |
| 12 | yaw | int16 | 0.01°（陀螺积分，会漂移，仅供 EKF 参考） |
| 14 | gyro x/y/z | int16×3 | 0.1°/s |
| 20 | accel x/y/z | int16×3 | 0.001g |
| 26 | 电池电压 | uint16 | mV |

## 下行帧：速度指令（X5 → UNO，类型 0x10，载荷 4 字节）

| 偏移 | 字段 | 类型 | 单位 |
|---|---|---|---|
| 0 | 线速度 v | int16 | mm/s |
| 2 | 角速度 ω | int16 | 0.001 rad/s |

对应 ROS2 `/cmd_vel`（`geometry_msgs/Twist` 的 linear.x、angular.z）。

**安全规则**：UNO 若 **500ms** 未收到有效下行帧，目标速度自动清零（原地平衡）。

## 里程计换算常量表（两端共用，实测后更新 ⚠️）

| 常量 | 值 | 说明 |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 67.0 ⚠️待实测 | 轮径 |
| `WHEEL_TRACK_MM` | 170.0 ⚠️待实测 | 两轮中心距 |
| `ENCODER_PPR` | 11 | 编码器线数（电机轴） |
| `GEAR_RATIO` | 30 ⚠️按电机型号确认 | 减速比 |
| `COUNTS_PER_WHEEL_REV` | 11 × 4 × 30 = 1320 | 4 倍频后每轮转一圈计数 |

换算：`轮位移(mm) = 编码器增量 / COUNTS_PER_WHEEL_REV × π × WHEEL_DIAMETER_MM`

差速运动学：
```
v = (v_R + v_L) / 2          v_L = v − ω·L/2
ω = (v_R − v_L) / L          v_R = v + ω·L/2      （L = WHEEL_TRACK_MM）
```

> 本表为唯一权威定义。固件 `firmware/balance-car/src/` 与 ROS2 驱动
> `ros2_ws/src/balance_chassis_driver/` 中的常量必须与此保持一致，改动需同步三处。
