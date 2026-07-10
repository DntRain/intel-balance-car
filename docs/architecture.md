# 系统架构

## 三层分工

| 层 | 主控 | 职责 | 实时性 |
|---|---|---|---|
| 底盘层 | Arduino UNO | 姿态解算、平衡控制、电机驱动、里程计/IMU 上报 | 硬实时，控制环 ≥200Hz |
| 导航层 | RDK X5 | 传感器融合、SLAM、路径规划、OpenClaw 任务调度 | 软实时，ROS2 |
| 交互层 | ESP32-S3 | 语音对话、TFT 表情、舵机动作（esp-claw Lua skills） | 事件驱动 |

设计原则：**平衡控制完全闭环在 UNO 内**，RDK X5 只下发目标速度（v, ω），
串口断线或指令超时（>500ms）时 UNO 自动把目标速度清零、仅维持原地平衡——上层崩溃不摔车。

## 数据流

```
MPU6050 ──I2C──▶ UNO ◀──中断── 编码器×2
                 │  ▲
     上行 50Hz 遥测帧  │  下行 cmd_vel 帧          （UART 115200，注意电平匹配）
                 ▼  │
雷达 ──USB/UART──▶ RDK X5 ◀──CSI/USB── 摄像头
                 │
   /odom_raw + /imu/data_raw ─▶ robot_localization EKF ─▶ /odometry/filtered
   /scan ─▶ slam_toolbox（建图）/ AMCL（定位）─▶ Nav2 ─▶ /cmd_vel
                 │
            OpenClaw（自然语言 → Nav2 目标点，经 rosclaw 或自写 skill）
                 │
     Wi-Fi / UART（待定，见开放问题）
                 ▼
ESP32-S3（esp-claw）──▶ TFT 表情 / 舵机 / 扬声器；◀── 麦克风
```

## 技术选型

- **固件**：PlatformIO 工程（版本管理友好），Arduino 框架
- **姿态解算**：互补滤波（一阶）起步，不满意再上卡尔曼；不用 MPU6050 DMP
  （DMP 需要 INT 引脚，而 UNO 仅有的 D2/D3 外部中断要留给两路编码器 A 相）
- **ROS2**：RDK X5 官方 Ubuntu 22.04 + ROS2 Humble；建图 `slam_toolbox`、
  导航 `Nav2`、融合 `robot_localization`（EKF：轮式里程计 + IMU）
- **OpenClaw 桥接**：优先评估社区项目 rosclaw；备选方案是写一个 OpenClaw skill
  直接调 `ros2 action send_goal` / rclpy
- **esp-claw**：乐鑫官方 espressif/esp-claw，ESP-IDF v5.5，skills 用 Lua 热加载

## 供电拓扑

```
动力锂电池（2S/3S，按套件）
 ├─▶ TB6612 VM（电机电源，直连）
 ├─▶ 5V DC-DC（≥3A）─▶ RDK X5（X5 峰值电流大，供电必须充足）
 │                    └▶ UNO（5V 脚）、ESP32-S3 载板
 └─▶ 5V DC-DC（独立）─▶ 舵机（舵机堵转电流大，必须与逻辑电源分离）
共地：所有模块 GND 必须共地，尤其 UNO ↔ RDK X5 串口两端。
```

## 参考架构对比

朋友的同款车项目（lxbme/arduino-balance-car + esp32_sensor_bridge）走的是
「UNO →I2C→ ESP32 →micro-ROS(Wi-Fi)→ ROS2 主机」，雷达（LD08）也挂在 ESP32 上。
我们不沿用该架构，原因：任务书明确要求 UNO **串口**上报，且 RDK X5 就在车上，
UART 直连比 Wi-Fi micro-ROS 时延低、少一层故障点。朋友项目的价值在于
实测引脚/PID 参数/LD08 驱动代码（见 references.md）。

## 开放问题（实现阶段确认）

1. ESP32-S3 与 RDK X5 之间走 Wi-Fi（esp-claw 网络层对接 OpenClaw）还是 UART——esp-claw 教程走通后定
2. LD08 接 X5 用板载 UART 还是 USB 转串口
3. 摄像头在导航中的角色（纯预留 or 视觉辅助定位）
