# SummerTask：平衡小车 + OpenClaw 机器人

暑期任务：双轮自平衡小车底盘 + ROS2 自主导航 + AI 语音交互的三层架构机器人，
最终演示「语音下达指令 → 表情反馈 → 自主从 A 教室门口导航到 B 教室门口」。

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│ 交互层  ESP32-S3 + esp-claw                              │
│   麦克风/扬声器（对话） TFT（表情） 舵机（肢体动作）        │
└───────────────┬─────────────────────────────────────────┘
                │ UART / Wi-Fi
┌───────────────┴─────────────────────────────────────────┐
│ 导航层  RDK X5 + ROS2 Humble + OpenClaw                  │
│   单线激光雷达 + 视觉 → slam_toolbox 建图 / Nav2 导航     │
│   robot_localization EKF 融合里程计 + IMU                │
└───────────────┬─────────────────────────────────────────┘
                │ UART 115200（见 docs/serial-protocol.md）
┌───────────────┴─────────────────────────────────────────┐
│ 底盘层  Arduino UNO                                      │
│   MPU6050 姿态解算 → 串级 PID（直立/速度/转向）           │
│   → TB6612 驱动电机；11 线编码器测速 → 上报里程计/IMU     │
└─────────────────────────────────────────────────────────┘
```

## 目录导航

| 目录 | 内容 | 对应阶段 |
|---|---|---|
| [`docs/`](docs/) | 架构、路线图、串口协议、硬件接线 | — |
| [`firmware/balance-car/`](firmware/balance-car/) | Arduino UNO 平衡车固件（PlatformIO） | 阶段 1 |
| [`ros2_ws/src/`](ros2_ws/src/) | ROS2 底盘驱动 / bringup / URDF | 阶段 2–3 |
| [`esp32/`](esp32/) | esp-claw 工程与 Lua skills | 阶段 4 |
| [`pcb/`](pcb/) | 立创 EDA 载板设计 | 阶段 5 |
| [`mechanical/`](mechanical/) | Fusion 3D 固定件建模 | 阶段 6 |

## 快速上手

1. 阅读 [`docs/roadmap.md`](docs/roadmap.md) 了解七阶段计划与验收标准
2. 阶段 1 从 `firmware/balance-car/` 开始：PlatformIO 编译烧录到 UNO
3. RDK X5 侧：把 `ros2_ws/` 拷到板上 `colcon build`（板载 Ubuntu 22.04 + ROS2 Humble）

## 技术栈

Arduino (PlatformIO) · ROS2 Humble · slam_toolbox · Nav2 · robot_localization ·
OpenClaw (+rosclaw 桥接) · esp-claw (ESP-IDF v5.5, Lua skills) · 立创EDA · Fusion 360
