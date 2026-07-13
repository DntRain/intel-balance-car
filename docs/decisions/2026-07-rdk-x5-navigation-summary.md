# 2026-07 RDK X5 导航决策摘要

这是对已删除的 `docs/superpowers/` 过程草稿的压缩摘要。日常操作以 [RDK X5 传感器、手持建图与定位记录](../rdk-x5-sensor-stack.md) 为准。

## 最终主路线

```text
LD08 --UART /dev/ttyS1--> ld03_lidar --/scan--> RF2O / slam_toolbox / AMCL
IMX219 MIPI ---------------------------------->/image_raw
UNO（待完成） --UART--> 底盘驱动 --/odom_raw、/imu/data_raw、/cmd_vel
```

- LD08 直接接 RDK X5，不经过 ESP32；UART 使用物理针脚 10（RX），只读。
- LD08 PWM 使用物理针脚 33，16 kHz、60% 占空比；不能以恒定 3.3V 取代正式 PWM。
- 实际型号为 LD08；代码包名称 `ld03_lidar` 是历史名称，但 47 字节 `0x54 0x2C` 协议兼容。
- MIPI 相机启动时固定 `frame_id: camera_link`；雷达使用 `laser_link`。

## 已完成并验证

- `/scan`、`/image_raw`、传感器静态 TF。
- RF2O 手持激光里程计。
- `slam_toolbox` 手持建图、标准地图保存。
- 手持建图与 AMCL 已完成链路验证；RF2O 和 `handheld_room` 地图仅供验证，不进入最终验收。后续改由装车后的双编码器、IMU 和 EKF 进行真实建图与导航。
- 地图加载与 AMCL 手持重新定位。

## 已放弃的主路线

最初曾实现 ESP32 + micro-ROS Agent 的雷达桥接备选方案。它保留在 `esp32/ld03_microros_bridge/` 作为备份，但不是当前导航链路；ESP32-S3 留给交互层。

## 待底盘完成后恢复

- 根据 `docs/serial-protocol.md` 实现并实测 UNO 串口底盘驱动。
- 使用轮编码器与 MPU6050，通过 EKF 取代手持模式的 RF2O。
- 标定 `base_link` 到雷达/相机的真实安装外参。
- 启动完整 Nav2 并验证 `/cmd_vel` 驱动、自动导航。
