# LD03 X5 直连雷达驱动

该包在 RDK X5 上直接读取 40Pin UART1 的 `/dev/ttyS1`，解析 LD03 的 115200 波特率数据，并发布 `sensor_msgs/msg/LaserScan`。

## 前置硬件接线

- 雷达数据线接物理 10（UART1 RX）。
- 雷达 5V、GND、3.3V PWM 按当前硬件接线表连接；不要连接物理 8（UART TX）。
- 雷达通电后须持续旋转。

## 在 X5 构建

```bash
cd ~/intel-balance-car/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ld03_lidar --symlink-install
source install/setup.bash
```

## 启动和验证

```bash
ros2 launch ld03_lidar ld03_lidar.launch.py
```

在另一终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/intel-balance-car/ros2_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

期望：`/scan` 的 `header.frame_id` 为 `laser_link`，`ranges` 包含以米为单位的距离。实习方案要求扫描频率达到 5-10Hz；必须以 `ros2 topic hz /scan` 的现场输出为准。
