# LD03 micro-ROS bridge

该工程运行在原 LD03 转接板上的 ESP32 上。雷达保持接在原转接板，ESP32 通过 Wi-Fi 将 `sensor_msgs/msg/LaserScan` 发布给 RDK X5；不要把雷达四线插入 X5 的 USB 口。

## 配置与烧录

1. 复制本机网络配置，并填入 ESP32 与 X5 所在的同一 Wi-Fi：

   ```bash
   cp include/network_config.example.h include/network_config.h
   ```

2. 将 `AGENT_IP` 填为 X5 上 `hostname -I` 显示的 IPv4 地址；当前验证地址为 `192.168.1.107`。
3. 在 Fedora 主机的本工程目录编译并上传：

   ```bash
   pio run
   pio run -t upload
   ```

`include/network_config.h` 已被忽略，不能提交 Wi-Fi 凭据。

## X5 端启动与验证

先在 RDK X5 安装 micro-ROS Agent，然后启动 UDP Agent：

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

在第二个 X5 终端验证：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

应出现 `/scan`，频率约 10Hz，且消息的 `header.frame_id` 为 `laser`。在 RViz2 中将 Fixed Frame 设为 `laser`、LaserScan Topic 设为 `/scan`，即可查看 360°扫描。
