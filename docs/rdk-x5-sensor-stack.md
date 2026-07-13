# RDK X5 传感器、手持建图与定位记录

本文记录已在 RDK X5 实机验证通过的导航传感器链路。底盘未完成前，曾用“手持 X5 + 雷达”验证雷达建图与 AMCL 定位；该验证已完成，后续不再继续手持测试，也不得把手持地图或 RF2O 作为最终导航方案。

## 已验证结果

- LD08 通过 X5 板载 UART 发布 `/scan`，实测约 7.6 Hz。
- IMX219 MIPI 相机发布 `/image_raw`，约 30 Hz，消息帧名为 `camera_link`。
- 雷达消息帧名为 `laser_link`；URDF 已发布 `base_link -> laser_link`、`base_link -> camera_link`。
- RF2O 基于 `/scan` 发布 `/odom_rf2o` 与 `odom -> base_link`，实测约 6 Hz。
- `slam_toolbox` 已生成并保存 5 cm 分辨率地图；AMCL 已使用该地图完成手持重新定位。

## LD08 到 X5 的接线

必须在 X5 断电状态下改线。40Pin 使用物理针脚编号。

| LD08 线色 | 信号 | X5 物理针脚 | 说明 |
|---|---|---:|---|
| 黑 | P5V | 2 | 5V |
| 白 | GND | 9 | 地 |
| 黄 | Tx | 10 | UART_RXD；LD08 单向发送数据到 X5 |
| 红 | PWM | 33 | PWM7，3.3V 逻辑 |

LD08 不是恒定 3.3V 使能模式：数据手册要求 PWM 为 10–30 kHz，典型值 16 kHz、60% 占空比。项目脚本 `balance_bot_bringup/scripts/ld08_pwm.py` 在物理针脚 33 输出该波形。

因此，雷达只有在相关 ROS launch 运行时才会转；关闭 launch 后，PWM 脚本退出并释放引脚，雷达停止是正常现象。

## 软件位置与依赖

- 工作区：`~/intel-balance-car/ros2_ws`
- LD08 UART 驱动：`ros2_ws/src/ld03_lidar`。包名保留历史名称 `ld03_lidar`，但 LD08 的 UART 协议兼容：115200 8N1、帧头 `0x54`、`VerLen=0x2C`、每帧 12 个点。
- 一键启动包：`ros2_ws/src/balance_bot_bringup`
- 手持激光里程计：X5 工作区的 `src/rf2o_laser_odometry`，上游 `humble-devel` 分支构建。

每次新开 X5 Shell，按用途加载环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash   # 启动 MIPI 相机时需要
source ~/intel-balance-car/ros2_ws/install/setup.bash
```

## 交接前准备

### 交接现成 TF 卡（推荐）

本项目的 RDK X5 通过 TF 卡启动。交接时若一并提供**当前已烧录并验证可启动的 TF 卡**，新接手者无需重新下载镜像、烧录系统或重新安装 ROS/TROS 环境：断电插卡、启动 X5、连接 Wi-Fi、同步仓库代码并重新构建即可。不要在 X5 运行时插拔 TF 卡。

TF 卡中的 Ubuntu 22.04、ROS2 Humble、官方 TROS（含 `mipi_cam`）、已安装依赖、Wi-Fi 配置，以及可能存在的地图文件都不由 Git 仓库恢复。交接前应建立 TF 卡镜像备份；TF 卡丢失、损坏或更换 X5 时，才按下一节重装环境。

### 重刷系统或更换 TF 卡时恢复环境

新 TF 卡上的 RDK X5 需具备 Ubuntu 22.04、ROS2 Humble 和官方 TROS（提供 `mipi_cam`）。安装 ROS 导航依赖：

```bash
apt-get update
apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-xacro
```

RF2O 不在当前 apt 源中，需在工作区恢复：

```bash
cd ~/intel-balance-car/ros2_ws/src
git clone --depth 1 --branch humble-devel \
  https://github.com/Adlink-ROS/rf2o_laser_odometry.git

cd ~/intel-balance-car/ros2_ws
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
colcon build --packages-select \
  rf2o_laser_odometry ld03_lidar balance_bot_description balance_bot_bringup \
  --symlink-install
```

当前实测地图只保存在 X5 的 `~/maps/handheld_room.yaml`、`~/maps/handheld_room.pgm`，不在仓库中。交接或重刷系统前必须备份：

```bash
mkdir -p maps
scp root@<X5_IP>:/root/maps/handheld_room.yaml maps/
scp root@<X5_IP>:/root/maps/handheld_room.pgm maps/
```

## 三种启动方式

三种模式互斥：同一时刻只启动其中一个，因为后两种已包含传感器和 PWM。

### 只运行传感器

```bash
ros2 launch balance_bot_bringup sensors.launch.py
```

用于检查雷达、相机、静态 TF。验证雷达：

```bash
timeout 15 ros2 topic hz /scan
```

### 手持建图

> 仅用于已完成的传感器与 SLAM 链路验证。本项目后续建图应等待小车底盘、编码器和 IMU 接入后在车上进行；本节保留为排障参考，不作为日常操作流程。

```bash
ros2 launch balance_bot_bringup handheld_mapping.launch.py
```

保持雷达水平，先静置约 5 秒，再缓慢绕室内环境移动并回到起点。建图完成后保持 launch 运行，保存地图：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/handheld_room
```

产物为 `~/maps/handheld_room.yaml` 和 `~/maps/handheld_room.pgm`。该地图仅是手持验证产物，不能用于最终 Nav2 导航；装车后须重新建图并保存正式地图。

### 建图可视化（RViz2 / Foxglove）

先在一个终端启动 `handheld_mapping.launch.py`，并保持它运行。它会发布 `/scan`、`/odom_rf2o`、`/map`、`/tf` 和 `/tf_static`。

#### RViz2：仅用于短时调试

RDK X5 的串口或普通 SSH Shell 没有图形显示环境，不能直接在其中运行 `rviz2`；会出现 `could not connect to display`。若电脑与 X5 在同一网络，可在电脑上通过 X11 转发运行 X5 的 RViz2：

```bash
# 首次在 X5 安装
ssh -t root@<X5_IP> 'apt-get update && apt-get install -y ros-humble-rviz2'

# 在电脑终端运行；RViz 窗口显示在电脑上
ssh -Y -t root@<X5_IP> '
source /opt/ros/humble/setup.bash
source ~/intel-balance-car/ros2_ws/install/setup.bash
rviz2
'
```

在 RViz2 中设置 `Global Options -> Fixed Frame` 为 `map`，再依次添加：`Map`（`/map`）、`LaserScan`（`/scan`）、`TF`，以及可选的 `Odometry`（`/odom_rf2o`）。X11 会转发 RViz 图形，可能有明显延迟，不作为日常可视化方案。

#### Foxglove：SSH 场景推荐

Foxglove Bridge 在 X5 提供 WebSocket，电脑浏览器直接渲染地图，避免 X11 图形转发延迟。首次安装：

```bash
ssh -t root@<X5_IP> 'apt-get update && apt-get install -y ros-humble-foxglove-bridge'
```

建图 launch 运行期间，在另一个电脑终端启动桥接：

```bash
ssh -t root@<X5_IP> '
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
'
```

在同一网络的电脑浏览器打开 `https://app.foxglove.dev`，选择 **Foxglove WebSocket**，连接地址填写 `ws://<X5_IP>:8765`。添加 `3D` 面板，参考坐标系选择 `map`，并显示 `/map`、`/scan`、`/tf`。浏览器应使用 Chromium/Chrome 119 或更高版本；仅保留 `/image_jpeg` 预览，关闭高带宽的 `/image_raw` 面板。

`/scan` 中的 `Infinity` 表示无有效测距回波，Foxglove 可能以黑色/品红色提示；这是合法的 `LaserScan` 数据，SLAM 会忽略它们，不能将其替换为 `0`。

### 手持 AMCL 定位

先停止建图 launch；`slam_toolbox` 和 AMCL 不能同时发布 `map -> odom`。

```bash
ros2 launch balance_bot_bringup handheld_localization.launch.py \
  map:=/root/maps/handheld_room.yaml
```

放回建图起点附近后，需通过 RViz 的 `2D Pose Estimate` 或向 `/initialpose` 发布初始位姿。确认状态：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

两者都应为 `active [3]`。定位结果：

```bash
timeout 15 ros2 topic echo /amcl_pose --once --field pose.pose
```

## 重要故障判断

| 现象 | 判断与处理 |
|---|---|
| 雷达转但 `/scan` 无消息 | 先停掉雷达节点，再直接读取 `/dev/ttyS1`；不要让 `dd` 与雷达节点同时读串口。 |
| 串口没有稳定 `54 2c` 帧头 | 优先检查黄色 Tx 数据线、白色地线及雷达端 4Pin；LD08 稳定旋转后应持续自动发数据，无需发送命令。 |
| 红线接 PWM 后雷达不转 | 确认红线在物理 33，并由 `sensors.launch.py` 或其包含的 launch 启动 PWM 脚本；单独接 PWM 针脚不会自动产生波形。 |
| RF2O 偶尔打印 `Waiting for laser_scans` | 正常：RF2O 以 20 Hz 轮询，而雷达约 7.6 Hz。只要持续出现 `BASEodom` 与 `/odom_rf2o`，即为正常工作。 |
| `map_server` / `amcl` 是 `unconfigured [1]` | 检查 `config/nav2_params.yaml` 的 `map_server.ros__parameters.yaml_filename: ""` 是否已同步并重新构建、重启。 |

## 底盘完成后再做

- 实测并写入雷达、相机相对 `base_link` 的安装位姿；当前 URDF 默认值仅用于验证 TF 拓扑。
- 用 UNO 编码器与 MPU6050 替代 RF2O，发布稳定的 `/odom_raw` 与 `/imu/data_raw`，再由 EKF 输出正式 `odom -> base_link`。
- 使用装车后的真实里程计低速遥控建图，保存正式地图；不要复用 `handheld_room` 地图。
- 启动完整 Nav2，接通 `/cmd_vel` 到底盘，实现自动导航。

交接时还应一并提供 LD08 数据手册、当前 40Pin 接线照片，以及实际安装后的传感器外参测量值；这些内容目前不在版本库内。
