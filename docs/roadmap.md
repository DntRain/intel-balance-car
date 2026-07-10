# 开发路线图（七阶段）

每阶段以「验收标准」判定完成，按序推进；阶段 5/6（PCB、3D）可与 3/4 并行。

## 阶段 1：平衡车固件（firmware/balance-car/）

- MPU6050 姿态解算：互补滤波或卡尔曼（避免占用 D2/D3 中断脚，不用 DMP INT）
- 串级 PID：直立环（角度）→ 速度环（编码器）→ 转向环（Z 轴角速度）
- TB6612 PWM 驱动，控制环 ≥200Hz
- 11 线双通道编码器测速（A 相外部中断 + B 相判向）
- 串口协议实现：50Hz 上报里程计+IMU，接收 v/ω 速度指令（见 serial-protocol.md）

**验收**：小车静止推不倒；上位机发指令可前进/后退/转向；串口帧可被 PC 端脚本正确解析。

## 阶段 2：ROS2 底盘接入（ros2_ws/）

- `balance_chassis_driver`：串口收发节点，发布 `/odom_raw`、`/imu/data_raw`，订阅 `/cmd_vel`
- `balance_bot_description`：URDF/TF 树（odom → base_link → laser / imu_link）
- `robot_localization` EKF：融合轮式里程计 + IMU → `/odometry/filtered`
- 雷达驱动接入（按实际雷达型号选包）
- `slam_toolbox` 建图 → 保存走廊地图
- Nav2 配置（AMCL 定位 + 代价地图 + 规划器），标定 A/B 教室门口坐标

**验收**：RViz 建出走廊地图；RViz 下发目标点，小车自主从 A 教室门口到 B 教室门口。

## 阶段 3：OpenClaw 接入（RDK X5）

- RDK X5 安装 OpenClaw（Ubuntu 22.04 + Node.js 环境）
- 评估 rosclaw（github.com/PlaiPin/rosclaw）桥接；不合适则自写 OpenClaw skill 调用 Nav2 action
- 定义地点词表：「A 教室」「B 教室」→ 地图坐标

**验收**：通过聊天消息「去 B 教室」触发导航并到达。

## 阶段 4：esp-claw 交互（esp32/）

- ESP32-S3（≥8MB Flash + 8MB PSRAM）刷 esp-claw（ESP-IDF v5.5）
- Lua skills：`voice`（麦克风拾音 + 扬声器播报对话）、`expression`（TFT 表情）、`motion`（舵机肢体动作）
- 对话状态与表情/动作联动（说话时嘴部动画、挥手等）

**验收**：语音提问有应答；应答时表情与舵机动作同步。

## 阶段 5：PCB 设计（pcb/，立创 EDA）

- ESP32-S3 模组与 TFT 屏通过排针/排母插装
- 接口：舵机 ×N（PWM + 独立 5V）、I2S 麦克风、功放+扬声器、与 RDK X5 的 UART/USB
- 供电：5V 输入，板载 3.3V LDO；舵机电源与逻辑电源分离

**验收**：DRC 通过，生成 Gerber 可下单打样。

## 阶段 6：3D 建模（mechanical/，Fusion 360）

- RDK X5 支架、PCB（含 ESP32-S3+TFT）支架、激光雷达支架（雷达需 360° 无遮挡且尽量水平）
- 麦克风、扬声器、舵机安装固定件
- 整机布局：重心居中偏低，利于平衡

**验收**：打印装配无干涉，整车可正常起立平衡。

## 阶段 7：系统联调

- 三层数据链贯通：esp-claw 语音 →（OpenClaw）→ Nav2 → 底盘
- 整机演示脚本与视频

**验收**：语音下达指令 → 表情反馈 → 自主导航到目标教室。
