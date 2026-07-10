# 参考资料

## 朋友的同款小车项目（重要：硬件相同，参数可直接参考）

### [lxbme/arduino-balance-car](https://github.com/lxbme/arduino-balance-car)

MiniBalance（平衡小车之家）UNO 版改造，**同款小车套件**。可直接复用：

- **实测引脚映射**（已同步到 hardware.md）：IN1=12/IN2=13（左电机）、IN3=7/IN4=6（右电机）、
  PWMA=10/PWMB=9、左编码器 A=D2+方向=D5、右编码器 A=D4+方向=D8、按键=D3、电池=A0
- **调参初值**：BALANCE_KP=15.0、KD=0.4；VELOCITY_KP=2.0、KI=0.01；机械中值 TARGET_ANGLE=-2.3°
- **卡尔曼参数**：dt=0.005、Q_angle=0.001、Q_gyro=0.005、R_angle=0.5、C_0=1、K1=0.05
- **其他**：5ms 控制环（MsTimer2）、PWM 限幅 250、倾角±40°/欠压 10V 停机、
  电池换算系数 0.05371、拿起/放下检测逻辑
- **已知缺陷（我们要改进）**：右编码器被禁用（单编码器模式），里程计精度不足以导航——
  需用 PCINT（PinChangeInt）给 D4 加中断实现双编码器
- 依赖库随仓库 `libs/` 提供：MsTimer2、PinChangeInt、KalmanFilter、I2Cdev、MPU6050

### [lxbme/esp32_sensor_bridge](https://github.com/lxbme/esp32_sensor_bridge)

ESP32 + **LD08 激光雷达** + micro-ROS（Wi-Fi 发布 /scan，10Hz）。价值：

- **确认雷达型号为 LD08**（UART 115200，DTOF，量程 12m）——含完整驱动 `ld08_driver.cpp`（CRC+解析），阶段 2 可移植
- 朋友的整体架构是「UNO →I2C(0x55, 30 字节包/50ms)→ ESP32 →micro-ROS→ ROS2」；
  我们按任务书要求改为「UNO →UART→ RDK X5」直连，雷达也直接接 RDK X5，不走 ESP32 中转

## 老师提供的链接

### 导航层（阶段 2–3）

- **[RDK X5 TROS OpenClaw skill](https://www.toolify.ai/openclaw-skills/rdk-x5-tros-46486)**：
  `npx clawhub@latest install rdk-x5-tros`，让 OpenClaw 管理 TROS 环境
  （`source /opt/tros/humble/setup.bash`），自带 43 个预装算法包（YOLO、MobileSAM、边缘 LLM、MIPI/USB 相机）
- 注意：RDK X5 用 **TROS（TogetheROS.Bot）Humble**，环境在 `/opt/tros/humble/`，与标准 ROS2 Humble 兼容

### 交互层（阶段 4）

- **[esp-claw 中文教程](https://esp-claw.com/zh-cn/tutorial/)**：路径为
  面包板组装（BOM）→ **浏览器一键烧录**（esp-claw.com 在线工具，无需本地编译）→
  Web 控制台配置 → Skills Lab 开发 Lua skills → FAQ。进阶再走源码编译（ESP-IDF）

### PCB（阶段 5）

- **[立创EDA MCP Bridge](https://jlc-ext.com/item/lion0503/jlceda-mcp-bridge)**：
  嘉立创EDA 扩展管理器装「MCP Bridge」+ VS Code/Cursor 装「JLCEDA MCP Hub」，
  AI 可读原理图语义快照、审网表/DRC、引导器件选型与放置（电源/地符号需手动）
- **[Codex+EasyEDA Bridge 实战文章](https://jishuzhan.net/article/2061968042455805953)**：
  Codex→MCP→EasyEDA Bridge→嘉立创EDA Pro 全链路，案例生成整板且 DRC 0 错误

### 3D 建模（阶段 6）

- **[cad-modeling-fusion-skill](https://github.com/Foster1202/cad-modeling-fusion-skill/blob/main/README.zh.md)**：
  自然语言→Fusion CAD 脚本，装到 `claude-skills/`（Claude Code）或 `$CODEX_HOME/skills/`（Codex），需 Python 3.10+
- **[openclaw-fusion](https://github.com/pacificmeister/openclaw-fusion)**：
  OpenClaw 的 Fusion 360 skill，经 AuraFriday MCP-Link + Fusion MCP Add-in 连接，
  特色是**闭环视觉验证**（每步操作截图确认，不符则撤销重做），全本地运行

## 待补充

- [ ] TFT 屏驱动 IC 数据手册（对照 esp-claw 支持列表）
- [ ] 舵机数量/型号、麦克风类型（I2S/模拟）
