// 平衡小车固件骨架 —— 阶段 1 实现
// 引脚映射与调参初值取自同款小车的实测项目 lxbme/arduino-balance-car（见 docs/references.md）
// 引脚表见 docs/hardware.md；串口协议见 docs/serial-protocol.md
#include <Arduino.h>

// ---- 引脚定义（MiniBalance 套件固定接线，勿改）----
constexpr uint8_t PIN_ENC_L_A  = 2;   // INT0，CHANGE 触发
constexpr uint8_t PIN_ENC_L_B  = 5;   // 方向判读
constexpr uint8_t PIN_ENC_R_A  = 4;   // ⚠️ 需 PCINT（PinChangeInt），朋友项目未启用
constexpr uint8_t PIN_ENC_R_B  = 8;
constexpr uint8_t PIN_KEY      = 3;   // 启停按键
constexpr uint8_t PIN_IN1      = 12;  // 左电机方向
constexpr uint8_t PIN_IN2      = 13;
constexpr uint8_t PIN_IN3      = 7;   // 右电机方向
constexpr uint8_t PIN_IN4      = 6;
constexpr uint8_t PIN_PWMA     = 10;  // 左电机 PWM
constexpr uint8_t PIN_PWMB     = 9;   // 右电机 PWM
constexpr uint8_t PIN_VBAT     = A0;

// ---- 里程计常量（唯一权威定义在 docs/serial-protocol.md，改动需同步）----
constexpr float    WHEEL_DIAMETER_MM    = 67.0f;  // ⚠️ 待实测
constexpr float    WHEEL_TRACK_MM       = 170.0f; // ⚠️ 待实测
constexpr uint16_t COUNTS_PER_WHEEL_REV = 660;    // 11 线 × 2 倍频 × 减速比 30

// ---- 调参初值（朋友项目实测，本车微调）----
constexpr float TARGET_ANGLE   = -2.3f;   // 机械中值，本车需重测
constexpr float BALANCE_KP     = 15.0f;   // 直立环 PD
constexpr float BALANCE_KD     = 0.4f;
constexpr float VELOCITY_KP    = 2.0f;    // 速度环 PI
constexpr float VELOCITY_KI    = 0.01f;
constexpr int   PWM_LIMIT      = 250;     // PWM 限幅
constexpr float ANGLE_PROTECT  = 40.0f;   // 倾角保护阈值（°）
constexpr float VBAT_PROTECT   = 10.0f;   // 欠压保护（V）
constexpr float VBAT_COEFF     = 0.05371f;// A0 均值 → 电压

// ---- 控制周期 ----
constexpr uint8_t  CONTROL_MS     = 5;    // 5ms 控制环（MsTimer2），与朋友项目一致
constexpr uint16_t TELEMETRY_HZ   = 50;   // 串口遥测（每 10 个控制周期）
constexpr uint16_t CMD_TIMEOUT_MS = 500;  // 超时未收到指令 → 目标速度清零

void setup() {
  Serial.begin(115200);  // 协议波特率（不沿用朋友的 9600 调试口）
  // TODO(阶段1): 初始化引脚、I2C、MPU6050（参考朋友项目：手动写寄存器 0x6B/0x1B/0x1C，
  //              不用 initialize()）；D2 attachInterrupt + D4 PinChangeInt 双编码器
  // TODO(阶段1): 卡尔曼参数沿用朋友项目：dt=0.005 Q_angle=0.001 Q_gyro=0.005 R_angle=0.5
}

void loop() {
  // TODO(阶段1): MsTimer2 5ms 定时执行串级控制（结构沿用朋友项目 control()）：
  //   1. MPU6050 读取 + 卡尔曼滤波 → angle / gyro
  //   2. 直立环 PD（5ms）：angle+TARGET_ANGLE、gyro → Balance_Pwm
  //   3. 速度环 PI（40ms）：双轮编码器均速 vs 目标 v → Velocity_Pwm（低通 0.7/0.3 + 积分限幅）
  //   4. 转向环（20ms）：gyro_z vs 目标 ω → Turn_Pwm（朋友项目为空，需要实现）
  //   5. Motor = Balance − Velocity ± Turn，限幅 ±PWM_LIMIT，倾角/欠压保护后输出
  //   6. 拿起/放下检测（Pick_Up/Put_Down 逻辑可直接移植）
  // TODO(阶段1): 50Hz 组遥测帧上报；解析下行 cmd_vel 帧（带 CMD_TIMEOUT_MS 保护）
  //              联调时关闭所有调试打印（与协议共用串口）
}
