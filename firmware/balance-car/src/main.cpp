// 平衡小车固件骨架 —— 阶段 1 实现
// 引脚分配见 docs/hardware.md；串口协议见 docs/serial-protocol.md
#include <Arduino.h>

// ---- 引脚定义（与 docs/hardware.md 保持一致）----
constexpr uint8_t PIN_ENC_L_A = 2;   // INT0
constexpr uint8_t PIN_ENC_R_A = 3;   // INT1
constexpr uint8_t PIN_ENC_L_B = 4;
constexpr uint8_t PIN_ENC_R_B = 5;
constexpr uint8_t PIN_STBY    = 6;
constexpr uint8_t PIN_AIN1    = 7;
constexpr uint8_t PIN_AIN2    = 8;
constexpr uint8_t PIN_PWMA    = 9;
constexpr uint8_t PIN_PWMB    = 10;
constexpr uint8_t PIN_BIN1    = 12;
constexpr uint8_t PIN_BIN2    = 13;
constexpr uint8_t PIN_VBAT    = A0;

// ---- 里程计常量（唯一权威定义在 docs/serial-protocol.md，改动需同步）----
constexpr float    WHEEL_DIAMETER_MM     = 67.0f;   // ⚠️ 待实测
constexpr float    WHEEL_TRACK_MM        = 170.0f;  // ⚠️ 待实测
constexpr uint16_t COUNTS_PER_WHEEL_REV  = 1320;    // 11 线 × 4 倍频 × 减速比 30

// ---- 控制周期 ----
constexpr uint16_t CONTROL_HZ   = 200;  // 平衡控制环
constexpr uint16_t TELEMETRY_HZ = 50;   // 串口遥测上报
constexpr uint16_t CMD_TIMEOUT_MS = 500; // 超时未收到指令 → 目标速度清零

void setup() {
  Serial.begin(115200);
  // TODO(阶段1): 初始化引脚、I2C、MPU6050、编码器中断
}

void loop() {
  // TODO(阶段1): 200Hz 定时执行以下串级控制
  //   1. MPU6050 读取 + 互补滤波 → pitch / gyro
  //   2. 速度环 PI：编码器均速 vs 目标 v → 目标倾角偏置
  //   3. 直立环 PD：pitch vs 目标倾角 → 基础 PWM
  //   4. 转向环 PD：gyro_z vs 目标 ω → 差动 PWM
  //   5. PWM 输出到 TB6612（倾角 > 安全阈值时停机保护）
  // TODO(阶段1): 50Hz 组遥测帧上报；解析下行 cmd_vel 帧（带超时保护）
}
