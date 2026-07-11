/****************************************************************************
  平衡小车固件 —— 阶段 1
  基线：lxbme/arduino-balance-car（MiniBalance 同款套件实测版本）
  改造（见 docs/roadmap.md 阶段 1）：
    1. 启用右编码器（D4 原生 PCINT，不用 PinChangeInt 库）
    2. 实现转向环（基线的 turn() 是空函数）
    3. I2C→ESP32 改为 UART→RDK X5，按 docs/serial-protocol.md v1.0
    4. 接收下行 cmd_vel 帧，500ms 超时自动清零
  MPU6050 直接用 Wire 读寄存器（不依赖 I2Cdev/MPU6050 大库）。
****************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <KalmanFilter.h>

// 置 1 时输出人类可读调试信息、关闭二进制遥测（两者共用串口，不能同时开）
#define DEBUG_PRINT 0

// ================= 引脚（MiniBalance 套件固定接线） =================
#define PIN_KEY     3
#define PIN_IN1     12   // 左电机方向
#define PIN_IN2     13
#define PIN_IN3     7    // 右电机方向
#define PIN_IN4     6
#define PIN_PWMA    10   // 左电机 PWM
#define PIN_PWMB    9    // 右电机 PWM
// ⚠️ 左右归属与文档初版相反：2026-07-10 单轮手转实测，D2 编码器在右轮、D4 在左轮
#define PIN_ENC_R_A 2    // INT0
#define PIN_ENC_R_B 5
#define PIN_ENC_L_A 4    // PCINT20
#define PIN_ENC_L_B 8
#define PIN_VBAT    A0

// ================= 里程计常量（权威定义：docs/serial-protocol.md） =================
constexpr float    WHEEL_DIAMETER_MM    = 67.0f;   // ⚠️ 待实测
constexpr float    WHEEL_TRACK_MM       = 170.0f;  // ⚠️ 待实测
constexpr uint16_t COUNTS_PER_WHEEL_REV = 660;     // 11 线 × 2 倍频 × 减速比 30
constexpr float    MM_PER_COUNT = PI * WHEEL_DIAMETER_MM / COUNTS_PER_WHEEL_REV; // ≈0.319

// ================= 控制参数（初值来自基线项目，本车微调） =================
constexpr float TARGET_ANGLE  = -0.5f;  // 机械中值二分逼近：+1.3 前倒 / -1.2 后倒（2026-07-11）
// 回归基线验证值（朋友同款车实测能站）：此前"软倒/狂抖"的真因是速度环缺积分泄放，
// 不是这些增益；基线单编码器模式=左轮复制右轮再求和，与双编码器求和同尺度，无需减半
constexpr float BALANCE_KP    = 15.0f;
constexpr float BALANCE_KD    = 0.4f;
constexpr float VELOCITY_KP   = 2.0f;
constexpr float VELOCITY_KI   = 0.01f;
constexpr float TURN_KP       = 2.0f;   // 转向环 ⚠️ 待调（基线为空函数）
constexpr float TURN_KD       = 0.1f;
constexpr int   PWM_LIMIT     = 250;
constexpr float ANGLE_PROTECT = 40.0f;  // 倾角保护（°）
constexpr float VBAT_PROTECT  = 10.0f;  // 欠压保护（V）
constexpr float VBAT_COEFF    = 0.05371f;

// 卡尔曼参数（基线实测）
constexpr float K_DT = 0.005f, K_Q_ANGLE = 0.001f, K_Q_GYRO = 0.005f;
constexpr float K_R_ANGLE = 0.5f, K_C0 = 1.0f, K_K1 = 0.05f;

// cmd_vel（mm/s）→ 速度环 Movement（双轮 counts / 40ms）
// counts/s 每轮 = v / MM_PER_COUNT；40ms 双轮 = v / MM_PER_COUNT * 0.04 * 2
constexpr float MMS_TO_MOVEMENT = 0.08f / MM_PER_COUNT;

// ================= 串口协议（docs/serial-protocol.md v1.0） =================
constexpr uint8_t FRAME_H0 = 0xAA, FRAME_H1 = 0x55;
constexpr uint8_t TYPE_TELEMETRY = 0x01;  // 上行，载荷 28 字节
constexpr uint8_t TYPE_CMD_VEL   = 0x10;  // 下行，载荷 4 字节
constexpr uint16_t CMD_TIMEOUT_MS = 500;

struct __attribute__((packed)) Telemetry {
  int16_t  d_enc_l, d_enc_r;      // 编码器增量（自上帧）
  int16_t  speed_l, speed_r;      // mm/s
  int16_t  roll, pitch, yaw;      // 0.01°
  int16_t  gyro_x, gyro_y, gyro_z;// 0.1°/s
  int16_t  acc_x, acc_y, acc_z;   // 0.001g
  uint16_t vbat_mv;
};
static_assert(sizeof(Telemetry) == 28, "telemetry payload must be 28 bytes");

// ================= 全局状态 =================
KalmanFilter kal;
int16_t ax, ay, az, gx, gy, gz;
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
int Motor1, Motor2;
float Battery_Voltage = 12.0f;      // 初值防止开机误触发欠压保护
float yaw_deg = 0;                  // 陀螺积分航向（漂移，仅供参考）
unsigned char Flag_Stop = 1;

// 编码器：Velocity_* 供速度环（40ms 清零），Odom_* 供遥测（每帧清零）
volatile int16_t Velocity_L, Velocity_R;
volatile int16_t Odom_L, Odom_R;
int16_t Velocity_Left, Velocity_Right;  // 速度环采样值

// cmd_vel 目标（loop 写入，控制 ISR 读取；16 位访问需关中断保护）
volatile int16_t target_v_mms = 0;      // mm/s
volatile int16_t target_w_mrads = 0;    // 0.001 rad/s
volatile uint32_t last_cmd_ms = 0;

volatile uint8_t telemetry_due = 0;     // 控制 ISR 置位，loop 发送

// ================= 编码器中断 =================
// 2 倍频：A 相 CHANGE 触发，A^B 判向。直读端口寄存器（ISR 内避免 digitalRead 开销）
// 两路符号均为 2026-07-10 单轮手转实测校准：前进方向计数为正
void readEncoderR() {  // D2/INT0 = 右轮
  bool a = PIND & _BV(PD2), b = PIND & _BV(PD5);
  if (a ^ b) { Velocity_R--; Odom_R--; } else { Velocity_R++; Odom_R++; }
}
ISR(PCINT2_vect) {  // D4 = PCINT20（PCMSK2 只使能该位）= 左轮
  bool a = PIND & _BV(PD4), b = PINB & _BV(PB0);   // D8 = PB0
  if (a ^ b) { Velocity_L--; Odom_L--; } else { Velocity_L++; Odom_L++; }
}

// ================= MPU6050（Wire 直读，无库依赖） =================
static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static void mpuInit() {
  mpuWrite(0x6B, 0x00);  delay(50);  // 唤醒
  mpuWrite(0x1B, 0x00);  delay(10);  // 陀螺 ±250°/s（131 LSB/(°/s)）
  mpuWrite(0x1C, 0x00);  delay(10);  // 加计 ±2g（16384 LSB/g）
  // DLPF 保持默认关闭（与基线一致）：0x1A=0x03 的 44Hz 滤波给陀螺加 ~4.8ms 延迟，
  // 直立环 KD 项晚一整拍，实测削弱阻尼；振动噪声交给卡尔曼滤波处理
}
static void mpuRead() {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();          // 跳过温度
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// ================= 控制律（结构沿用基线） =================
static int balance(float angle, float gyro) {
  return BALANCE_KP * angle + BALANCE_KD * gyro;
}

static int velocity(int enc_left, int enc_right, float movement) {
  static float Encoder, Encoder_Integral;
  // 积分泄放（基线原码关键两行，初版移植时遗漏）：没有它积分会漂到 ±21000
  // （±84 PWM 恒定推力）把静止的车慢慢拱倒，方向随漂移历史——"两边都倒"的元凶
  if (Encoder_Integral >  300) Encoder_Integral -= 200;
  if (Encoder_Integral < -300) Encoder_Integral += 200;
  float least = (enc_left + enc_right) - 0;
  Encoder = Encoder * 0.7f + least * 0.3f;         // 低通
  Encoder_Integral += Encoder;
  Encoder_Integral -= movement;                    // 目标速度以积分偏置注入
  if (Encoder_Integral >  21000) Encoder_Integral =  21000;
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;
  float out = Encoder * VELOCITY_KP + Encoder_Integral * VELOCITY_KI;
  // 与基线一致：停机、倒下、欠压任一条件都清零积分，防止扶起瞬间带残留推力
  if (Flag_Stop || kal.angle < -ANGLE_PROTECT || kal.angle > ANGLE_PROTECT ||
      Battery_Voltage < VBAT_PROTECT) Encoder_Integral = 0;
  return out;
}

static int turn(float target_w_dps, float gyro_z_dps) {
  // 前馈 + 角速度反馈 PD（基线为空函数，此为新增）
  return TURN_KP * target_w_dps + TURN_KD * (target_w_dps - gyro_z_dps) * 10.0f;
}

static void setPwm(int moto1, int moto2) {  // 极性沿用基线（电机镜像安装）
  if (moto1 > 0) { digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW); }
  else           { digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH); }
  analogWrite(PIN_PWMA, abs(moto1));
  if (moto2 < 0) { digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); }
  else           { digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); }
  analogWrite(PIN_PWMB, abs(moto2));
}

static bool turnOff(float angle, float voltage) {
  if (angle < -ANGLE_PROTECT || angle > ANGLE_PROTECT || Flag_Stop || voltage < VBAT_PROTECT) {
    analogWrite(PIN_PWMA, 0); analogWrite(PIN_PWMB, 0);
    return true;
  }
  return false;
}

// 拿起/放下检测（移植自基线）
static int pickUp(float acc_z, float angle, int enc_l, int enc_r) {
  static unsigned int flag, c0, c1, c2;
  if (flag == 0) {
    if (abs(enc_l) + abs(enc_r) < 15) c0++; else c0 = 0;
    if (c0 > 10) { flag = 1; c0 = 0; }
  }
  if (flag == 1) {
    if (++c1 > 400) { c1 = 0; flag = 0; }
    if (acc_z > 27000 && angle > (-14 + TARGET_ANGLE) && angle < (14 + TARGET_ANGLE)) flag = 2;
  }
  if (flag == 2) {
    if (++c2 > 200) { c2 = 0; flag = 0; }
    if (abs(enc_l + enc_r) > 300) { flag = 0; return 1; }
  }
  return 0;
}
static int putDown(float angle, int enc_l, int enc_r) {
  static unsigned int flag, count;
  if (!Flag_Stop) return 0;
  if (flag == 0) {
    if (angle > (-10 + TARGET_ANGLE) && angle < (10 + TARGET_ANGLE) && enc_l == 0 && enc_r == 0) flag = 1;
  }
  if (flag == 1) {
    if (++count > 100) { count = 0; flag = 0; }
    if (enc_l > 12 && enc_r > 12 && enc_l < 80 && enc_r < 80) { flag = 0; return 1; }
  }
  return 0;
}

static bool keyClick() {
  static bool armed = true;
  bool k = digitalRead(PIN_KEY);
  if (armed && k == 0) { armed = false; return true; }
  if (k == 1) armed = true;
  return false;
}

// ================= 5ms 控制中断 =================
void control() {
  static int velocity_cnt, turn_cnt, telemetry_cnt;
  static float voltage_sum; static int voltage_cnt;
  static float movement = 0, target_w_dps = 0;

  sei();  // 允许编码器/串口中断嵌套（I2C 读约 0.5ms @400kHz）

  mpuRead();
  kal.Angletest(ax, ay, az, gx, gy, gz, K_DT, K_Q_ANGLE, K_Q_GYRO, K_R_ANGLE, K_C0, K_K1);
  yaw_deg += kal.Gyro_z * K_DT;

  // 指令目标（16 位原子读取 + 超时保护）
  cli();
  int16_t tv = target_v_mms, tw = target_w_mrads;
  uint32_t lc = last_cmd_ms;
  sei();
  if (millis() - lc > CMD_TIMEOUT_MS) { tv = 0; tw = 0; }
  movement = tv * MMS_TO_MOVEMENT;
  target_w_dps = tw * 0.0573f;  // 0.001 rad/s → °/s

  // 误差 = 当前角 - 机械中值（TARGET_ANGLE 按直立实测存正值；基线存负值配 + 号，勿混用）
  Balance_Pwm = balance(kal.angle - TARGET_ANGLE, kal.Gyro_x);

  if (++velocity_cnt >= 8) {   // 40ms
    cli(); Velocity_Left = Velocity_L; Velocity_L = 0;
           Velocity_Right = Velocity_R; Velocity_R = 0; sei();
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right, movement);
    velocity_cnt = 0;
  }
  if (++turn_cnt >= 4) {       // 20ms
    Turn_Pwm = turn(target_w_dps, kal.Gyro_z);
    turn_cnt = 0;
  }

  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  Motor1 = constrain(Motor1, -PWM_LIMIT, PWM_LIMIT);
  Motor2 = constrain(Motor2, -PWM_LIMIT, PWM_LIMIT);

  if (pickUp(az, kal.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 1;
  if (putDown(kal.angle, Velocity_Left, Velocity_Right))    Flag_Stop = 0;
  if (!turnOff(kal.angle, Battery_Voltage)) setPwm(Motor1, Motor2);
  if (keyClick()) Flag_Stop = !Flag_Stop;

  voltage_sum += analogRead(PIN_VBAT);
  if (++voltage_cnt >= 200) {
    Battery_Voltage = voltage_sum * VBAT_COEFF / 200;
    voltage_sum = 0; voltage_cnt = 0;
  }

  if (++telemetry_cnt >= 4) {  // 20ms → 50Hz
    telemetry_due = 1;
    telemetry_cnt = 0;
  }
}

// ================= 遥测发送（loop 上下文） =================
static void sendTelemetry() {
  Telemetry t;
  cli();
  t.d_enc_l = Odom_L; Odom_L = 0;
  t.d_enc_r = Odom_R; Odom_R = 0;
  sei();
  // 50Hz 帧周期 20ms：speed = delta * MM_PER_COUNT / 0.02
  t.speed_l = t.d_enc_l * MM_PER_COUNT * 50.0f;
  t.speed_r = t.d_enc_r * MM_PER_COUNT * 50.0f;
  t.roll  = Flag_Stop;                    // ⚠️ 临时复用：启停状态（1=停机 0=运行），调平衡期间用；roll 原值仅参考无人消费
  t.pitch = kal.angle * 100;
  t.yaw   = yaw_deg * 100;
  t.gyro_x = (int32_t)gx * 10 / 131;      // raw → 0.1°/s
  t.gyro_y = (int32_t)gy * 10 / 131;
  t.gyro_z = (int32_t)gz * 10 / 131;
  t.acc_x = (int32_t)ax * 1000 / 16384;   // raw → 0.001g
  t.acc_y = (int32_t)ay * 1000 / 16384;
  t.acc_z = (int32_t)az * 1000 / 16384;
  t.vbat_mv = Battery_Voltage * 1000;

  uint8_t buf[5 + sizeof(Telemetry)];
  buf[0] = FRAME_H0; buf[1] = FRAME_H1;
  buf[2] = TYPE_TELEMETRY; buf[3] = sizeof(Telemetry);
  memcpy(buf + 4, &t, sizeof(Telemetry));   // AVR 小端，直接拷贝
  uint8_t sum = 0;
  for (uint8_t i = 2; i < 4 + sizeof(Telemetry); i++) sum += buf[i];
  buf[4 + sizeof(Telemetry)] = sum;
  Serial.write(buf, sizeof(buf));
}

// ================= 下行帧解析（loop 上下文，状态机） =================
static void pollCommand() {
  static uint8_t state, type, len, idx, sum;
  static uint8_t payload[8];
  while (Serial.available()) {
    uint8_t c = Serial.read();
    switch (state) {
      case 0: state = (c == FRAME_H0) ? 1 : 0; break;
      case 1: state = (c == FRAME_H1) ? 2 : 0; break;
      case 2: type = c; sum = c; state = 3; break;
      case 3:
        len = c; sum += c; idx = 0;
        state = (type == TYPE_CMD_VEL && len == 4) ? 4 : 0;
        break;
      case 4:
        payload[idx++] = c; sum += c;
        if (idx >= len) state = 5;
        break;
      case 5:
        if (c == sum) {
          int16_t v = payload[0] | (payload[1] << 8);
          int16_t w = payload[2] | (payload[3] << 8);
          cli();
          target_v_mms = v; target_w_mrads = w;
          last_cmd_ms = millis();
          sei();
        }
        state = 0;
        break;
    }
  }
}

// ================= 初始化与主循环 =================
void setup() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  setPwm(0, 0);
  pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
  pinMode(PIN_KEY, INPUT_PULLUP);  // 按键对地，开内部上拉防悬空误读

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);  // 无 I2C 从机，可用 400kHz（基线因 ESP32 从机限 100k）
  mpuInit();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), readEncoderR, CHANGE);
  PCICR  |= _BV(PCIE2);     // 使能端口 D 的 PCINT 组
  PCMSK2 |= _BV(PCINT20);   // 仅 D4

  MsTimer2::set(5, control);
  MsTimer2::start();

#if DEBUG_PRINT
  Serial.println(F("\n=== Balance Car (Phase 1) READY, DEBUG mode ==="));
#endif
}

void loop() {
  pollCommand();
  if (telemetry_due) {
    telemetry_due = 0;
#if DEBUG_PRINT
    static uint8_t n;
    if (++n >= 10) {  // 5Hz 人类可读输出
      n = 0;
      Serial.print(F("Ang:"));  Serial.print(kal.angle);
      Serial.print(F(" VL:"));  Serial.print(Velocity_Left);
      Serial.print(F(" VR:"));  Serial.print(Velocity_Right);
      Serial.print(F(" Bat:")); Serial.print(Battery_Voltage);
      Serial.print(F(" PWM:")); Serial.print(Motor1);
      Serial.print(F(","));     Serial.print(Motor2);
      Serial.print(F(" Stop:"));Serial.println(Flag_Stop);
    }
#else
    sendTelemetry();
#endif
  }
}
