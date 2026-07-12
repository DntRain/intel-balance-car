/****************************************************************************
   简化版平衡小车代码 - 最终工作版本
   基于：Minibalance For Arduino
   
   主要修改：
   1. 修复MPU6050初始化问题（手动配置寄存器代替initialize()）
   2. 移除上位机和蓝牙交互
   3. 使用单编码器模式（左编码器）
   4. 串口调试输出（9600波特率）
   5. 添加I2C主机功能，向ESP32发送编码器和MPU6050数据
   
   硬件要求：
   - Arduino Uno (I2C主机)
   - ESP32 (I2C从机，地址0x55)
   - MPU6050 (I2C: A4/SDA, A5/SCL)
   - TB6612FNG电机驱动
   - 左编码器 (Pin 2 + Pin 5)
   - 按键 (Pin 3)
   
   I2C连接：
   - Arduino SDA (A4) <--> ESP32 SDA
   - Arduino SCL (A5) <--> ESP32 SCL
   - 共地 GND <--> GND
****************************************************************************/
// ============================================================================
// [复刻说明] 本文件 = lxbme/arduino-balance-car Minibalance_for_Arduino_Final.ino
// 逐字复制，含以下标注为 [PATCH] 的改动：
//   P0 编译必需：.ino→.cpp 需要 Arduino.h 和 READ_ENCODER_L 前置声明
//   P1 本车实测：D3 按键悬空，需内部上拉（基线板有外部上拉）
//   P2 本车实测：机械中值（基线车 +2.3° 不适用本车）
//   P3 本车实测：编码器判向公式在本车前进=负计数，需反相（2026-07-10 手转校准）
//   P5 电机死区补偿（见 Set_Pwm）
//   P6 恢复双编码器：D4 用原生 PCINT2（极性照搬自有固件 2026-07-10 手转校准）
//   P7 串口调参：控制参数改为运行时变量，loop 中解析单字母命令（见 loop 前注释）
// ============================================================================
#include <Arduino.h>          // [PATCH P0]
void READ_ENCODER_L();        // [PATCH P0]
void Print_Params();          // [PATCH P7] setup 中调用，定义在文件尾部
#include <avr/wdt.h>  // 看门狗
#include <Wire.h>
// [PATCH P6] 移除 PinChangeInt.h：该库会占用 PCINT2 中断向量，与下面手写的
// ISR(PCINT2_vect) 冲突；右编码器改用原生 PCINT，无需此库
#include <MsTimer2.h>
#include <KalmanFilter.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// ========== 硬件引脚定义 ==========
#define KEY 3
#define IN1 12
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9
#define ENCODER_L 2
#define DIRECTION_L 5
#define ENCODER_R 4
#define DIRECTION_R 8

// ========== 控制参数 ==========
// [PATCH P7] 全部改为运行时变量，可经串口在线调整（见 loop 中的命令解析）。
// 下面的初值 = 截至 2026-07-11 的实测收敛值，调参历史见 git log。
// [PATCH P2] 注意符号：平衡角 = -target_angle（基线用 angle+TARGET 求偏差）
float target_angle = -0.4;  // 命令 a<平衡角> 会自动取负存入；2026-07-12 在线调参：平衡角 +0.4°
#define DIFFERENCE 2
float balance_kp  = 21.0;    // 命令 p<值>
float balance_kd  = 0.9;     // 命令 d<值>
float velocity_kp = 2.5;     // 命令 v<值>，2026-07-12 手推验证：抗拒感明显
float velocity_ki = 0.0095;  // 命令 i<值>
int   dead_zone   = 0;       // 命令 z<值>，[PATCH P5] 死区补偿，2026-07-12 调参置 0

// ========== 卡尔曼滤波参数 ==========
#define K1 0.05
#define Q_ANGLE 0.001
#define Q_GYRO 0.005
#define R_ANGLE 0.5
#define C_0 1.0
#define DT 0.005

// ========== I2C通信参数 ==========
#define ESP32_I2C_ADDR 0x55  // ESP32从机地址

// I2C数据包结构（30字节，紧凑排列）
struct I2C_Data_Packet {
  // 帧头和校验
  uint8_t header;           // 帧头：0xAA
  uint8_t data_type;        // 数据类型：1=编码器+IMU
  
  // 编码器数据（8字节）
  int32_t encoder_left;     // 左轮编码器计数
  int32_t encoder_right;    // 右轮编码器计数
  
  // IMU数据（18字节）
  int16_t accel_x;          // 加速度X
  int16_t accel_y;          // 加速度Y
  int16_t accel_z;          // 加速度Z
  int16_t gyro_x;           // 角速度X
  int16_t gyro_y;           // 角速度Y
  int16_t gyro_z;           // 角速度Z
  float angle;              // 卡尔曼滤波后的角度
  
  uint8_t checksum;         // 校验和
} __attribute__((packed));  // 总计30字节，紧凑排列无填充

// ========== 全局对象 ==========
MPU6050 Mpu6050;
KalmanFilter KalFilter;

// ========== 全局变量 ==========
int16_t ax, ay, az, gx, gy, gz;
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
int Motor1, Motor2;
float Battery_Voltage;
volatile long Velocity_L = 0, Velocity_R = 0;
int Velocity_Left = 0, Velocity_Right = 0;
int Angle;
unsigned char Flag_Stop = 1;

// I2C通信变量
I2C_Data_Packet i2c_packet;
volatile long Encoder_Left_Total = 0;   // 累计编码器计数
volatile long Encoder_Right_Total = 0;  // 累计编码器计数

/**************************************************************************
函数功能：计算校验和
**************************************************************************/
uint8_t Calculate_Checksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

/**************************************************************************
函数功能：向ESP32发送I2C数据
**************************************************************************/
void Send_Data_To_ESP32() {
  static unsigned int send_count = 0;
  
  // 准备数据包
  i2c_packet.header = 0xAA;
  i2c_packet.data_type = 0x01;  // 同时发送编码器和IMU数据
  
  // 编码器数据（累计值）
  i2c_packet.encoder_left = Encoder_Left_Total;
  i2c_packet.encoder_right = Encoder_Right_Total;
  
  // IMU原始数据
  i2c_packet.accel_x = ax;
  i2c_packet.accel_y = ay;
  i2c_packet.accel_z = az;
  i2c_packet.gyro_x = gx;
  i2c_packet.gyro_y = gy;
  i2c_packet.gyro_z = gz;
  
  // 卡尔曼滤波后的角度
  i2c_packet.angle = KalFilter.angle;
  
  // 计算校验和（不包括checksum字段本身，即前29字节）
  i2c_packet.checksum = Calculate_Checksum((uint8_t*)&i2c_packet, sizeof(I2C_Data_Packet) - 1);
  
  // 通过I2C发送数据到ESP32
  Wire.beginTransmission(ESP32_I2C_ADDR);
  
  // 逐字节发送数据包
  uint8_t* data_ptr = (uint8_t*)&i2c_packet;
  for (int i = 0; i < sizeof(I2C_Data_Packet); i++) {
    Wire.write(data_ptr[i]);
  }
  
  byte error = Wire.endTransmission();
  
  // 每100次发送输出一次调试信息（50ms*100=5秒）
  if (++send_count >= 100) {
    Serial.print("[I2C] Sent ");
    Serial.print(sizeof(I2C_Data_Packet));
    Serial.print(" bytes to 0x");
    Serial.print(ESP32_I2C_ADDR, HEX);
    Serial.print(" result=");
    Serial.print(error);
    Serial.print(" (0=OK) EncL=");
    Serial.print(Encoder_Left_Total);
    Serial.print(" EncR=");
    Serial.println(Encoder_Right_Total);
    send_count = 0;
  }
}

/**************************************************************************
函数功能：检测小车是否被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count0, count1, count2;
  
  if (flag == 0) {
    if (abs(encoder_left) + abs(encoder_right) < 15) count0++;
    else count0 = 0;
    if (count0 > 10) flag = 1, count0 = 0;
  }
  
  if (flag == 1) {
    if (++count1 > 400) count1 = 0, flag = 0;
    if (Acceleration > 27000 && (Angle > (-14 + target_angle)) && (Angle < (14 + target_angle))) flag = 2;
  }
  
  if (flag == 2) {
    if (++count2 > 200) count2 = 0, flag = 0;
    if (abs(encoder_left + encoder_right) > 300) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
函数功能：检测小车是否被放下
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right) {
  static unsigned int flag, count;
  
  if (Flag_Stop == 0) return 0;
  
  if (flag == 0) {
    if (Angle > (-10 + target_angle) && Angle < (10 + target_angle) && 
        encoder_left == 0 && encoder_right == 0) flag = 1;
  }
  
  if (flag == 1) {
    if (++count > 100) count = 0, flag = 0;
    if (encoder_left > 12 && encoder_right > 12 && 
        encoder_left < 80 && encoder_right < 80) {
      flag = 0;
      return 1;
    }
  }
  return 0;
}

/**************************************************************************
函数功能：异常关闭电机
**************************************************************************/
unsigned char Turn_Off(float angle, float voltage) {
  unsigned char temp;
  
  if (angle < -40 || angle > 40 || 1 == Flag_Stop || voltage < 10) {
    temp = 1;
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
  else temp = 0;
  
  return temp;
}

/**************************************************************************
函数功能：按键扫描
**************************************************************************/
unsigned char My_click(void) {
  static unsigned char flag_key = 1;
  unsigned char Key;
  
  Key = digitalRead(KEY);
  if (flag_key && Key == 0) {
    flag_key = 0;
    return 1;
  }
  else if (1 == Key) flag_key = 1;
  
  return 0;
}

/**************************************************************************
函数功能：直立PD控制
**************************************************************************/
int balance(float Angle, float Gyro) {
  float Bias;
  int balance;
  
  Bias = Angle - 0;
  balance = balance_kp * Bias + Gyro * balance_kd;
  
  return balance;
}

/**************************************************************************
函数功能：速度PI控制
**************************************************************************/
int velocity(int encoder_left, int encoder_right) {
  static float Velocity, Encoder_Least, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
  
  Movement = 0;
  if (Encoder_Integral > 300) Encoder_Integral -= 200;
  if (Encoder_Integral < -300) Encoder_Integral += 200;
  
  Encoder_Least = (encoder_left + encoder_right) - 0;
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3;
  Encoder_Integral += Encoder;
  Encoder_Integral = Encoder_Integral - Movement;
  
  if (Encoder_Integral > 21000) Encoder_Integral = 21000;
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;
  
  Velocity = Encoder * velocity_kp + Encoder_Integral * velocity_ki;
  
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1) 
    Encoder_Integral = 0;
  
  return Velocity;
}

/**************************************************************************
函数功能：转向控制
**************************************************************************/
int turn(float gyro) {
  return 0;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
**************************************************************************/
// [PATCH P5] 死区补偿：PWM 非零即加偏置跳过电机静摩擦死区，
// 解决"小角度倾斜时输出落在死区内、轮子不动、白白倾倒"的问题
// 用 dead_zone 变量（串口命令 z 在线可调）；20 时过零跳变引发高频抖
void Set_Pwm(int moto1, int moto2) {
  if (moto1 > 0) moto1 += dead_zone;
  else if (moto1 < 0) moto1 -= dead_zone;
  if (moto2 > 0) moto2 += dead_zone;
  else if (moto2 < 0) moto2 -= dead_zone;

  if (moto1 > 0) digitalWrite(IN1, HIGH), digitalWrite(IN2, LOW);
  else digitalWrite(IN1, LOW), digitalWrite(IN2, HIGH);
  analogWrite(PWMA, min(abs(moto1), 255));

  if (moto2 < 0) digitalWrite(IN3, HIGH), digitalWrite(IN4, LOW);
  else digitalWrite(IN3, LOW), digitalWrite(IN4, HIGH);
  analogWrite(PWMB, min(abs(moto2), 255));
}

/**************************************************************************
函数功能：限制PWM赋值
**************************************************************************/
void Xianfu_Pwm(void) {
  int Amplitude = 250;
  
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude) Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude) Motor2 = Amplitude;
}

/**************************************************************************
函数功能：5ms控制函数 - 核心代码
**************************************************************************/
void control() {
  static int Velocity_Count, Turn_Count, I2C_Count;
  static float Voltage_All, Voltage_Count;
  int Temp;
  
  sei();
  
  // 获取MPU6050数据
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // 卡尔曼滤波获取角度
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, DT, Q_ANGLE, Q_GYRO, R_ANGLE, C_0, K1);
  Angle = KalFilter.angle;
  
  // 直立PD控制，控制周期5ms
  Balance_Pwm = balance(KalFilter.angle + target_angle, KalFilter.Gyro_x);
  
  // 速度控制，控制周期40ms
  if (++Velocity_Count >= 8) {
    Velocity_Left = Velocity_L;      // [PATCH P6] 恢复双编码器独立采样
    Velocity_L = 0;
    Velocity_Right = Velocity_R;
    Velocity_R = 0;
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);
    Velocity_Count = 0;
  }
  
  // 转向控制，控制周期20ms
  if (++Turn_Count >= 4) {
    Turn_Pwm = turn(gz);
    Turn_Count = 0;
  }
  
  // 三环叠加
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  Xianfu_Pwm();
  
  // 检测拿起和放下
  if (Pick_Up(az, KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 1;
  if (Put_Down(KalFilter.angle, Velocity_Left, Velocity_Right)) Flag_Stop = 0;
  
  // 电机控制
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 0) Set_Pwm(Motor1, Motor2);
  
  // 按键控制
  if (My_click()) Flag_Stop = !Flag_Stop;
  
  // 电池电压采样
  Temp = analogRead(0);
  Voltage_Count++;
  Voltage_All += Temp;
  if (Voltage_Count == 200) {
    // （系数验证过是对的：电池实际接通时 A0 均值 229 × 0.05371 = 12.3V 吻合满电 3S；
    //  之前读到 4.14V 是电池开关未真正接通时的残余电压，勿再怀疑分压比）
    Battery_Voltage = Voltage_All * 0.05371 / 200;
    Voltage_All = 0;
    Voltage_Count = 0;
  }
  
  // I2C数据发送，控制周期50ms（每10个5ms周期）
  if (++I2C_Count >= 10) {
    Send_Data_To_ESP32();
    I2C_Count = 0;
  }
}

/**************************************************************************
函数功能：初始化
**************************************************************************/
void setup() {
  // TB6612引脚初始化
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  
  // 编码器引脚初始化
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(8, INPUT);
  
  // 按键引脚初始化
  pinMode(3, INPUT_PULLUP);  // [PATCH P1] 本车 D3 悬空，无外部上拉
  
  // 禁用看门狗（防止复位）
  wdt_disable();
  
  // 串口初始化
  Serial.begin(9600);
  delay(500);
  Serial.println("\n=== MiniBalance Starting ===");
  delay(500);
  
  // I2C初始化
  Serial.print("Init I2C...");
  Wire.begin();
  Wire.setClock(100000L);
  delay(100);
  Serial.println("OK");
  Serial.print("I2C packet size: ");
  Serial.print(sizeof(I2C_Data_Packet));
  Serial.println(" bytes");
  
  // MPU6050手动配置（使用纯Wire库）
  Serial.println("Config MPU6050:");
  Serial.flush();
  
  Serial.print("  Wake up...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  delay(50);
  
  Serial.print("  Set gyro...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  delay(10);
  
  Serial.print("  Set accel...");
  Serial.flush();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  err = Wire.endTransmission();
  Serial.print("err=");
  Serial.println(err);
  Serial.flush();
  delay(10);
  
  Serial.println("MPU OK");
  Serial.flush();
  delay(100);
  
  // 编码器外部中断初始化（先初始化中断）
  Serial.println("Init Encoder");
  Serial.flush();
  
  attachInterrupt(0, READ_ENCODER_L, CHANGE);
  Serial.println("Encoder L OK");
  Serial.flush();
  delay(50);

  // [PATCH P6] 右编码器：D4 = PCINT20，用原生 Pin Change 中断（不依赖 PinChangeInt 库）
  PCICR |= _BV(PCIE2);      // 使能 PCINT2 组（D0~D7）
  PCMSK2 = _BV(PCINT20);    // 只放行 D4，避免同组其它引脚误触发
  Serial.println("Encoder R OK (dual encoder mode)");
  Serial.flush();
  delay(50);
  
  // 定时中断初始化（最后启动定时器）
  Serial.println("Init Timer");
  Serial.flush();
  delay(50);
  
  MsTimer2::set(5, control);
  Serial.println("Timer set");
  Serial.flush();
  delay(50);
  
  MsTimer2::start();
  Serial.println("Timer started");
  Serial.flush();
  delay(100);
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Mode: Dual encoder");
  Serial.println("Press KEY (Pin 3) to start/stop");
  Serial.println("Tune: p/d=balance v/i=velocity a=angle z=deadzone ?=show g=go x=stop");
  Print_Params();
  Serial.flush();
  delay(100);
}

/**************************************************************************
[PATCH P7] 串口调参命令（9600 波特率，一行一条，\n 结尾）：
  p<值>  直立环 KP        例 p36
  d<值>  直立环 KD        例 d0.9
  v<值>  速度环 KP        例 v1.5
  i<值>  速度环 KI        例 i0.0075
  a<值>  平衡角（°，与遥测 Ang 同符号，内部自动取负存 target_angle） 例 a0.2
  z<值>  死区补偿 PWM     例 z12
  ?      打印当前全部参数
  g      启动电机（Flag_Stop=0）   x  停止电机（Flag_Stop=1）
参数被 5ms 定时中断读取，写入时短暂关中断防止 float 撕裂。
**************************************************************************/
void Print_Params() {
  Serial.print("[PARAM] KP=");   Serial.print(balance_kp);
  Serial.print(" KD=");          Serial.print(balance_kd);
  Serial.print(" VKP=");         Serial.print(velocity_kp);
  Serial.print(" VKI=");         Serial.print(velocity_ki, 4);
  Serial.print(" BAL_ANG=");     Serial.print(-target_angle);
  Serial.print(" DZ=");          Serial.print(dead_zone);
  Serial.print(" Stop=");        Serial.println(Flag_Stop);
}

void Handle_Command() {
  static char buf[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      continue;
    }
    if (len == 0) continue;
    buf[len] = '\0';
    len = 0;

    char cmd = buf[0];
    float val = atof(buf + 1);
    bool ok = true;

    noInterrupts();
    switch (cmd) {
      case 'p': balance_kp   = val;  break;
      case 'd': balance_kd   = val;  break;
      case 'v': velocity_kp  = val;  break;
      case 'i': velocity_ki  = val;  break;
      case 'a': target_angle = -val; break;  // 用户给平衡角，存偏置
      case 'z': dead_zone    = (int)val; break;
      case 'g': Flag_Stop = 0; break;
      case 'x': Flag_Stop = 1; break;
      case '?': break;
      default:  ok = false; break;
    }
    interrupts();

    if (ok) Print_Params();
    else { Serial.print("[ERR] unknown cmd: "); Serial.println(buf); }
  }
}

/**************************************************************************
函数功能：主循环
**************************************************************************/
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  Handle_Command();  // [PATCH P7] 串口调参

  // 每200ms输出一次状态
  if (currentTime - lastTime >= 200) {
    Serial.print("Ang:");
    Serial.print(Angle);
    Serial.print(" VL:");
    Serial.print(Velocity_Left);
    Serial.print(" VR:");
    Serial.print(Velocity_Right);
    Serial.print(" Bat:");
    Serial.print(Battery_Voltage);
    Serial.print("V PWM:");
    Serial.print(Motor1);
    Serial.print(",");
    Serial.print(Motor2);
    Serial.print(" Stop:");
    Serial.println(Flag_Stop);
    
    lastTime = currentTime;
  }
}

/**************************************************************************
函数功能：左编码器中断
**************************************************************************/
void READ_ENCODER_L() {
  // [PATCH P3] 判向整体反相：基线公式在本车上前进=负计数，速度环符号会反
  if (digitalRead(ENCODER_L) == LOW) {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L++;
      Encoder_Left_Total++;
    }
    else {
      Velocity_L--;
      Encoder_Left_Total--;
    }
  }
  else {
    if (digitalRead(DIRECTION_L) == LOW) {
      Velocity_L--;
      Encoder_Left_Total--;
    }
    else {
      Velocity_L++;
      Encoder_Left_Total++;
    }
  }
}

/**************************************************************************
函数功能：右编码器中断
[PATCH P6] D4 = PCINT20，原生 Pin Change 中断。判向极性照搬自有固件
2026-07-10 手转校准结果（a^b 为负、否则为正 → 前进=正计数），
与左编码器 P3 反相后的约定一致。
**************************************************************************/
ISR(PCINT2_vect) {
  bool a = PIND & _BV(PD4);
  bool b = PINB & _BV(PB0);   // D8 = PB0
  if (a ^ b) {
    Velocity_R--;
    Encoder_Right_Total--;
  } else {
    Velocity_R++;
    Encoder_Right_Total++;
  }
}
