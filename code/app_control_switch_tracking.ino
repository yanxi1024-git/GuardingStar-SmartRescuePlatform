#include <Arduino.h>
#include "FastLED.h"
#include <Servo.h>
#include "Ultrasound.h"
#include <Wire.h>

/* ====== 配置区 ====== */
#define SERIAL_DEBUG        0            // 默认关闭调试，避免干扰蓝牙
#define LINE_FOLLOWER_I2C_ADDR 0x78

// A3：按键（你实测空闲≈132、按下≈0）
#define KEY_PIN_ANALOG      A3
#define KEY_ACTIVE_LOW      1
#define KEY_ADC_TH          60
#define KEY_QUIET_MS        150

// 电压采样（与按键同脚A3；仅空闲稳定时采样 + 有效性过滤 + 滤波/限流）
#define VOLTAGE_ENABLED             1
#define VOLTAGE_PIN                 A3
#define VOLTAGE_K                   0.02989f     // 需时可按万用表微调
#define VOLTAGE_RAW_MIN             250          // 原始ADC低于此判为无效样本
#define VOLTAGE_VALID_TIMEOUT_MS    10000        // 超过此时间未得到有效样本则视为无效

// ——【稳定性增强参数】——
#define VOLTAGE_SAMPLES             12           // 每轮采样次数
#define VOLTAGE_TRIM                2            // 去极值个数（两端各丢 TRIM 个）
#define VOLTAGE_IIR_NUM             1            // IIR 系数：alpha = NUM/DEN（默认 1/4）
#define VOLTAGE_IIR_DEN             4
#define VOLTAGE_UPDATE_MIN_MS       400          // 向 APP 上报的最小时间间隔
#define VOLTAGE_REPORT_DELTA_MV     80           // 只有变化超过该阈值才上报（mV）

// 低电阈值（2S 建议 6.6~7.0V；略放宽减少误报）
#define LOW_BATT_MV                 6800

typedef enum {
  MODE_NULL,
  MODE_ROCKERANDGRAVITY,
  MODE_RGB_ADJUST,
  MODE_SPEED_CONTROL,
  MODE_ULTRASOUND_SEND,
  MODE_SERVO_CONTROL,
  MODE_VOLTAGE_SEND,
  MODE_AVOID,
  MODE_LINE_TRACK
} CarMode;

typedef enum { WARNING_OFF, WARNING_BEEP, WARNING_RGB } VoltageWarning;
typedef enum { READ_VOLTAGE_ON, READ_VOLTAGE_OFF } ReadVoltageState;

Servo myservo;
Ultrasound ultrasound;

static VoltageWarning g_warning = WARNING_OFF;
static CarMode g_mode = MODE_NULL;
static ReadVoltageState g_read = READ_VOLTAGE_OFF;

static uint8_t g_state = 8;
static uint8_t avoid_flag = 0;
static uint8_t rot_flag = 0;

static int car_derection = 0;
static int8_t car_rot = 0;
static uint8_t speed_data = 0;
static uint8_t speed_update = 50;

/* 电压 */
static float voltage;
static int voltage_send;
static int last_voltage_send;
static int real_voltage_send;
static int error_voltage;
static bool voltage_valid = false;
static uint32_t last_voltage_valid_ms = 0;

// 新增：IIR 滤波后的毫伏 & 上报节流
static int filt_voltage_mv = 0;
static uint32_t last_voltage_report_ms = 0;

static CRGB rgbs[1];
String rec_data[4];
char *charArray;

/* 引脚 */
const static uint8_t ledPin = 2;
const static uint8_t servoPin = 5;
const static uint8_t motorpwmPin[4] = { 10, 9, 6, 11};
const static uint8_t motordirectionPin[4] = { 12, 8, 7, 13};

const static int pwmFrequency = 500;
const static int period = 10000000 / pwmFrequency;

static uint32_t previousTime_us = 0;
const static uint32_t interval_ms = 1000;
static uint32_t previousTime_ms = 0;

static int increase_angle = 0;
static int default_angle = 90;
static uint16_t distance = 0;

/* 巡线 */
const static uint8_t pwm_min = 50;
static uint8_t line_bits[4];
static bool line_track_enabled = false;

/* 按键门控/去抖 */
static bool key_last = false;
static bool key_curr = false;
static uint32_t key_debounce_ms = 0;
static uint32_t last_key_activity_ms = 0;

/* 调试宏 */
#if SERIAL_DEBUG
  #define DBG(...)     do{ Serial.print(__VA_ARGS__); }while(0)
  #define DBGLN(...)   do{ Serial.println(__VA_ARGS__); }while(0)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

/* 声明 */
void Aovid(void);
void Rgb_Task(void);
void Motor_Init(void);
void Speed_Task(void);
void Task_Dispatcher(void);
void Servo_Data_Receive(void);
void Rockerandgravity_Task(void);
void Voltage_Detection(void);
void PWM_Out(uint8_t PWM_Pin, int8_t DutyCycle);
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue);
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

bool WireWriteByte(uint8_t val);
bool WireReadDataByte(uint8_t reg, uint8_t &val);
void Sensor_Receive_LineBits(void);
void Tracking_Line_Task_NonBlocking(void);
void LineKey_Update(void);

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, ledPin, RGB>(rgbs, 1);
  Motor_Init();

  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);
  myservo.write(default_angle + increase_angle);

  pinMode(KEY_PIN_ANALOG, INPUT);
  Wire.begin();

  voltage_send = last_voltage_send = real_voltage_send = 0;
  voltage_valid = false;
  last_voltage_valid_ms = 0;
  filt_voltage_mv = 0;
  last_voltage_report_ms = 0;

  g_read = READ_VOLTAGE_OFF;
}

void loop() {
  LineKey_Update();
  Velocity_Controller(car_derection, speed_data, car_rot);
  Task_Dispatcher();

#if VOLTAGE_ENABLED
  uint32_t now = millis();
  bool key_is_idle = !key_curr;
  bool quiet_enough = (now - last_key_activity_ms) >= KEY_QUIET_MS;
  if (g_read == READ_VOLTAGE_ON && key_is_idle && quiet_enough) {
    Voltage_Detection();
  } else {
    if (millis() - last_voltage_valid_ms > VOLTAGE_VALID_TIMEOUT_MS) {
      voltage_valid = false;
    }
  }
#endif

  if (line_track_enabled) {
    Tracking_Line_Task_NonBlocking();
  }
}

/* RGB */
void Rgb_Show(uint8_t rValue,uint8_t gValue,uint8_t bValue) {
  rgbs[0].r = rValue;
  rgbs[0].g = gValue;
  rgbs[0].b = bValue;
  FastLED.show();
}

/* 串口命令调度 */
void Task_Dispatcher(void)
{
  while (Serial.available() > 0)
  {
    String cmd = Serial.readStringUntil('$');

    uint8_t index = 0;
    while (cmd.indexOf('|') != -1 && index < 4)
    {
      rec_data[index] = cmd.substring(0, cmd.indexOf('|'));
      cmd = cmd.substring(cmd.indexOf('|') + 1);
      index++;
    }
    if (index == 0) return;

    charArray = rec_data[0].c_str();
    bool motion_locked = line_track_enabled;

    if(strcmp(charArray, "A") == 0 && avoid_flag == 0 && !motion_locked) {
      g_mode = MODE_ROCKERANDGRAVITY;
    }
    else if(strcmp(charArray, "B") == 0 && avoid_flag == 0) {
      g_mode = MODE_RGB_ADJUST;
    }
    else if(strcmp(charArray, "C") == 0 && avoid_flag == 0 && !motion_locked) {
      g_mode = MODE_SPEED_CONTROL;
    }
    else if(strcmp(charArray, "E") == 0 && avoid_flag == 0 && !motion_locked) {
      g_mode = MODE_SERVO_CONTROL;
    }
    else if(strcmp(charArray, "D") == 0) {
      g_mode = MODE_ULTRASOUND_SEND;
    }
    else if(strcmp(charArray, "F") == 0 && !motion_locked) {
      g_mode = MODE_AVOID;
      avoid_flag = 1;
      g_state = atoi(rec_data[1].c_str());
    }
  }

  if(g_mode == MODE_ROCKERANDGRAVITY) { Rockerandgravity_Task(); g_mode = MODE_NULL; }
  if(g_mode == MODE_RGB_ADJUST)       { Rgb_Task();             g_mode = MODE_NULL; }
  if(g_mode == MODE_SPEED_CONTROL)    { Speed_Task();           g_mode = MODE_NULL; }
  if(g_mode == MODE_SERVO_CONTROL)    { Servo_Data_Receive();   g_mode = MODE_NULL; }

  if(g_mode == MODE_ULTRASOUND_SEND) {
    distance = ultrasound.Filter();
    Serial.print("$");Serial.print(distance);Serial.print(",");
    Serial.print(real_voltage_send);Serial.print("$");
    g_mode = MODE_NULL;
  }

  if(avoid_flag == 1 && !line_track_enabled) { Aovid(); }

  g_read = (g_state == 8) ? READ_VOLTAGE_ON : READ_VOLTAGE_OFF;
}

/* 摇杆控制 —— 方向档0~7时清除自转 */
void Rockerandgravity_Task(void)
{
  g_state = atoi(rec_data[1].c_str());
  switch (g_state)
  {
    case 0: car_derection = 90;  speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 1: car_derection = 45;  speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 2: car_derection = 0;   speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 3: car_derection = 315; speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 4: car_derection = 270; speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 5: car_derection = 225; speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 6: car_derection = 180; speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 7: car_derection = 135; speed_data = speed_update; car_rot=0; rot_flag=0; break;
    case 8:
      car_derection = 0; speed_data = 0;
      car_rot = (rot_flag==1 ? speed_update : (rot_flag==2 ? -speed_update : 0));
      break;
    case 9:
      rot_flag = 1;
      car_rot = (speed_data == 0) ? speed_update : (speed_update/3);
      break;
    case 10:
      rot_flag = 2;
      car_rot = (speed_data == 0) ? -speed_update : -(speed_update/3);
      break;
    case 11:
      car_rot = 0; rot_flag = 0;
      break;
    default: break;
  }
}

/* RGB */
void Rgb_Task(void)
{
  uint8_t r_data = (uint8_t)atoi(rec_data[1].c_str());
  uint8_t g_data = (uint8_t)atoi(rec_data[2].c_str());
  uint8_t b_data = (uint8_t)atoi(rec_data[3].c_str());
  ultrasound.Color(r_data,g_data,b_data,r_data,g_data,b_data);
}

/* === 速度调节函数 === */
void Speed_Task(void)
{
  speed_update = (uint8_t)atoi(rec_data[1].c_str());
  Serial.print("C|");
  Serial.print(speed_update);
  Serial.print("|$");
}

/* 电压：多次采样去极值 + IIR 低通 + 限频/限幅上报 + 仅在按键空闲稳定时运行 */
void Voltage_Detection(void)
{
#if VOLTAGE_ENABLED
  // 采样缓存（局部数组，避免全局内存占用）
  int rawBuf[VOLTAGE_SAMPLES];
  int n = 0;

  // 多次采样：丢掉极低噪声样本
  for (int i = 0; i < VOLTAGE_SAMPLES; ++i) {
    int raw = analogRead(VOLTAGE_PIN);
    if (raw >= VOLTAGE_RAW_MIN) {
      rawBuf[n++] = raw;
    }
  }

  if (n < (VOLTAGE_SAMPLES - 2*VOLTAGE_TRIM)) {
    // 有效样本太少：保持上次值，不更新
    if (millis() - last_voltage_valid_ms > VOLTAGE_VALID_TIMEOUT_MS) {
      voltage_valid = false;
    }
    return;
  }

  // 对有效部分做简单选择排序（小数组，代价可接受）
  for (int i = 0; i < n-1; ++i) {
    int minIdx = i;
    for (int j = i+1; j < n; ++j) {
      if (rawBuf[j] < rawBuf[minIdx]) minIdx = j;
    }
    if (minIdx != i) { int t = rawBuf[i]; rawBuf[i] = rawBuf[minIdx]; rawBuf[minIdx] = t; }
  }

  // 去掉两端各 TRIM 个，做均值
  long sum = 0;
  int start = VOLTAGE_TRIM;
  int end   = n - VOLTAGE_TRIM;
  for (int k = start; k < end; ++k) sum += rawBuf[k];
  float raw_avg = (float)sum / (end - start);

  // 转毫伏
  voltage = raw_avg * VOLTAGE_K;          // V
  int mv = (int)(voltage * 1000 + 0.5f);  // mV 四舍五入

  // IIR 低通：y = (1-α)*y + α*x
  if (!voltage_valid) {
    filt_voltage_mv = mv; // 首次初始化
  } else {
    filt_voltage_mv = ((VOLTAGE_IIR_DEN - VOLTAGE_IIR_NUM) * filt_voltage_mv +
                        VOLTAGE_IIR_NUM * mv) / VOLTAGE_IIR_DEN;
  }

  // 标记最近有效
  voltage_valid = true;
  last_voltage_valid_ms = millis();

  // 仅当间隔足够且变化幅度超过阈值才更新“发给 APP 的值”
  uint32_t now = millis();
  if ( (now - last_voltage_report_ms >= VOLTAGE_UPDATE_MIN_MS) &&
       (abs(filt_voltage_mv - real_voltage_send) >= VOLTAGE_REPORT_DELTA_MV) ) {
    real_voltage_send = filt_voltage_mv;
    last_voltage_report_ms = now;
  }

  // 低电判定基于“滤波后的电压”
  if (real_voltage_send <= LOW_BATT_MV) {
    g_warning = WARNING_RGB;
    Rgb_Show(0,10,0); // 蜂鸣器禁用，仅微绿提示
  } else {
    g_warning = WARNING_OFF;
  }

  // 同步旧变量（保持与你原始框架兼容）
  voltage_send = real_voltage_send;
  last_voltage_send = real_voltage_send;
  error_voltage = real_voltage_send;
#endif
}

/* 机械爪 */
void Servo_Data_Receive(void)
{
  increase_angle = atoi(rec_data[1].c_str());
  myservo.write(default_angle + increase_angle);
}

/* 避障 */
void Aovid(void)
{
  distance = ultrasound.Filter();
  if(g_state == 1)
  {
    if(distance < 400) { car_derection = 0; car_rot = 100; speed_data = 0; }
    if(distance >= 500){ car_derection = 0; car_rot = 0;   speed_data = 50; }
  }
  else if(g_state == 0)
  {
    car_derection = 0; car_rot = 0; speed_data = 0;
    g_mode = MODE_NULL; avoid_flag = 0;
  }
}

/* 电机初始化 */
void Motor_Init(void)
{
  for(uint8_t i = 0; i < 4; i++) {
    pinMode(motordirectionPin[i], OUTPUT);
    pinMode(motorpwmPin[i], OUTPUT);
  }
  Velocity_Controller( 0, 0, 0);
}

/* 运动学控制 */
void Velocity_Controller(uint16_t angle, uint8_t velocity,int8_t rot)
{
  int8_t v0, v1, v2, v3;
  float speed = (rot == 0) ? 1 : 0.5;
  angle += 90;
  float rad = angle * PI / 180;
  velocity /= sqrt(2);
  v0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
  v1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
  v2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
  v3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
  Motors_Set(v0, v1, v2, v3);
}

/* 软件PWM + 转向 */
void Motors_Set(int8_t M0, int8_t M1, int8_t M2, int8_t M3)
{
  int8_t pwm_set[4];
  int8_t motors[4] = { M0, M1, M2, M3};
  bool direction[4] = { 1, 0, 0, 1};
  for(uint8_t i=0; i < 4; ++i)
  {
    if(motors[i] < 0) direction[i] = !direction[i];
    if(motors[i] == 0) pwm_set[i] = 0;
    else {
      int8_t duty = abs(motors[i]);
      pwm_set[i] = max(duty, (int8_t)map(pwm_min, 0, 255, 0, 100));
    }
    digitalWrite(motordirectionPin[i], direction[i]);
    PWM_Out(motorpwmPin[i], pwm_set[i]);
  }
}

/* 软件PWM */
void PWM_Out(uint8_t PWM_Pin, int8_t DutyCycle)
{
  uint32_t currentTime_us = micros();
  int highTime = (period/100) * DutyCycle;
  if ((currentTime_us - previousTime_us) <= highTime) digitalWrite(PWM_Pin, HIGH);
  else                                                 digitalWrite(PWM_Pin, LOW);
  if (currentTime_us - previousTime_us >= period) previousTime_us = currentTime_us;
}

/* I2C */
bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) return false;
  Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1);
  while (Wire.available()) { val = Wire.read(); }
  return true;
}

/* 灰度读取 */
void Sensor_Receive_LineBits(void) {
  uint8_t data = 0;
  if (WireReadDataByte(1, data)) {
    line_bits[0] =  data       & 0x01;
    line_bits[1] = (data >> 1) & 0x01;
    line_bits[2] = (data >> 2) & 0x01;
    line_bits[3] = (data >> 3) & 0x01;
  }
}

/* 非阻塞巡线 */
void Tracking_Line_Task_NonBlocking(void) {
  Rgb_Show(255, 0, 0);
  Sensor_Receive_LineBits();

  if (line_bits[1] == 1 && line_bits[2] == 1) {
    car_derection = 0;  speed_data = 80;  car_rot = 0;
  } else if (line_bits[1] == 1 && line_bits[2] == 0) {
    car_derection = 0;  speed_data = 80;  car_rot = 65;
  } else if (line_bits[1] == 0 && line_bits[2] == 1) {
    car_derection = 0;  speed_data = 80;  car_rot = -65;
  } else {
    car_derection = 0;  speed_data = 0;   car_rot = 0;
  }
}

/* A3 按键切换巡线 */
void LineKey_Update(void) {
  uint32_t now = millis();
  int rawA = analogRead(KEY_PIN_ANALOG);
  key_curr = KEY_ACTIVE_LOW ? (rawA < KEY_ADC_TH) : (rawA > (1023 - KEY_ADC_TH));

  if (key_curr != key_last && (now - key_debounce_ms) > 80) {
    key_debounce_ms = now;
    key_last = key_curr;
    last_key_activity_ms = now;

    if (key_curr) {
      line_track_enabled = !line_track_enabled;
      car_derection = 0; speed_data = 0; car_rot = 0; rot_flag = 0;
      if (line_track_enabled) {
        avoid_flag = 0;
        Rgb_Show(255, 0, 0);
      } else {
        Rgb_Show(255, 255, 255);
      }
    }
  }
  if (key_curr) last_key_activity_ms = now;
}
