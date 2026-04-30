/*
 * ============================================================
 *   Arduino Pro Mini 车船模型控制板 程序
 * ============================================================
 * 功能说明：
 *   通过NRF24L01无线模块接收遥控器数据，控制有刷/无刷电机和舵机，
 *   使用0.91寸OLED显示本地和远程信息，通过PCF8574扩展I/O，
 *   通过MPU6050实现直线行驶自修正，支持设置模式保存参数到EEPROM。
 *
 * 使用的库及版本：
 *   - SPI.h            (Arduino内置)      - SPI通信
 *   - Wire.h           (Arduino内置)      - I2C通信
 *   - Mirf.h           (0.1.0)            - NRF24L01驱动
 *   - MirfHardwareSpiDriver.h              - NRF24L01 SPI驱动
 *   - Adafruit_GFX.h   (v1.11.9)          - OLED图形库基础
 *   - Adafruit_SSD1306.h (v2.5.7)         - SSD1306 OLED驱动
 *   - Adafruit_PCF8574.h (v1.1.1)         - PCF8574 I2C扩展
 *   - Adafruit_MPU6050.h (v2.1.4)         - MPU6050六轴传感器
 *   - Servo.h          (Arduino内置 v1.1.8) - 舵机/无刷电调控制
 *   - EEPROM.h         (Arduino内置)       - 参数掉电保存
 *   - avr/wdt.h        (Arduino内置)       - 看门狗定时器
 *
 * 开发者：用户
 * 日期：2026-04-30
 * 版本：v2.0（优化版）
 *
 * ============================================================
 * 硬件连接说明（参考YaoDriverV3板）：
 * ============================================================
 * 电机输出接口：
 *   有刷模式：
 *     D3、D9  - 左侧电机方向控制（D3=DIR1, D9=DIR2）
 *     D2、D10 - 右侧电机方向控制（D2=DIR1, D10=DIR2）
 *     D5、D6  - 左/右电机PWM输出（Timer0，约980Hz）
 *   无刷模式（与有刷复用引脚）：
 *     D3      - 左侧无刷电调信号1（Servo库控制）
 *     D5      - 左侧无刷电调信号2（Servo库控制）
 *     D6      - 右侧无刷电调信号1（Servo库控制）
 *     D9      - 右侧无刷电调信号2（Servo库控制）
 *     注意：Servo库使用Timer1，此时D9/D10硬件PWM不可用
 *
 * 电池电压检测：A0（91K/33K电阻分压 + 10K限流电阻）
 * 电池电流检测：A1（需配合电流传感器，如ACS712）
 *
 * PCF8574扩展IO（I2C）：
 *   地址0x20（pcfIO）：
 *     P0~P3 - 输入，复用BTS7960电流反馈
 *     P4~P7 - 输出，辅助驱动电机控制位（对应遥控器按钮）
 *   地址0x21（pcfSetup）：
 *     P0 - 输入，上拉，接地低电平进入设置模式
 *     P1 - 输入，上拉，设置项切换
 *     P2 - 输入，上拉，设置项+操作
 *     P3 - 输入，上拉，设置项-操作
 *     P7 - 输入，上拉，低电平强制使用无刷电机
 *
 * OLED显示屏（I2C，地址0x3C）：
 *   A4 - SDA
 *   A5 - SCL
 *
 * MPU6050六轴传感器（I2C，地址0x68）：
 *   A4 - SDA（与OLED共用I2C总线）
 *   A5 - SCL
 *
 * NRF24L01无线模块（SPI）：
 *   D7  - CSN
 *   D8  - CE
 *   D11 - MOSI
 *   D12 - MISO
 *   D13 - SCK
 * ============================================================
 */

#include <SPI.h>
#include <Wire.h>
#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PCF8574.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>
#include <avr/wdt.h>
#include <EEPROM.h>

/* ============================================================
 *  宏定义 - 引脚与常量
 * ============================================================ */

// ----- 电机引脚定义 -----
const uint8_t LEFT_DIR1  = 3;    // 左侧电机方向1（有刷）/ 左电调1（无刷）
const uint8_t LEFT_DIR2  = 9;    // 左侧电机方向2（有刷）/ 右电调2（无刷）
const uint8_t RIGHT_DIR1 = 2;    // 右侧电机方向1（有刷）
const uint8_t RIGHT_DIR2 = 10;   // 右侧电机方向2（有刷）
const uint8_t LEFT_PWM   = 5;    // 左侧电机PWM（有刷）/ 左电调2（无刷）
const uint8_t RIGHT_PWM  = 6;    // 右侧电机PWM（有刷）/ 右电调1（无刷）

// ----- 传感器引脚 -----
const uint8_t BATTERY_PIN = A0;  // 电池电压检测
const uint8_t CURRENT_PIN = A1;  // 电流检测

// ----- OLED配置 -----
// -1表示不使用RESET引脚，大多数OLED模块有自动复位电路
#define OLED_RESET -1
// OLED I2C地址
#define OLED_ADDR 0x3C

// ----- NRF24L01配置 -----
// 收发双方必须使用相同的payload大小（与遥控器 sizeof(RemoteData) = 21 一致）
#define NRF_PAYLOAD_SIZE 21

// ----- 电压分压参数 -----
// 分压电阻：91K上臂 + 33K下臂
// 已在readSensors()中使用整数运算，不再需要浮点比例

// ADC参考电压=5000mV（5V Arduino Nano），已在readSensors()整数运算中硬编码

// ----- 电流传感器参数 -----
// ACS712-30A: 灵敏度66mV/A，零点2500mV
// 已在readSensors()中使用整数运算

// ----- 电池自动识别阈值（单位：V） -----
// 锂电池满电电压约4.2V/节，标称3.7V/节
#define BAT_1S_MAX  4.3    // <4.3V = 1S
#define BAT_2S_MAX  8.6    // <8.6V = 2S
#define BAT_3S_MAX  13.0   // <13.0V = 3S
// >=13.0V = 4S

// ----- 电机控制参数 -----
#define SERVO_NEUTRAL     1500   // 无刷电调中位脉冲宽度（us）
#define SERVO_MAX_FORWARD 2000   // 最大前进脉冲宽度
#define SERVO_MAX_REVERSE 1000   // 最大后退脉冲宽度
#define STEERING_DEADZONE 30     // 转向死区，避免摇杆漂移
#define RF_TIMEOUT        1000   // 遥控信号超时时间（ms）

// ----- EEPROM配置 -----
#define EEPROM_ADDR        0
#define EEPROM_MAGIC_BYTE  0xAB  // 校验标识，用于判断EEPROM数据是否有效

// ----- 定时参数 -----
#define DISPLAY_INTERVAL  250    // OLED刷新间隔（ms）
#define SEND_INTERVAL     200    // 数据回传间隔（ms）
#define SENSOR_INTERVAL   100    // 传感器读取间隔（ms）

/* ============================================================
 *  数据结构定义
 * ============================================================ */

// 设置参数结构体（保存在EEPROM中）
struct Settings {
  uint8_t magic;         // 校验标识，应为EEPROM_MAGIC_BYTE
  uint8_t channel;       // 遥控器频道，默认90，范围1~125
  uint8_t batteryType;   // 电池节数：1=1S, 2=2S, 3=3S, 4=4S, 0=自动检测
  uint8_t motorType;     // 电机类型：0=有刷，1=无刷（P7硬件开关可覆盖）
  uint8_t useAuxMotor;   // 是否使用辅助驱动输出：0=否，1=是
  int8_t  motorOffset;   // 左电机PWM修正值：-32~+32，默认0
};

// ★ 遥控器传来的数据结构（必须与遥控器 RemoteData 完全一致！）
struct RemoteData {
  uint8_t buttons[8];    // 遥控器按钮状态（8个），0=未按下，1=按下
  int joystick[4];       // 摇杆数据：[0]左Y(油门), [1]左X(方向), [2]右X, [3]右Y
                         // 范围：-throttleHalf ~ +throttleHalf，0为中位
  uint8_t channel;       // 遥控器频道
  uint16_t throttleMax;  // 油门最大值
  uint16_t steeringMax;  // 转向最大值
};  // 总计 21 字节（AVR上int=2字节：8+8+1+2+2=21）

// ★ 回传给遥控器的数据结构（必须与遥控器 RemoteResponse 完全一致！）
struct ControlData {
  uint16_t batteryVoltage; // 电池电压（放大100倍存储，单位0.01V）
  uint16_t current;        // 电流（放大100倍存储，单位0.01A）
  int8_t  leftMotorPWM;    // 左侧电机PWM (-128~127)
  int8_t  rightMotorPWM;   // 右侧电机PWM (-128~127)
  int16_t accelX;          // MPU6050 X轴加速度（放大1000倍）
  int16_t accelY;          // MPU6050 Y轴加速度（放大1000倍）
  int16_t accelZ;          // MPU6050 Z轴加速度（放大1000倍）
};  // 总计 12 字节

/* ============================================================
 *  全局变量
 * ============================================================ */

// ----- 外设对象 -----
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);
Adafruit_PCF8574 pcfIO;       // 扩展IO，地址0x20
Adafruit_PCF8574 pcfSetup;    // 设置控制输入，地址0x21
Adafruit_MPU6050 mpu;

// 无刷电调Servo对象（4路，对应D3, D5, D6, D9）
Servo servoLeft1;    // D3 - 左侧电调1
Servo servoLeft2;    // D5 - 左侧电调2
Servo servoRight1;   // D6 - 右侧电调1
Servo servoRight2;   // D9 - 右侧电调2
bool servoAttached = false;  // Servo对象是否已attach

// ----- 数据 -----
Settings settings;
RemoteData remoteData;
ControlData controlData;
byte nrfBuffer[NRF_PAYLOAD_SIZE];  // NRF24L01收发缓冲区

// ----- 状态标志 -----
bool inSettingMode = false;    // 是否处于设置模式
bool rfConnected = false;      // 遥控器信号是否连接
bool oledAvailable = false;    // OLED是否可用
bool mpuAvailable = false;     // MPU6050是否可用
bool isBrushlessMode = false;  // 当前是否为无刷模式
uint8_t currentSettingIndex = 0; // 当前设置项索引
uint8_t currentScreen = 0;    // 当前显示屏幕编号

// ----- 电机输出值（用于显示和回传） -----
int8_t finalLeftPWM = 0;      // 最终左电机输出 -127~+127
int8_t finalRightPWM = 0;     // 最终右电机输出 -127~+127

// ----- 遥控器数据转换缓存 -----
// 遥控器发来的joystick是偏移量格式（-half ~ +half，0=中位），
// 控制板内部统一转换为 0~1023 格式（512=中位），方便电机控制逻辑使用
uint16_t rcThrottle = 512;      // 油门值 0~1023，512为中位
uint16_t rcSteering = 512;      // 方向值 0~1023，512为中位
uint16_t rcRightStickY = 512;   // 右摇杆Y轴 0~1023
uint16_t rcRightStickX = 512;   // 右摇杆X轴 0~1023
uint8_t  rcInputSW = 0xFF;      // P0~P7开关状态位编码（从buttons[]转换，默认全高电平=上拉）

// ----- 定时变量（用于非阻塞定时） -----
unsigned long lastDisplayTime = 0;   // 上次OLED刷新时间
unsigned long lastSendTime = 0;      // 上次数据回传时间
unsigned long lastSensorTime = 0;    // 上次传感器读取时间
unsigned long lastReceiveTime = 0;   // 上次收到遥控数据时间
unsigned long settingEnterTime = 0;  // 进入设置模式的时间

// ----- MPU6050传感器数据 -----
sensors_event_t accelEvent, gyroEvent, tempEvent;

// ----- 调试开关（设为1开启串口调试，设为0关闭节省约1KB Flash） -----
#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#else
#define DBG(x)
#define DBGLN(x)
#endif

// ----- 电池电压缓存（整数，单位mV，避免浮点库） -----
uint16_t batteryVoltageMV = 0;  // 电池电压，单位毫伏

/* ============================================================
 *  设置参数 - 读取 / 保存 / 恢复默认
 * ============================================================ */

// 加载设置（从EEPROM读取）
void loadSettings() {
  EEPROM.get(EEPROM_ADDR, settings);
  // 校验magic字节，如果不匹配说明EEPROM数据无效，恢复默认值
  if (settings.magic != EEPROM_MAGIC_BYTE ||
      settings.channel == 0 || settings.channel > 125) {
    restoreDefaultSettings();
  }
}

// 保存设置（写入EEPROM）
void saveSettings() {
  settings.magic = EEPROM_MAGIC_BYTE;
  EEPROM.put(EEPROM_ADDR, settings);
}

// 恢复默认设置
void restoreDefaultSettings() {
  settings.magic = EEPROM_MAGIC_BYTE;
  settings.channel = 90;
  settings.batteryType = 0;   // 0=自动检测
  settings.motorType = 0;     // 0=有刷
  settings.useAuxMotor = 0;   // 不使用辅助输出
  settings.motorOffset = 0;   // 无修正
  saveSettings();
}

/* ============================================================
 *  电池自动检测
 * ============================================================ */

// 根据电压自动判断电池节数（参数单位mV）
uint8_t autoDetectBattery(uint16_t voltageMV) {
  if (voltageMV < (uint16_t)(BAT_1S_MAX * 1000)) return 1;
  if (voltageMV < (uint16_t)(BAT_2S_MAX * 1000)) return 2;
  if (voltageMV < (uint16_t)(BAT_3S_MAX * 1000)) return 3;
  return 4;
}

/* ============================================================
 *  电机模式管理
 * ============================================================ */

// 更新电机模式（根据P7硬件开关和软件设置）
void updateMotorMode() {
  // 读取PCF8574(0x21)的P7引脚
  int p7State = pcfSetup.digitalRead(7);

  // P7低电平=强制无刷；P7高电平=使用软件设置
  bool newMode = (p7State == LOW) ? true : (settings.motorType == 1);

  // 模式切换时需要重新配置引脚
  if (newMode != isBrushlessMode) {
    if (newMode) {
      // 切换到无刷模式：attach Servo对象
      enterBrushlessMode();
    } else {
      // 切换到有刷模式：detach Servo对象，恢复GPIO
      enterBrushedMode();
    }
    isBrushlessMode = newMode;
  }
}

// 进入有刷电机模式
void enterBrushedMode() {
  // 先detach所有Servo对象，释放Timer1
  if (servoAttached) {
    servoLeft1.detach();
    servoLeft2.detach();
    servoRight1.detach();
    servoRight2.detach();
    servoAttached = false;
  }

  // 配置方向引脚为输出
  pinMode(LEFT_DIR1, OUTPUT);   // D3
  pinMode(LEFT_DIR2, OUTPUT);   // D9
  pinMode(RIGHT_DIR1, OUTPUT);  // D2
  pinMode(RIGHT_DIR2, OUTPUT);  // D10
  pinMode(LEFT_PWM, OUTPUT);    // D5
  pinMode(RIGHT_PWM, OUTPUT);   // D6

  // 初始状态：全部低电平（停止）
  digitalWrite(LEFT_DIR1, LOW);
  digitalWrite(LEFT_DIR2, LOW);
  digitalWrite(RIGHT_DIR1, LOW);
  digitalWrite(RIGHT_DIR2, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// 进入无刷电机模式
void enterBrushlessMode() {
  // D2、D10在无刷模式下不使用，设为输入（高阻态）
  pinMode(RIGHT_DIR1, INPUT);   // D2
  pinMode(RIGHT_DIR2, INPUT);   // D10

  // attach Servo对象到D3, D5, D6, D9
  // Servo库会接管这些引脚，输出50Hz的舵机信号
  servoLeft1.attach(LEFT_DIR1);   // D3 - 左电调1
  servoLeft2.attach(LEFT_PWM);    // D5 - 左电调2
  servoRight1.attach(RIGHT_PWM);  // D6 - 右电调1
  servoRight2.attach(LEFT_DIR2);  // D9 - 右电调2
  servoAttached = true;

  // 初始化到中位（停止）
  servoLeft1.writeMicroseconds(SERVO_NEUTRAL);
  servoLeft2.writeMicroseconds(SERVO_NEUTRAL);
  servoRight1.writeMicroseconds(SERVO_NEUTRAL);
  servoRight2.writeMicroseconds(SERVO_NEUTRAL);
}

/* ============================================================
 *  电机停止
 * ============================================================ */

void stopMotors() {
  if (isBrushlessMode) {
    // 无刷模式：输出中位脉冲（停止）
    if (servoAttached) {
      servoLeft1.writeMicroseconds(SERVO_NEUTRAL);
      servoLeft2.writeMicroseconds(SERVO_NEUTRAL);
      servoRight1.writeMicroseconds(SERVO_NEUTRAL);
      servoRight2.writeMicroseconds(SERVO_NEUTRAL);
    }
  } else {
    // 有刷模式：方向引脚全低，PWM归零
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, LOW);
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
  }
  finalLeftPWM = 0;
  finalRightPWM = 0;
}

/* ============================================================
 *  传感器读取
 * ============================================================ */

void readSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL) return;
  lastSensorTime = now;

  // ----- 读取电池电压（整数运算，单位mV） -----
  // 分压比 124/33，ADC参考5.0V
  // Vin(mV) = adcValue × 5000 × 124 / (1023 × 33)
  // 简化: Vin(mV) = adcValue × 620000 / 33759 ≈ adcValue × 18.37
  // 更精确: adcValue × 620000L / 33759
  uint16_t adcValue = analogRead(BATTERY_PIN);
  batteryVoltageMV = (uint16_t)((uint32_t)adcValue * 620000UL / 33759UL);
  controlData.batteryVoltage = (uint16_t)((uint32_t)adcValue * 62000UL / 33759UL);

  // ----- 读取电流（整数运算，单位0.01A） -----
  // Vout(mV) = adcValue × 5000 / 1023
  // I(0.01A) = |Vout - 2500| / 66  (灵敏度66mV/A，零点2500mV)
  adcValue = analogRead(CURRENT_PIN);
  int16_t currentMV = (int16_t)((uint32_t)adcValue * 5000UL / 1023UL);
  int16_t diff = currentMV - 2500;
  if (diff < 0) diff = -diff;
  controlData.current = (uint16_t)((uint32_t)diff * 100UL / 66UL);

  // ----- 读取MPU6050 -----
  if (mpuAvailable) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    controlData.accelX = (int16_t)(accelEvent.acceleration.x * 100);
    controlData.accelY = (int16_t)(accelEvent.acceleration.y * 100);
    controlData.accelZ = (int16_t)(accelEvent.acceleration.z * 100);
    // gyroZ仍保留在gyroEvent中供直线修正使用，但不再回传给遥控器
  }
}

/* ============================================================
 *  电机控制 - 核心逻辑
 * ============================================================ */

void controlMotors() {
  // 设置模式或信号丢失时，停止电机
  if (inSettingMode || !rfConnected) {
    stopMotors();
    return;
  }

  // 更新电机模式（检查P7硬件开关）
  updateMotorMode();

  // ----- 解析遥控器输入（使用转换后的0~1023格式数据） -----
  // rcThrottle: 油门，512为中位（停止），>512前进，<512后退
  // rcSteering: 方向，512为中位，>512右转，<512左转
  int throttle = rcThrottle;
  int steering = rcSteering;

  // 转换为以0为中心的偏移量
  int basePWM = throttle - 512;     // -512 ~ +511
  int steerOffset = steering - 512; // -512 ~ +511

  // 限制范围
  basePWM = constrain(basePWM, -512, 511);

  // ----- 计算左右电机基础速度（0~512） -----
  int leftSpeed = abs(basePWM);
  int rightSpeed = abs(basePWM);

  // ----- 应用转向（差速转向） -----
  // 方向往左转（steerOffset < 0）：右侧保持不变，减小左侧
  // 方向往右转（steerOffset > 0）：左侧保持不变，减小右侧
  if (steerOffset > STEERING_DEADZONE) {
    // 右转：减小右侧电机速度
    rightSpeed = max(rightSpeed - abs(steerOffset), 0);
  } else if (steerOffset < -STEERING_DEADZONE) {
    // 左转：减小左侧电机速度
    leftSpeed = max(leftSpeed - abs(steerOffset), 0);
  }

  // ----- 应用电机偏差修正（仅修正左侧电机） -----
  // motorOffset为正时增加左侧PWM，为负时减少
  leftSpeed = constrain(leftSpeed + settings.motorOffset * 4, 0, 512);

  // ----- 直线行驶自修正（使用MPU6050陀螺仪） -----
  // 仅在接近直线行驶时启用（转向偏移量在死区内）
  if (abs(steerOffset) < STEERING_DEADZONE && mpuAvailable && abs(basePWM) > 20) {
    // 使用Z轴角速度（偏航率）检测行驶偏转
    // gyro.z > 0 表示左偏转，需要增加左侧电机速度
    // gyro.z < 0 表示右偏转，需要增加右侧电机速度
    // 转换为0.01rad/s精度的整数，乘以比例系数30/100=0.3
    int16_t yaw100 = (int16_t)(gyroEvent.gyro.z * 100);
    int correction = (int)yaw100 * 30 / 100;
    leftSpeed  = constrain(leftSpeed + correction, 0, 512);
    rightSpeed = constrain(rightSpeed - correction, 0, 512);
  }

  // ----- 限制最终速度 -----
  leftSpeed  = constrain(leftSpeed, 0, 512);
  rightSpeed = constrain(rightSpeed, 0, 512);

  // ----- 根据电机模式输出控制信号 -----
  if (isBrushlessMode) {
    controlBrushlessMotors(basePWM, leftSpeed, rightSpeed);
  } else {
    controlBrushedMotors(basePWM, leftSpeed, rightSpeed);
  }

  // ----- 遥控器4位按钮信号输出 -----
  // 将遥控器buttons[0~3]（轻触开关P0~P3）映射到PCF8574(0x20)的P4~P7输出
  // P0→P4, P1→P5, P2→P6, P3→P7
  for (uint8_t i = 0; i < 4; i++) {
    bool state = bitRead(rcInputSW, i);
    pcfIO.digitalWrite(i + 4, state ? HIGH : LOW);
  }
}

// 有刷电机控制
void controlBrushedMotors(int basePWM, int leftSpeed, int rightSpeed) {
  // 将0~512的速度映射到0~255的PWM占空比
  uint8_t leftPWM  = (uint8_t)map(leftSpeed, 0, 512, 0, 255);
  uint8_t rightPWM = (uint8_t)map(rightSpeed, 0, 512, 0, 255);

  // 记录最终输出值（带符号，用于回传和显示）
  if (basePWM > 5) {
    // 前进：D2/D3输出高电平，D9/D10输出低电平
    digitalWrite(LEFT_DIR1, HIGH);   // D3=HIGH
    digitalWrite(LEFT_DIR2, LOW);    // D9=LOW
    digitalWrite(RIGHT_DIR1, HIGH);  // D2=HIGH
    digitalWrite(RIGHT_DIR2, LOW);   // D10=LOW
    analogWrite(LEFT_PWM, leftPWM);
    analogWrite(RIGHT_PWM, rightPWM);
    finalLeftPWM  = map(leftPWM, 0, 255, 0, 127);
    finalRightPWM = map(rightPWM, 0, 255, 0, 127);
  } else if (basePWM < -5) {
    // 后退：D2/D3输出低电平，D9/D10输出高电平
    digitalWrite(LEFT_DIR1, LOW);    // D3=LOW
    digitalWrite(LEFT_DIR2, HIGH);   // D9=HIGH
    digitalWrite(RIGHT_DIR1, LOW);   // D2=LOW
    digitalWrite(RIGHT_DIR2, HIGH);  // D10=HIGH
    analogWrite(LEFT_PWM, leftPWM);
    analogWrite(RIGHT_PWM, rightPWM);
    finalLeftPWM  = -map(leftPWM, 0, 255, 0, 127);
    finalRightPWM = -map(rightPWM, 0, 255, 0, 127);
  } else {
    // 停止
    stopMotors();
  }

  // 更新回传数据
  controlData.leftMotorPWM  = finalLeftPWM;
  controlData.rightMotorPWM = finalRightPWM;
}

// 无刷电机控制（双向电调，使用Servo库输出50Hz舵机信号）
void controlBrushlessMotors(int basePWM, int leftSpeed, int rightSpeed) {
  if (!servoAttached) return;

  // 将速度(0~512)映射为脉冲宽度偏移(0~500us)
  // 中位1500us=停止，>1500us=前进，<1500us=后退
  int leftOffset  = map(leftSpeed, 0, 512, 0, 500);
  int rightOffset = map(rightSpeed, 0, 512, 0, 500);

  int leftPulse, rightPulse;

  if (basePWM > 5) {
    // 前进：脉冲宽度 > 1500us
    leftPulse  = SERVO_NEUTRAL + leftOffset;
    rightPulse = SERVO_NEUTRAL + rightOffset;
    finalLeftPWM  = map(leftOffset, 0, 500, 0, 127);
    finalRightPWM = map(rightOffset, 0, 500, 0, 127);
  } else if (basePWM < -5) {
    // 后退：脉冲宽度 < 1500us
    leftPulse  = SERVO_NEUTRAL - leftOffset;
    rightPulse = SERVO_NEUTRAL - rightOffset;
    finalLeftPWM  = -map(leftOffset, 0, 500, 0, 127);
    finalRightPWM = -map(rightOffset, 0, 500, 0, 127);
  } else {
    // 停止：中位脉冲
    leftPulse  = SERVO_NEUTRAL;
    rightPulse = SERVO_NEUTRAL;
    finalLeftPWM  = 0;
    finalRightPWM = 0;
  }

  // 限制脉冲宽度范围（1000~2000us）
  leftPulse  = constrain(leftPulse,  SERVO_MAX_REVERSE, SERVO_MAX_FORWARD);
  rightPulse = constrain(rightPulse, SERVO_MAX_REVERSE, SERVO_MAX_FORWARD);

  // 输出到电调
  servoLeft1.writeMicroseconds(leftPulse);   // D3
  servoLeft2.writeMicroseconds(leftPulse);   // D5（与D3同步）
  servoRight1.writeMicroseconds(rightPulse); // D6
  servoRight2.writeMicroseconds(rightPulse); // D9（与D6同步）

  // 更新回传数据
  controlData.leftMotorPWM  = finalLeftPWM;
  controlData.rightMotorPWM = finalRightPWM;
}

/* ============================================================
 *  遥控数据接收
 * ============================================================ */

// 收到遥控器数据后解析并回传
static bool pendingReply = false;   // 有待回传的数据（给sendControlData用）
static bool newDataArrived = false; // 有新数据需要解析（给parseRemoteData用）

void receiveRemoteData() {
  // ★ 确保NRF在RX模式：isSending()在发送完成时会自动切换回RX模式
  if (Mirf.isSending()) return;

  if (Mirf.dataReady()) {
    // 读取数据到缓冲区，再拷贝到结构体（避免溢出）
    Mirf.getData(nrfBuffer);
    memset(&remoteData, 0, sizeof(RemoteData));
    memcpy(&remoteData, nrfBuffer, sizeof(RemoteData));
    lastReceiveTime = millis();
    rfConnected = true;
    pendingReply = true;   // 标记需要回传
    newDataArrived = true; // 标记需要解析
  }

  // ----- 信号丢失保护（Fail-Safe） -----
  if (millis() - lastReceiveTime > RF_TIMEOUT) {
    rfConnected = false;
  }
}

// 解析遥控器数据（在回传之后执行，避免延迟回传）
void parseRemoteData() {
  if (!newDataArrived) return;
  newDataArrived = false;

  // ★ 将遥控器的buttons[8]数组转换为位编码inputSW
  rcInputSW = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (remoteData.buttons[i]) {
      bitSet(rcInputSW, i);
    }
  }

  // ★ 将遥控器的joystick偏移量转换为0~1023格式
  int throttleHalf = remoteData.throttleMax / 2;
  int steeringHalf = remoteData.steeringMax / 2;
  if (throttleHalf == 0) throttleHalf = 127;
  if (steeringHalf == 0) steeringHalf = 127;

  rcThrottle = (uint16_t)constrain(
    map(remoteData.joystick[0], -throttleHalf, throttleHalf, 0, 1023), 0, 1023);
  rcSteering = (uint16_t)constrain(
    map(remoteData.joystick[1], -steeringHalf, steeringHalf, 0, 1023), 0, 1023);
  rcRightStickY = (uint16_t)constrain(
    map(remoteData.joystick[3], -throttleHalf, throttleHalf, 0, 1023), 0, 1023);
  rcRightStickX = (uint16_t)constrain(
    map(remoteData.joystick[2], -steeringHalf, steeringHalf, 0, 1023), 0, 1023);

  // 根据遥控器P6/P7位切换显示屏幕
  bool p6 = bitRead(rcInputSW, 6);
  bool p7 = bitRead(rcInputSW, 7);
  if (!p6) {
    currentScreen = 0;
  } else if (p6 && p7) {
    currentScreen = 1;
  } else if (!p7) {
    currentScreen = 2;
  }
}

// 回传控制数据（在loop中紧接receiveRemoteData之后调用）
void sendControlData() {
  if (!pendingReply) return;  // 没有收到新数据，不回传
  pendingReply = false;       // 标记已处理（无论发送是否成功）

  // 立刻回传，不做任何延迟
  memset(nrfBuffer, 0, NRF_PAYLOAD_SIZE);
  memcpy(nrfBuffer, &controlData, sizeof(ControlData));
  Mirf.send(nrfBuffer);
  // ★ 加超时防止卡死（无ACK时正常<1ms，但加保险）
  unsigned long ts = millis();
  while(Mirf.isSending()) {
    wdt_reset();               // 等发送时喂狗，防止WDT复位
    if (millis() - ts > 100) { // 最多等100ms
      Mirf.powerUpRx();        // 强制切回RX模式
      Mirf.flushRx();          // 清RX FIFO
      break;
    }
  }
  lastSendTime = millis();
}

/* ============================================================
 *  设置模式处理
 * ============================================================ */

void handleSetupMode() {
  int p0State = pcfSetup.digitalRead(0);

  // ----- 进入设置模式 -----
  if (!inSettingMode && p0State == LOW) {
    inSettingMode = true;
    currentSettingIndex = 0;
    settingEnterTime = millis();  // 记录进入时间（全局变量，不会被覆盖）
    stopMotors();  // 立即停止所有电机
    return;
  }

  // ----- 设置模式主逻辑 -----
  if (!inSettingMode) return;

  // 读取设置按钮状态（P1=切换项，P2=加，P3=减）
  int btnSelect = pcfSetup.digitalRead(1);
  int btnPlus   = pcfSetup.digitalRead(2);
  int btnMinus  = pcfSetup.digitalRead(3);

  // 防抖：使用静态变量记录上次按键时间
  static unsigned long lastSelectPress = 0;
  static unsigned long lastPlusPress   = 0;
  static unsigned long lastMinusPress  = 0;

  // P1：切换设置项（共5项，循环）
  if (btnSelect == LOW && millis() - lastSelectPress > 300) {
    currentSettingIndex = (currentSettingIndex + 1) % 5;
    lastSelectPress = millis();
  }

  // P2：增加当前设置项的值
  if (btnPlus == LOW && millis() - lastPlusPress > 300) {
    switch (currentSettingIndex) {
      case 0: // 遥控器频道
        settings.channel = min(settings.channel + 1, (uint8_t)125);
        break;
      case 1: // 电池节数（0=自动，1~4=手动）
        settings.batteryType = min(settings.batteryType + 1, (uint8_t)4);
        break;
      case 2: // 电机类型切换
        settings.motorType = 1 - settings.motorType;
        break;
      case 3: // 辅助输出开关
        settings.useAuxMotor = 1 - settings.useAuxMotor;
        break;
      case 4: // 左电机修正值+1
        settings.motorOffset = min(settings.motorOffset + 1, (int8_t)32);
        break;
    }
    lastPlusPress = millis();
  }

  // P3：减少当前设置项的值
  if (btnMinus == LOW && millis() - lastMinusPress > 300) {
    switch (currentSettingIndex) {
      case 0: // 遥控器频道
        settings.channel = max(settings.channel - 1, (uint8_t)1);
        break;
      case 1: // 电池节数
        settings.batteryType = (settings.batteryType == 0) ? 0 : max(settings.batteryType - 1, (uint8_t)0);
        break;
      case 2: // 电机类型切换
        settings.motorType = 1 - settings.motorType;
        break;
      case 3: // 辅助输出开关
        settings.useAuxMotor = 1 - settings.useAuxMotor;
        break;
      case 4: // 左电机修正值-1
        settings.motorOffset = max(settings.motorOffset - 1, (int8_t)-32);
        break;
    }
    lastMinusPress = millis();
  }

  // ----- 退出设置模式 -----
  // P0恢复高电平且已进入超过500ms（防抖）时退出
  if (p0State == HIGH && millis() - settingEnterTime > 500) {
    saveSettings();       // 保存设置到EEPROM
    inSettingMode = false;

    // 更新NRF24L01频道
    Mirf.channel = settings.channel;
    Mirf.config();

    // 短暂闪烁LED表示保存成功
    digitalWrite(LED_BUILTIN, HIGH);

    // 切换到有刷/无刷模式
    updateMotorMode();
    return;
  }

  // ----- 设置界面显示（仅在OLED可用时） -----
  if (oledAvailable) {
    displaySettingScreen();
  }
}

// 显示设置界面（独立函数，减少handleSetupMode的代码量）
void displaySettingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 首行：居中显示"Setting..."
  display.setCursor(35, 0);
  display.print(F("Setting..."));

  // 第二行：当前设置项名称和值
  display.setCursor(0, 8);
  switch (currentSettingIndex) {
    case 0: // 遥控器频道
      display.print(F(">Ch:"));
      display.print(settings.channel);
      break;
    case 1: // 电池节数
      display.print(F(">Bat:"));
      if (settings.batteryType == 0)
        display.print(F("Auto"));
      else {
        display.print(settings.batteryType);
        display.print(F("S"));
      }
      break;
    case 2: // 电机类型
      display.print(F(">Motor:"));
      display.print(settings.motorType ? F("BLDC") : F("BDC"));
      break;
    case 3: // 辅助输出
      display.print(F(">Aux:"));
      display.print(settings.useAuxMotor ? F("ON") : F("OFF"));
      break;
    case 4: // 电机修正值
      display.print(F(">Offset:"));
      display.print(settings.motorOffset);
      break;
  }

  // 第三行：电池电压（实时显示，方便判断电池节数）
  display.setCursor(0, 16);
  display.print(F("V:"));
  display.print(batteryVoltageMV / 1000);
  display.print(F("."));
  if (batteryVoltageMV % 1000 < 100) display.print('0');
  display.print(batteryVoltageMV % 1000 / 10);
  display.print(F("V "));

  // 第四行：操作提示
  display.setCursor(0, 24);
  display.print(F("P1:Sel P2:+ P3:-"));

  display.display();
}

/* ============================================================
 *  OLED显示 - 三屏切换
 * ============================================================ */

void displayInfo() {
  // 设置模式由handleSetupMode()自行显示，此处跳过
  if (inSettingMode) return;
  if (!oledAvailable) return;

  // 限流：每隔DISPLAY_INTERVAL才刷新一次，避免I2C占用过多时间
  unsigned long now = millis();
  if (now - lastDisplayTime < DISPLAY_INTERVAL) return;
  lastDisplayTime = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 获取当前电池类型（用于显示）
  uint8_t batType = settings.batteryType;
  if (batType == 0) {
    batType = autoDetectBattery(batteryVoltageMV);
  }

  switch (currentScreen) {
    case 0: // 第一屏：本地信息
      displayScreen0(batType);
      break;
    case 1: // 第二屏：远程遥控器数据
      displayScreen1();
      break;
    case 2: // 第三屏：传感器和状态
      displayScreen2();
      break;
  }

  display.display();
}

// 第一屏：本地信息
void displayScreen0(uint8_t batType) {
  // 首行：居中显示"Local"
  display.setCursor(48, 0);
  display.print(F("Local"));

  // 第二行：遥控器频道 + 电机类型
  display.setCursor(0, 8);
  display.print(F("Ch:"));
  display.print(settings.channel);
  display.print(F(" "));
  display.print(isBrushlessMode ? F("BLDC") : F("BDC"));

  // 第三行：电池电压 + 电池类型
  display.setCursor(0, 16);
  display.print(F("Bat:"));
  display.print(batteryVoltageMV / 1000);
  display.print(F("."));
  if (batteryVoltageMV % 1000 < 100) display.print('0');
  display.print(batteryVoltageMV % 1000 / 10);
  display.print(F("V "));
  display.print(batType);
  display.print(F("S"));

  // 第四行：最终输出的PWM值
  display.setCursor(0, 24);
  display.print(F("PWM L:"));
  display.print((int)finalLeftPWM);
  display.print(F(" R:"));
  display.print((int)finalRightPWM);
}

// 第二屏：远程遥控器数据
void displayScreen1() {
  // 首行：居中显示"from remoter"
  display.setCursor(21, 0);
  display.print(F("from remoter"));

  // 第二行：油门值(YM) + 方向值(FX)
  display.setCursor(0, 8);
  display.print(F("YM:"));
  display.print(rcThrottle);
  display.print(F(" FX:"));
  display.print(rcSteering);

  // 第三行：右摇杆Y轴(RY) + X轴(RX)
  display.setCursor(0, 16);
  display.print(F("RY:"));
  display.print(rcRightStickY);
  display.print(F(" RX:"));
  display.print(rcRightStickX);

  // 第四行：遥控器P0~P7开关状态（二进制显示）
  display.setCursor(0, 24);
  display.print(F("SW:"));
  for (int8_t i = 7; i >= 0; i--) {
    display.print(bitRead(rcInputSW, i));
  }
}

// 第三屏：传感器与状态信息
void displayScreen2() {
  // 首行：居中显示"Status"
  display.setCursor(42, 0);
  display.print(F("Status"));

  // 第二行：电流值（整数显示，单位0.1A）
  display.setCursor(0, 8);
  display.print(F("Cur:"));
  display.print(controlData.current / 10);
  display.print(F("."));
  display.print(controlData.current % 10);
  display.print(F("A"));

  // 第三行：陀螺仪Z轴（整数显示，单位0.01rad/s）
  display.setCursor(0, 16);
  display.print(F("Yaw:"));
  if (mpuAvailable) {
    int16_t yaw100 = (int16_t)(gyroEvent.gyro.z * 100);
    if (yaw100 < 0) { display.print('-'); yaw100 = -yaw100; }
    display.print(yaw100 / 100);
    display.print(F("."));
    if (yaw100 % 100 < 10) display.print('0');
    display.print(yaw100 % 100);
  } else {
    display.print(F("N/A"));
  }

  // 第四行：RF连接状态 + 8位开关位编码
  display.setCursor(0, 24);
  display.print(rfConnected ? F("RF:OK") : F("RF:--"));
  display.print(F(" SW:"));
  for (int8_t i = 7; i >= 0; i--) {
    display.print(bitRead(rcInputSW, i));
  }
}

/* ============================================================
 *  setup() - 初始化
 * ============================================================ */

void setup() {
  // 尽早启用看门狗，防止初始化过程中跑飞
  wdt_enable(WDTO_2S);
  wdt_reset();

  // ----- 基础初始化 -----
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if ENABLE_DEBUG
  Serial.begin(9600);
  Serial.println(F("=== Control Board v2.0 ==="));
#endif

  Wire.begin();

  // ----- 加载设置 -----
  loadSettings();
  DBG(F("Ch:")); DBGLN(settings.channel);

  // ----- 初始化OLED -----
  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(35, 12);
    display.print(F("Starting..."));
    display.display();
  }
  DBGLN(oledAvailable ? F("OLED:OK") : F("OLED:FAIL"));

  // ----- 初始化PCF8574 -----
  if (pcfIO.begin(0x20)) {
    DBGLN(F("PCF20:OK"));
  } else {
    DBGLN(F("PCF20:FAIL"));
  }

  if (pcfSetup.begin(0x21)) {
    DBGLN(F("PCF21:OK"));
  } else {
    DBGLN(F("PCF21:FAIL"));
  }

  // 配置PCF8574(0x20)引脚方向
  // P0~P3：输入（BTS7960电流反馈）
  // P4~P7：输出（辅助控制位）
  for (uint8_t i = 0; i < 4; i++) {
    pcfIO.pinMode(i, INPUT);
  }
  for (uint8_t i = 4; i < 8; i++) {
    pcfIO.pinMode(i, OUTPUT);
    pcfIO.digitalWrite(i, LOW);
  }

  // 配置PCF8574(0x21)引脚方向
  // 全部设为输入上拉（P0=进入设置，P1=切换项，P2=加，P3=减，P7=电机类型）
  for (uint8_t i = 0; i < 8; i++) {
    pcfSetup.pinMode(i, INPUT_PULLUP);
  }

  // ----- 初始化MPU6050 -----
  mpuAvailable = mpu.begin();
  if (mpuAvailable) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    DBGLN(F("MPU:OK"));
  } else {
    DBGLN(F("MPU:FAIL"));
  }

  // ----- 初始化NRF24L01 -----
  // ★ 已关闭auto-ACK (EN_AA=0)，无ACK等待/重试，通信更可靠
  // 双方使用相同地址：P0="SEND1", TX="SEND1"
  // 遥控器→控制板：发"SEND1"，控制板P0="SEND1" ✓
  // 控制板→遥控器：发"SEND1"，遥控器P0="SEND1" ✓
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.configRegister(EN_AA, 0x00);  // ★ 关闭auto-ACK！避免NRF卡死在重试中
  Mirf.setRADDR((byte *)"RECVE");
  Mirf.setTADDR((byte *)"SEND1");
  Mirf.payload = NRF_PAYLOAD_SIZE;
  Mirf.channel = settings.channel;   // 使用保存的频道
  Mirf.config();
  DBG(F("NRF Ch=")); DBGLN(settings.channel);

  // ----- 初始化遥控器数据（安全默认值） -----
  // buttons默认全0（未按下），joystick默认0（中位）
  memset(&remoteData, 0, sizeof(RemoteData));
  remoteData.channel = settings.channel;
  remoteData.throttleMax = 255;   // 默认值，与遥控器匹配
  remoteData.steeringMax = 255;

  // 初始化转换后的缓存变量（512=中位，0xFF=所有开关高电平/上拉）
  rcThrottle = 512;
  rcSteering = 512;
  rcRightStickY = 512;
  rcRightStickX = 512;
  rcInputSW = 0xFF;

  // ----- 初始化控制数据 -----
  memset(&controlData, 0, sizeof(ControlData));

  // ----- 初始化电机模式 -----
  // 根据当前设置确定初始模式
  int p7State = pcfSetup.digitalRead(7);
  isBrushlessMode = (p7State == LOW) ? true : (settings.motorType == 1);

  if (isBrushlessMode) {
    enterBrushlessMode();
    DBGLN(F("M:BLDC"));
  } else {
    enterBrushedMode();
    DBGLN(F("M:BDC"));
  }

  // ----- 就绪提示 -----
  if (oledAvailable) {
    display.clearDisplay();
    display.setCursor(45, 12);
    display.print(F("Ready!"));
    display.display();
  }

  // LED闪烁两次表示初始化完成
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(80);
    digitalWrite(LED_BUILTIN, LOW);
    delay(80);
  }

  // 初始化定时变量
  lastReceiveTime = millis();
  lastDisplayTime = millis();
  lastSendTime = millis();
  lastSensorTime = millis();

  DBGLN(F("Setup OK"));
}

/* ============================================================
 *  loop() - 主循环
 * ============================================================ */

void loop() {
  // 1. 读取传感器数据（限流：每SENSOR_INTERVAL毫秒一次）
  readSensors();

  // 2. 接收遥控器数据（含Fail-Safe检测）
  receiveRemoteData();

  // 3. 收到数据后立刻回传（请求-响应模式，不做任何处理先回传）
  sendControlData();

  // 4. 解析遥控器数据（回传后再处理，避免延迟）
  parseRemoteData();

  // 5. 处理设置模式（设置模式下禁止电机控制）
  handleSetupMode();

  // 6. 电机控制（核心逻辑）
  controlMotors();

  // 7. OLED显示更新（限流：每DISPLAY_INTERVAL毫秒一次）
  displayInfo();

  // 8. 喂看门狗
  wdt_reset();

  // 主循环不使用delay()，依靠各函数内部的限流机制
}
