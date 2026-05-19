/*
 * ============================================================
 *   STM32F103C8T6 车船模型控制板 程序（从Arduino Nano移植优化）
 * ============================================================
 * 功能说明：
 *   通过NRF24L01无线模块接收遥控器数据，控制有刷/无刷电机，
 *   使用0.91寸OLED(128x32)显示本地和远程信息，
 *   通过PCF8574扩展I/O，通过MPU6050实现直线行驶自修正，
 *   支持设置模式保存参数到EEPROM。
 *
 * ★ 迁移原因：Arduino的Servo库使用Timer1中断，与NRF24L01 SPI通信冲突；
 *   STM32F103C8T6拥有多个独立定时器，Servo库使用不同的定时器资源，
 *   与SPI外设完全独立，从根本上消除了冲突。
 *
 * 使用的库及版本（STM32duino / Arduino_Core_STM32）：
 *   - SPI.h            (内置)                - SPI通信
 *   - Wire.h           (内置)                - I2C通信
 *   - RF24.h           (TMRh20 v1.4.9)      - NRF24L01无线模块驱动（替代Mirf）
 *   - Servo.h          (STM32内置)           - 舵机/电调信号生成（原AVR Timer1寄存器操作改为标准库）
 *   - Adafruit_GFX.h   (v1.11.9)            - OLED图形库基础
 *   - Adafruit_SSD1306.h (v2.5.7)           - SSD1306 OLED驱动
 *   - Adafruit_PCF8574.h (v1.1.1)           - PCF8574 I2C扩展
 *   - Adafruit_MPU6050.h (v2.1.4)           - MPU6050六轴传感器
 *   - EEPROM.h          (STM32内置)          - 参数掉电保存（Flash模拟EEPROM）
 *   - IWatchdog.h       (STM32内置)          - 独立看门狗(IWDG)
 *
 * 开发者：用户
 * 日期：2026-05-13
 * 版本：v3.0（STM32移植优化版）
 *
 * ============================================================
 * STM32F103C8T6 引脚规划（原理说明）
 * ============================================================
 *
 * 【核心优化思路】
 * 1. 原Arduino Nano上Servo库占用Timer1产生50Hz中断，
 *    与NRF24L01 SPI通信冲突导致丢包/卡死。
 *    → STM32上Servo库使用独立定时器（如TIM1/TIM8），
 *    与SPI1外设完全独立，无任何冲突。
 * 2. 改为使用RF24库替代Mirf库，通信更可靠，跨平台支持更好。
 * 3. BLDC模式从Timer1寄存器操作改为标准Servo库，代码更清晰。
 * 4. ADC精度从10位(0-1023)提升为12位(0-4095)，电压/电流测量更精确。
 * 5. 主频从16MHz提升到72MHz，处理能力大幅增强，移除冗余delay。
 *
 * ----- SPI1 (NRF24L01无线模块，独立外设，无中断冲突) -----
 *   PA5 - SCK
 *   PA6 - MISO
 *   PA7 - MOSI
 *   PB0 - CSN
 *   PB1 - CE
 *
 * ----- I2C1 (OLED, PCF8574 x2, MPU6050 - 共用总线) -----
 *   PB6 - SCL
 *   PB7 - SDA
 *
 * ----- 电机控制（有刷模式使用TIM2硬件PWM PA0/PA1） -----
 *   PB3  - LEFT_DIR1  (有刷方向1) / BLDC模式: LEFT_ESC(Servo1)
 *   PB4  - LEFT_DIR2  (有刷方向2) / BLDC模式: 空闲(高阻)
 *   PB5  - RIGHT_DIR1 (有刷方向1) / BLDC模式: RIGHT_ESC(Servo2)
 *   PB8  - RIGHT_DIR2 (有刷方向2) / BLDC模式: 空闲(高阻)
 *   PA0  - LEFT_PWM   (有刷PWM左, TIM2_CH1, ~1kHz)
 *   PA1  - RIGHT_PWM  (有刷PWM右, TIM2_CH2, ~1kHz)
 *
 *   ★ BLDC模式引脚复用说明：
 *     有刷模式使用4个方向脚+2个PWM脚，共6个引脚
 *     无刷模式仅需2个引脚(PB3/PB5)输出50Hz电调信号(Servo库)
 *     PB3/PB5在两种模式下复用，无需额外接线
 *
 * ----- ADC输入（12位精度，参考电压3.3V） -----
 *   PA2  - 电池电压检测 (ADC12_IN2)
 *           分压电阻：91K上臂 + 33K下臂 + 10K限流
 *           分压比 = (91+33)/33 ≈ 3.7576
 *   PA3  - 电池电流检测 (ADC12_IN3)
 *           配合ACS712-30A电流传感器
 *
 * ----- 内置LED & 串口 -----
 *   PC13 - 板载LED (低电平点亮)
 *   PA9  - USART1_TX (可选调试串口)
 *   PA10 - USART1_RX
 *
 * ----- 外设I2C地址 -----
 *   OLED SSD1306    - 0x3C (0.91寸, 128x32)
 *   PCF8574(扩展IO)  - 0x20
 *   PCF8574(设置控制) - 0x21
 *   MPU6050          - 0x68
 *
 * ============================================================
 * 硬件连接说明（参考YaoDriverV3板）
 * ============================================================
 * 电机输出接口：
 *   有刷模式：
 *     PB3、PB4 - 左侧电机方向控制
 *     PB5、PB8 - 右侧电机方向控制
 *     PA0、PA1 - 左/右电机PWM输出（TIM2，约1kHz）
 *   无刷模式（使用Servo库，任意引脚均可）：
 *     PB3 - 左侧无刷电调信号（两个左电调信号线并接至此脚）
 *     PB5 - 右侧无刷电调信号（两个右电调信号线并接至此脚）
 *     ★ STM32的Servo库使用独立定时器，与SPI无冲突
 *
 * 电池电压检测：PA2（91K/33K电阻分压 + 10K限流电阻）
 * 电池电流检测：PA3
 *
 * PCF8574扩展IO（I2C）：
 *   地址0x20（pcfIO）：
 *     P0~P3 - 输入，复用BTS7960电流反馈
 *     P4~P7 - 输出，遥控器开关信号输出
 *   地址0x21（pcfSetup）：
 *     P4 - 输入，上拉，接地进入设置模式
 *     P5 - 输入，上拉，设置项切换
 *     P6 - 输入，上拉，设置项+操作
 *     P7 - 输入，上拉，设置项-操作
 *
 * ============================================================
 */

/* ============================================================
 *  头文件包含
 * ============================================================ */
#include <SPI.h>              // SPI通信（NRF24L01使用）
#include <Wire.h>             // I2C通信（OLED/PCF8574/MPU6050使用）
#include <RF24.h>             // NRF24L01无线模块驱动（替代原Mirf库）
#include <Servo.h>            // 舵机/电调信号（STM32上无Timer冲突）
#include <Adafruit_GFX.h>     // OLED图形库基础
#include <Adafruit_SSD1306.h> // SSD1306 OLED驱动
#include <Adafruit_PCF8574.h> // PCF8574 I2C扩展
#include <Adafruit_MPU6050.h> // MPU6050六轴传感器
#include <EEPROM.h>           // 参数保存（STM32上为Flash模拟EEPROM）
#include <IWatchdog.h>        // STM32独立看门狗(IWDG)

/* ============================================================
 *  引脚宏定义
 * ============================================================ */

// ----- NRF24L01 (SPI1) -----
#define NRF_CE      PB1       // NRF使能引脚
#define NRF_CSN     PB0       // NRF片选引脚

// ----- 电机引脚 -----
// ★ PB3/PB5 在有刷模式下作为方向控制，无刷模式下作为Servo电调信号输出
const uint8_t LEFT_DIR1  = PB3;   // 有刷：左侧方向1 / 无刷：左电调Servo
const uint8_t LEFT_DIR2  = PB4;   // 有刷：左侧方向2 / 无刷：空闲
const uint8_t RIGHT_DIR1 = PB5;   // 有刷：右侧方向1 / 无刷：右电调Servo
const uint8_t RIGHT_DIR2 = PB8;   // 有刷：右侧方向2 / 无刷：空闲
const uint8_t LEFT_PWM   = PA0;   // 有刷：左侧PWM (TIM2_CH1)
const uint8_t RIGHT_PWM  = PA1;   // 有刷：右侧PWM (TIM2_CH2)

// 无刷电调信号脚（与有刷方向脚复用PB3/PB5）
#define BLDC_LEFT_PIN    PB3     // 左电调Servo连接脚
#define BLDC_RIGHT_PIN   PB5     // 右电调Servo连接脚

// ----- 传感器ADC引脚 -----
// STM32的analogRead()返回12位值(0~4095)，参考电压3.3V
const uint8_t BATTERY_PIN = PA2;  // 电池电压检测 (ADC12_IN2)
const uint8_t CURRENT_PIN = PA3;  // 电流检测 (ADC12_IN3)

// ----- 板载LED -----
#define LED_PIN   PC13      // STM32 Blue Pill板载LED（低电平点亮）

// ----- OLED配置 -----
#define OLED_RESET   -1     // 无独立复位引脚
#define OLED_ADDR    0x3C   // OLED I2C地址

// ★ ★ ★ 关键兼容参数 ★ ★ ★
// 遥控器端使用Mirf库，sizeof(RemoteData)=21字节（AVR上int=2字节）
// 控制板使用RF24库，必须配置完全相同的参数才能与Mirf通信：
//   - Payload尺寸：21字节（与Mirf的sizeof(RemoteData)一致）
//   - 数据速率：1Mbps（Mirf默认配置RF_SETUP=0x06）
//   - 自动ACK：关闭（Mirf设置EN_AA=0x00）
//   - 回传数据也必须补齐到21字节（Mirf固定payload接收模式）
// ★ 结构体RemoteData使用int16_t替代int，在ARM上也保持2字节，与AVR兼容
#define NRF_PAYLOAD_SIZE   21

// ----- 电池自动识别阈值（单位：V） -----
#define BAT_1S_MAX   4.3f
#define BAT_2S_MAX   8.6f
#define BAT_3S_MAX   13.0f

// ----- 电机控制参数 -----
#define STEERING_DEADZONE    30    // 转向死区
#define RF_TIMEOUT           1000  // 遥控信号超时（ms）

// ----- EEPROM校验标识 -----
#define EEPROM_ADDR        0
#define EEPROM_MAGIC_BYTE  0xBC   // 与原版区分，防止读取旧A板数据

// ----- 定时参数 -----
#define DISPLAY_INTERVAL   200    // OLED刷新间隔（ms）
#define SENSOR_INTERVAL    100    // 传感器读取间隔（ms）

/* ============================================================
 *  数据结构定义
 * ============================================================ */

// ★ 注意：STM32上int为4字节，与AVR不同！
// 原Arduino代码中int=2字节，STM32上int=4字节
// 所有跨平台传输的结构体需使用明确大小的类型(uint8_t/int16_t/uint16_t)

// 设置参数结构体（保存在EEPROM中）
struct Settings {
  uint8_t magic;           // 校验标识
  uint8_t channel;         // 遥控器频道，默认90
  uint8_t batteryType;     // 电池节数：0=自动, 1~4=1S~4S
  uint8_t motorType;       // 电机类型：0=有刷(BDC), 1=无刷(BLDC)
  uint8_t useAuxMotor;     // 辅助输出：0=否, 1=是
  int8_t  motorOffset;     // 左电机PWM修正：-32~+32
} __attribute__((packed));  // 避免结构体填充

// 遥控器传来的数据结构（使用明确大小类型，跨平台兼容）
struct RemoteData {
  uint8_t buttons[8];      // 遥控器按钮状态，0=未按下，1=按下
  int16_t joystick[4];     // 摇杆数据：[0]左Y(油门),[1]左X(方向),[2]右X,[3]右Y
  uint8_t channel;         // 遥控器频道
  uint16_t throttleMax;    // 油门最大值
  uint16_t steeringMax;    // 转向最大值
} __attribute__((packed));  // 固定29字节

// 回传给遥控器的数据结构
struct ControlData {
  uint16_t batteryVoltage; // 电池电压（放大100倍，单位0.01V）
  uint16_t current;        // 电流（放大100倍，单位0.01A）
  int8_t   leftMotorPWM;   // 左侧电机PWM (-128~+127)
  int8_t   rightMotorPWM;  // 右侧电机PWM (-128~+127)
  int16_t  accelX;         // MPU6050 X轴加速度（放大100倍）
  int16_t  accelY;         // MPU6050 Y轴加速度（放大100倍）
  int16_t  accelZ;         // MPU6050 Z轴加速度（放大100倍）
} __attribute__((packed));  // 固定12字节

/* ============================================================
 *  全局对象
 * ============================================================ */

// ----- NRF24L01无线模块（RF24库，替代原Mirf） -----
RF24 radio(NRF_CE, NRF_CSN);

// ----- OLED显示屏(0.91寸, 128x32) -----
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);

// ----- PCF8574扩展IO -----
Adafruit_PCF8574 pcfIO;       // 地址0x20：P0~P3输入, P4~P7输出
Adafruit_PCF8574 pcfSetup;    // 地址0x21：全部输入上拉（设置控制）

// ----- MPU6050 -----
Adafruit_MPU6050 mpu;

// ----- 无刷电调Servo对象 -----
// STM32的Servo库使用独立定时器(TIM1/TIM8)，与SPI1完全独立，无冲突
Servo escLeft;      // 左侧无刷电调（PB3）
Servo escRight;     // 右侧无刷电调（PB5）

/* ============================================================
 *  全局变量
 * ============================================================ */

// ----- 数据 -----
Settings settings;
RemoteData remoteData;
ControlData controlData;

// ----- 状态标志 -----
bool inSettingMode = false;      // 设置模式
bool rfConnected = false;        // 遥控器信号连接状态
bool oledAvailable = false;      // OLED是否可用
bool mpuAvailable = false;       // MPU6050是否可用
bool isBrushlessMode = false;    // 当前是否为无刷模式
uint8_t currentSettingIndex = 0; // 设置项索引
uint8_t currentScreen = 0;       // 当前显示屏幕(0~2)

// ----- 电机输出值（用于显示和回传） -----
int8_t finalLeftPWM = 0;        // 最终左电机输出 -127~+127
int8_t finalRightPWM = 0;       // 最终右电机输出 -127~+127

// ----- 遥控器转换数据（0~1023格式，512=中位） -----
uint16_t rcThrottle = 512;       // 油门值
uint16_t rcSteering = 512;       // 方向值
uint16_t rcRightStickY = 512;    // 右摇杆Y
uint16_t rcRightStickX = 512;    // 右摇杆X
uint8_t  rcInputSW = 0xFF;       // P0~P7开关状态位编码

// ----- 定时变量 -----
unsigned long lastDisplayTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastReceiveTime = 0;
unsigned long settingEnterTime = 0;

// ----- MPU6050传感器事件 -----
sensors_event_t accelEvent, gyroEvent, tempEvent;

// ----- 电池电压缓存（单位mV） -----
uint16_t batteryVoltageMV = 0;
uint16_t lastAdcRaw = 0;  // ADC原始值（诊断用）

// ----- 调试开关 -----
#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
  #define DBG(x)      Serial.print(x)
  #define DBGLN(x)    Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

/* ============================================================
 *  设置参数 - 读取 / 保存 / 恢复默认
 * ============================================================ */

// 加载设置（从EEPROM读取）
void loadSettings() {
  EEPROM.get(EEPROM_ADDR, settings);
  if (settings.magic != EEPROM_MAGIC_BYTE ||
      settings.channel == 0 || settings.channel > 125) {
    restoreDefaultSettings();
  }
}

// 保存设置（写入EEPROM）
void saveSettings() {
  settings.magic = EEPROM_MAGIC_BYTE;
  EEPROM.put(EEPROM_ADDR, settings);
  DBGLN(F("Settings saved"));
}

// 恢复默认设置
void restoreDefaultSettings() {
  settings.magic = EEPROM_MAGIC_BYTE;
  settings.channel = 90;
  settings.batteryType = 0;     // 自动检测
  settings.motorType = 0;       // 有刷
  settings.useAuxMotor = 0;     // 不使用辅助输出
  settings.motorOffset = 0;     // 无修正
  saveSettings();
  DBGLN(F("Default settings restored"));
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

// 更新电机模式（由软件设置决定）
void updateMotorMode() {
  bool newMode = (settings.motorType == 1);

  if (newMode != isBrushlessMode) {
    if (newMode) {
      enterBrushlessMode();
    } else {
      enterBrushedMode();
    }
    isBrushlessMode = newMode;
    DBG(F("Motor mode: "));
    DBGLN(isBrushlessMode ? F("BLDC") : F("BDC"));
  }
}

// 进入有刷电机模式
void enterBrushedMode() {
  // 分离无刷电调Servo（释放PB3/PB5）
  escLeft.detach();
  escRight.detach();

  // 配置方向引脚为输出
  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // ★ STM32 analogWrite频率调整
  // 默认TIM2 PWM频率约1kHz，适合有刷电机
  // 如需调整频率可使用：analogWriteFrequency(LEFT_PWM, 1000); (STM32F1支持)

  // 初始状态：全部低电平（停止）
  digitalWrite(LEFT_DIR1, LOW);
  digitalWrite(LEFT_DIR2, LOW);
  digitalWrite(RIGHT_DIR1, LOW);
  digitalWrite(RIGHT_DIR2, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

// 进入无刷电机模式（使用Servo库，STM32上无Timer冲突）
// ★ STM32与AVR的关键区别：
//   AVR的Servo库使用Timer1中断，与SPI共用中断优先级，导致NRF通信异常
//   STM32的Servo库使用TIM1/TIM8（高级定时器），独立运行，
//   SPI1使用外设DMA或独立中断通道，完全不受Servo影响
void enterBrushlessMode() {
  // 停止有刷PWM输出
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // 有刷方向脚在BLDC模式下：PB4/PB8设为输入高阻
  pinMode(LEFT_DIR2, INPUT);
  pinMode(RIGHT_DIR2, INPUT);

  // 连接Servo到PB3/PB5（电调信号线）
  // attach(pin, minPulse, maxPulse)
  // 标准电调：1000us=后退最大, 1500us=中位停止, 2000us=前进最大
  escLeft.attach(BLDC_LEFT_PIN, 1000, 2000);
  escRight.attach(BLDC_RIGHT_PIN, 1000, 2000);
  IWatchdog.reload();              // 喂狗再延时
  delay(100);  // 给电调初始化时间（上电时需要短暂延时等待电调自检）

  // 输出中位脉冲（停止）
  escLeft.writeMicroseconds(1500);
  escRight.writeMicroseconds(1500);

  // PB3/PB5已由Servo库管理，不再手动控制
}

/* ============================================================
 *  电机停止
 * ============================================================ */

void stopMotors() {
  if (isBrushlessMode) {
    // 无刷：Servo输出中位脉冲（停止）
    escLeft.writeMicroseconds(1500);
    escRight.writeMicroseconds(1500);
  } else {
    // 有刷：方向引脚全低，PWM归零
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

  // ----- 读取电池电压（多次采样取平均）-----
  // STM32 ADC: 12位(0~4095), 参考电压3.3V(3300mV)
  // 分压比: (91K+33K)/33K = 124/33
  // Vbat(mV) = adcValue × 3300 / 4095 × (91+33)/33 = adcValue × 3300 × 124 / (4095 × 33)
  uint32_t adcSum = 0;
  for (uint8_t i = 0; i < 16; i++) {
    adcSum += analogRead(BATTERY_PIN);
  }
  uint16_t adcValue = adcSum / 16;  // 16次平均
  lastAdcRaw = adcValue;  // 保存原始值用于诊断
  batteryVoltageMV = (uint16_t)((uint32_t)adcValue * 3300UL * 124UL / (4095UL * 33UL));
  controlData.batteryVoltage = batteryVoltageMV / 10;  // 转为0.01V单位

  // ----- 读取电流 -----
  // ACS712-30A: 灵敏度66mV/A, 零点2500mV
  // Vout(mV) = adcValue × 3300 / 4095
  // I(0.01A) = |Vout - 2500| / 66 × 100
  adcValue = analogRead(CURRENT_PIN);
  int16_t currentMV = (int16_t)((uint32_t)adcValue * 3300UL / 4095UL);
  int16_t diff = currentMV - 2500;
  if (diff < 0) diff = -diff;
  controlData.current = (uint16_t)((uint32_t)diff * 100UL / 66UL);

  // ----- 读取MPU6050 -----
  if (mpuAvailable) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    controlData.accelX = (int16_t)(accelEvent.acceleration.x * 100);
    controlData.accelY = (int16_t)(accelEvent.acceleration.y * 100);
    controlData.accelZ = (int16_t)(accelEvent.acceleration.z * 100);
    IWatchdog.reload();  // I2C操作后喂狗
  }
}

/* ============================================================
 *  电机控制 - 核心逻辑
 * ============================================================ */

void controlMotors() {
  // 设置模式或信号丢失时停止电机
  if (inSettingMode || !rfConnected) {
    stopMotors();
    return;
  }

  // 更新电机模式
  updateMotorMode();

  // ----- 解析遥控器输入 -----
  int throttle = rcThrottle;
  int steering = rcSteering;

  int basePWM = throttle - 512;      // -512 ~ +511
  int steerOffset = steering - 512;  // -512 ~ +511

  basePWM = constrain(basePWM, -512, 511);

  // ----- 计算左右电机基础速度 -----
  int leftSpeed = abs(basePWM);
  int rightSpeed = abs(basePWM);

  // ----- 应用转向（差速转向） -----
  if (steerOffset > STEERING_DEADZONE) {
    rightSpeed = max(rightSpeed - abs(steerOffset), 0);
  } else if (steerOffset < -STEERING_DEADZONE) {
    leftSpeed = max(leftSpeed - abs(steerOffset), 0);
  }

  // ----- 应用电机偏差修正（仅修正左侧） -----
  leftSpeed = constrain(leftSpeed + settings.motorOffset * 4, 0, 512);

  // ----- 直线行驶自修正（MPU6050陀螺仪Z轴） -----
  if (abs(steerOffset) < STEERING_DEADZONE && mpuAvailable && abs(basePWM) > 20) {
    int16_t yaw100 = (int16_t)(gyroEvent.gyro.z * 100);
    int correction = (int)yaw100 * 30 / 100;
    leftSpeed  = constrain(leftSpeed + correction, 0, 512);
    rightSpeed = constrain(rightSpeed - correction, 0, 512);
  }

  // ----- 限制最终速度 -----
  leftSpeed  = constrain(leftSpeed, 0, 512);
  rightSpeed = constrain(rightSpeed, 0, 512);

  // ----- 根据电机模式输出 -----
  if (isBrushlessMode) {
    controlBrushlessMotors(basePWM, leftSpeed, rightSpeed);
  } else {
    controlBrushedMotors(basePWM, leftSpeed, rightSpeed);
  }

  // ----- 遥控器4位按钮信号输出(PCF8574 0x20 P4~P7) -----
  for (uint8_t i = 0; i < 4; i++) {
    bool state = bitRead(rcInputSW, i);
    pcfIO.digitalWrite(i + 4, state ? HIGH : LOW);
  }
}

// 有刷电机控制
void controlBrushedMotors(int basePWM, int leftSpeed, int rightSpeed) {
  // 将0~512速度映射到0~255的PWM占空比
  uint8_t leftPWM  = (uint8_t)map(leftSpeed, 0, 512, 0, 255);
  uint8_t rightPWM = (uint8_t)map(rightSpeed, 0, 512, 0, 255);

  if (basePWM > 5) {
    // 前进：DIR1高, DIR2低
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    analogWrite(LEFT_PWM, leftPWM);
    analogWrite(RIGHT_PWM, rightPWM);
    finalLeftPWM  = map(leftPWM, 0, 255, 0, 127);
    finalRightPWM = map(rightPWM, 0, 255, 0, 127);
  } else if (basePWM < -5) {
    // 后退：DIR1低, DIR2高
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    analogWrite(LEFT_PWM, leftPWM);
    analogWrite(RIGHT_PWM, rightPWM);
    finalLeftPWM  = -map(leftPWM, 0, 255, 0, 127);
    finalRightPWM = -map(rightPWM, 0, 255, 0, 127);
  } else {
    stopMotors();
  }

  controlData.leftMotorPWM  = finalLeftPWM;
  controlData.rightMotorPWM = finalRightPWM;
}

// 无刷电机控制（使用Servo库，STM32上无Timer中断冲突）
// ★ STM32的优势：Servo库使用TIM1（高级定时器）产生50Hz脉冲，
//   SPI1使用独立外设，两者互不干扰，NRF24L01通信稳定可靠
void controlBrushlessMotors(int basePWM, int leftSpeed, int rightSpeed) {
  // 将速度(0~512)映射为脉冲宽度偏移(0~500us)
  // 中位1500us=停止, >1500us=前进, <1500us=后退
  int leftOffset  = map(leftSpeed, 0, 512, 0, 500);
  int rightOffset = map(rightSpeed, 0, 512, 0, 500);

  int leftPulse, rightPulse;

  if (basePWM > 5) {
    leftPulse  = 1500 + leftOffset;
    rightPulse = 1500 + rightOffset;
    finalLeftPWM  = map(leftOffset, 0, 500, 0, 127);
    finalRightPWM = map(rightOffset, 0, 500, 0, 127);
  } else if (basePWM < -5) {
    leftPulse  = 1500 - leftOffset;
    rightPulse = 1500 - rightOffset;
    finalLeftPWM  = -map(leftOffset, 0, 500, 0, 127);
    finalRightPWM = -map(rightOffset, 0, 500, 0, 127);
  } else {
    leftPulse  = 1500;
    rightPulse = 1500;
    finalLeftPWM  = 0;
    finalRightPWM = 0;
  }

  // 限制脉冲宽度（1000~2000us）
  leftPulse  = constrain(leftPulse,  1000, 2000);
  rightPulse = constrain(rightPulse, 1000, 2000);

  // Servo库writeMicroseconds()输出指定宽度的脉冲
  // STM32上由TIM1硬件自动产生50Hz脉冲，无需CPU干预
  escLeft.writeMicroseconds(leftPulse);
  escRight.writeMicroseconds(rightPulse);

  controlData.leftMotorPWM  = finalLeftPWM;
  controlData.rightMotorPWM = finalRightPWM;
}

/* ============================================================
 *  遥控数据收发（RF24库，替代原Mirf库）
 * ============================================================
 *
 * 通信架构（与遥控器Mirf库完全匹配）：
 *   遥控器(Mirf) → 发送到 "SEND1" → 控制板(Pipe0) 接收
 *   控制板(RF24) → 发送到 "SEND1" → 遥控器(Pipe0) 接收
 *
 *   注意：遥控器Mirf.setRADDR("RECVE")设置RX_ADDR_P1="RECVE"，
 *   但Mirf.setTADDR("SEND1")会覆写RX_ADDR_P0="SEND1"。
 *   遥控器发送时，Mirf.send()内部powerUpTx()将CONFIG设为TX模式，
 *   发送完成后isSending()内部powerUpRx()恢复RX模式。
 *   遥控器的RX_ADDR_P0="SEND1"用于auto-ack（已禁用）。
 *
 *   控制板回传时，发送到"SEND1"即可，遥控器TX完成后回到RX模式，
 *   RX_ADDR_P0="SEND1"会匹配，但Pipe0的ACK已禁用，
 *   遥控器通过dataReady()检查STATUS的RX_DR位来接收。
 * ============================================================ */

static bool pendingReply = false;    // 有待回传的数据
static bool newDataArrived = false;  // 有新数据需要解析
static uint32_t rxLoopCount = 0;     // receiveRemoteData() 调用次数
static uint32_t lastRxReport = 0;    // 上次OLED报告时间

// NRF 地址常量
const uint8_t addr_SEND1[5] = {'S','E','N','D','1'};

void receiveRemoteData() {
  rxLoopCount++;
  if (radio.available()) {
    radio.read(&remoteData, sizeof(RemoteData));
    IWatchdog.reload();

    lastReceiveTime = millis();
    rfConnected = true;

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    pendingReply = true;
    newDataArrived = true;
  }

  if (millis() - lastReceiveTime > RF_TIMEOUT) {
    rfConnected = false;
    digitalWrite(LED_PIN, HIGH);
  }

  IWatchdog.reload();
}

// 解析遥控器数据
void parseRemoteData() {
  if (!newDataArrived) return;
  newDataArrived = false;

  // 转换buttons[8]为位编码
  rcInputSW = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (remoteData.buttons[i]) {
      bitSet(rcInputSW, i);
    }
  }

  // 转换摇杆偏移量为0~1023格式
  int throttleHalf = remoteData.throttleMax / 2;
  int steeringHalf = remoteData.steeringMax / 2;
  if (throttleHalf == 0) throttleHalf = 255;
  if (steeringHalf == 0) steeringHalf = 255;

  // joystick[0]=油门(Y轴), [1]=方向(X轴)
  // [2]=右X, [3]=右Y
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
    currentScreen = 0;              // P6为低 = 第一屏
  } else if (p6 && p7) {
    currentScreen = 1;              // P6,P7同时高 = 第二屏
  } else if (!p7) {
    currentScreen = 2;              // P7为低 = 第三屏
  }
}

// 回传控制数据（必须发送NRF_PAYLOAD_SIZE=21字节，兼容遥控器Mirf固定payload接收）
void sendControlData() {
  if (!pendingReply) return;
  pendingReply = false;

  uint8_t txBuf[NRF_PAYLOAD_SIZE];
  memset(txBuf, 0, NRF_PAYLOAD_SIZE);
  memcpy(txBuf, &controlData, sizeof(ControlData));

  // ★ 使用RF24库API回传（发送到遥控器TX_ADDR/RX_ADDR_P0="SEND1"）
  radio.stopListening();

  // 发送到"SEND1"（遥控器TX完成后恢复RX模式时，RX_ADDR_P0="SEND1"）
  radio.openWritingPipe(addr_SEND1);
  radio.write(txBuf, NRF_PAYLOAD_SIZE);

  // 恢复RX模式：Pipe0="SEND1"接收，Pipe1="RECVE"接收
  radio.openWritingPipe(addr_SEND1);          // 设置TX_ADDR+RX_ADDR_P0="SEND1"
  const uint8_t addr_RECVE[5] = {'R','E','C','V','E'};
  radio.openReadingPipe(1, addr_RECVE);       // Pipe1="RECVE"
  radio.startListening();                      // _is_p0_rx=true，Pipe0保持使能

  IWatchdog.reload();
}

/* ============================================================
 *  设置模式处理
 * ============================================================ */

void handleSetupMode() {
  // 读取PCF8574(0x21)的P4~P7（上拉，闭合到GND为低电平）
  int p4State = pcfSetup.digitalRead(4);   // 进入设置
  int p5State = pcfSetup.digitalRead(5);   // 切换项
  int p6State = pcfSetup.digitalRead(6);   // +
  int p7State = pcfSetup.digitalRead(7);   // -
  IWatchdog.reload();  // I2C操作后喂狗

  // ----- 进入设置模式 -----
  if (!inSettingMode && p4State == LOW) {
    inSettingMode = true;
    currentSettingIndex = 0;
    settingEnterTime = millis();
    stopMotors();
    digitalWrite(LED_PIN, LOW);  // LED常亮提示进入设置
    return;
  }

  if (!inSettingMode) return;

  // ----- 防抖定时变量（静态） -----
  static unsigned long lastSelectPress = 0;
  static unsigned long lastPlusPress   = 0;
  static unsigned long lastMinusPress  = 0;

  // P5：切换设置项
  if (p5State == LOW && millis() - lastSelectPress > 300) {
    currentSettingIndex = (currentSettingIndex + 1) % 5;
    lastSelectPress = millis();
  }

  // P6：增加当前项的值
  if (p6State == LOW && millis() - lastPlusPress > 300) {
    switch (currentSettingIndex) {
      case 0: // 遥控器频道
        settings.channel = (uint8_t)min((int)settings.channel + 1, 125);
        break;
      case 1: // 电池节数（0=自动, 1~4=手动）
        settings.batteryType = (uint8_t)min((int)settings.batteryType + 1, 4);
        break;
      case 2: // 电机类型切换
        settings.motorType = 1 - settings.motorType;
        break;
      case 3: // 辅助输出开关
        settings.useAuxMotor = 1 - settings.useAuxMotor;
        break;
      case 4: // 左电机修正值+1
        settings.motorOffset = (int8_t)min((int)settings.motorOffset + 1, 32);
        break;
    }
    lastPlusPress = millis();
  }

  // P7：减少当前项的值
  if (p7State == LOW && millis() - lastMinusPress > 300) {
    switch (currentSettingIndex) {
      case 0:
        settings.channel = (uint8_t)max((int)settings.channel - 1, 1);
        break;
      case 1:
        settings.batteryType = (settings.batteryType == 0) ? 0 : (uint8_t)max((int)settings.batteryType - 1, 0);
        break;
      case 2:
        settings.motorType = 1 - settings.motorType;
        break;
      case 3:
        settings.useAuxMotor = 1 - settings.useAuxMotor;
        break;
      case 4:
        settings.motorOffset = (int8_t)max((int)settings.motorOffset - 1, -32);
        break;
    }
    lastMinusPress = millis();
  }

  // ----- 退出设置模式 -----
  if (p4State == HIGH && millis() - settingEnterTime > 500) {
    saveSettings();
    inSettingMode = false;

    // 更新NRF24L01频道
    radio.setChannel(settings.channel);

    // 更新电机模式
    updateMotorMode();

    // LED熄灭
    digitalWrite(LED_PIN, HIGH);

    DBG(F("Settings saved, Ch="));
    DBGLN(settings.channel);
    IWatchdog.reload();
    return;
  }

  // ----- 显示设置界面 -----
  if (oledAvailable) {
    displaySettingScreen();
    IWatchdog.reload();
  }
}

// 显示设置界面
void displaySettingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 首行：居中显示"Setting..."
  display.setCursor(35, 0);
  display.print(F("Setting..."));

  // 第二行：当前设置项
  display.setCursor(0, 8);
  switch (currentSettingIndex) {
    case 0:
      display.print(F(">Ch:"));
      display.print(settings.channel);
      break;
    case 1:
      display.print(F(">Bat:"));
      if (settings.batteryType == 0)
        display.print(F("Auto"));
      else {
        display.print(settings.batteryType);
        display.print(F("S"));
      }
      break;
    case 2:
      display.print(F(">Motor:"));
      display.print(settings.motorType ? F("BLDC") : F("BDC"));
      break;
    case 3:
      display.print(F(">Aux:"));
      display.print(settings.useAuxMotor ? F("ON") : F("OFF"));
      break;
    case 4:
      display.print(F(">Offset:"));
      display.print(settings.motorOffset);
      break;
  }

  // 第三行：电池电压（实时显示）
  display.setCursor(0, 16);
  display.print(F("V:"));
  display.print(batteryVoltageMV / 1000);
  display.print(F("."));
  if (batteryVoltageMV % 1000 < 100) display.print('0');
  display.print(batteryVoltageMV % 1000 / 10);
  display.print(F("V "));

  // 第四行：操作提示
  display.setCursor(0, 24);
  display.print(F("P5:Sel P6:+ P7:-"));

  display.display();
}

/* ============================================================
 *  OLED显示 - 三屏切换
 * ============================================================ */

void displayInfo() {
  if (inSettingMode) return;    // 设置模式由handleSetupMode自己显示
  if (!oledAvailable) return;

  // 限流刷新
  unsigned long now = millis();
  if (now - lastDisplayTime < DISPLAY_INTERVAL) return;
  lastDisplayTime = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // 获取电池类型
  uint8_t batType = settings.batteryType;
  if (batType == 0) {
    batType = autoDetectBattery(batteryVoltageMV);
  }

  switch (currentScreen) {
    case 0: displayScreen0(batType); break;
    case 1: displayScreen1(); break;
    case 2: displayScreen2(); break;
  }

  display.display();
  IWatchdog.reload();  // OLED I2C操作后喂狗
}

// 第一屏：本地信息
void displayScreen0(uint8_t batType) {
  display.setCursor(38, 0);
  display.print(F("Local"));
  // 右侧显示RF连接状态 "OK" 或 "--"
  display.setCursor(98, 0);
  if (rfConnected) {
    display.print(F("OK"));
  } else {
    display.print(F("--"));
  }

  display.setCursor(0, 8);
  display.print(F("Ch:"));
  display.print(settings.channel);
  display.print(F(" "));
  display.print(isBrushlessMode ? F("BLDC") : F("BDC"));

  display.setCursor(0, 16);
  display.print(F("Bat:"));
  display.print(batteryVoltageMV / 1000);
  display.print(F("."));
  if (batteryVoltageMV % 1000 < 100) display.print('0');
  display.print(batteryVoltageMV % 1000 / 10);
  display.print(F("V "));
  display.print(batType);
  display.print(F("S"));

  display.setCursor(0, 24);
  display.print(F("PWM L:"));
  display.print((int)finalLeftPWM);
  display.print(F(" R:"));
  display.print((int)finalRightPWM);
}

// 第二屏：远程遥控器数据
void displayScreen1() {
  display.setCursor(21, 0);
  display.print(F("from remoter"));

  display.setCursor(0, 8);
  display.print(F("YM:"));
  display.print(rcThrottle);
  display.print(F(" FX:"));
  display.print(rcSteering);

  display.setCursor(0, 16);
  display.print(F("RY:"));
  display.print(rcRightStickY);
  display.print(F(" RX:"));
  display.print(rcRightStickX);

  display.setCursor(0, 24);
  display.print(F("SW:"));
  for (int8_t i = 7; i >= 0; i--) {
    display.print(bitRead(rcInputSW, i));
  }
}

// 第三屏：ADC诊断信息
void displayScreen2() {
  display.setCursor(30, 0);
  display.print(F("ADC Diag"));

  // 行1: 电池电压
  display.setCursor(0, 8);
  display.print(F("Vbat:"));
  display.print(batteryVoltageMV / 1000);
  display.print(F("."));
  if (batteryVoltageMV % 1000 < 100) display.print('0');
  display.print(batteryVoltageMV % 1000 / 10);
  display.print(F("V"));

  // 行2: ADC原始值
  display.setCursor(0, 16);
  display.print(F("ADC:"));
  display.print(lastAdcRaw);
  display.print(F("/4095"));

  // 行3: 理论ADC值
  display.setCursor(0, 24);
  display.print(F("Exp:"));
  display.print(batteryVoltageMV * 33 / 124);  // 反推理论ADC
}

/* ============================================================
 *  setup() - 初始化
 * ============================================================ */

void setup() {
  // ----- 尽早启用看门狗 -----
  // STM32独立看门狗(IWDG)：使用内部低速时钟(40kHz)，独立于主时钟
  // 即使主时钟故障，看门狗仍能工作
  // 参数为超时时间，单位微秒
  IWatchdog.begin(8000000);  // 8秒超时（调试用，确认通信正常后改回2秒）
  IWatchdog.reload();

  // ----- 基础初始化 -----
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // 高电平=LED灭（Blue Pill PC13低电平点亮）

  // ----- ADC配置 -----
  // STM32 ADC时钟建议 ≤14MHz，主频72MHz时需要6分频
  analogReadResolution(12);  // 12位精度（0~4095）

#if ENABLE_DEBUG
  Serial.begin(115200);  // STM32主频72MHz，可用更高波特率
  Serial.println(F("=== Control Board STM32 v3.0 ==="));
#endif

  Wire.begin();  // PB6=SCL, PB7=SDA (I2C1)
  SPI.begin();   // PA5=SCK, PA6=MISO, PA7=MOSI (SPI1) - NRF24L01所需

  // ----- 加载设置 -----
  loadSettings();
  DBG(F("Ch:")); DBGLN(settings.channel);

  // ----- 初始化OLED -----
  // STM32上display.begin()内部使用Wire，无需额外配置SPI/I2C脚
  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledAvailable) {
    // STM32上SSD1306可能需要调整I2C速度
    // Wire.setClock(400000);  // 可选：提升I2C到400kHz快速模式
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(35, 12);
    display.print(F("Starting..."));
    display.display();
  }
  DBGLN(oledAvailable ? F("OLED:OK") : F("OLED:FAIL"));

  // ----- 初始化PCF8574(0x20) - 扩展IO -----
  if (pcfIO.begin(0x20)) {
    DBGLN(F("PCF20:OK"));
  } else {
    DBGLN(F("PCF20:FAIL"));
  }

  // 配置P0~P3为输入（BTS7960电流反馈）
  for (uint8_t i = 0; i < 4; i++) {
    pcfIO.pinMode(i, INPUT);
  }
  // 配置P4~P7为输出（辅助控制位），初始低电平
  for (uint8_t i = 4; i < 8; i++) {
    pcfIO.pinMode(i, OUTPUT);
    pcfIO.digitalWrite(i, LOW);
  }

  // ----- 初始化PCF8574(0x21) - 设置控制输入 -----
  if (pcfSetup.begin(0x21)) {
    DBGLN(F("PCF21:OK"));
  } else {
    DBGLN(F("PCF21:FAIL"));
  }

  // 全部设为输入上拉
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

  // ----- 初始化NRF24L01（纯RF24库API）-----
  // SPI1引脚：PA5(SCK), PA6(MISO), PA7(MOSI)
  // CE=PB1, CSN=PB0

  delay(50);
  if (radio.begin()) {
    DBGLN(F("NRF:begin OK"));
  } else {
    DBGLN(F("NRF:begin FAIL"));
  }

  // ★ 基础配置（按顺序，与Mirf完全匹配）
  radio.setAutoAck(false);                     // EN_AA = 0x00（Mirf: configRegister(EN_AA, 0)）
  radio.setChannel(settings.channel);          // RF_CH（Mirf: configRegister(RF_CH, channel)）
  radio.setPayloadSize(NRF_PAYLOAD_SIZE);      // 21字节（Mirf: configRegister(RX_PW_P0/P1, payload)）
  radio.setDataRate(RF24_2MBPS);               // ★★★ 2Mbps！（Mirf库默认RF_SETUP=0x0E=2Mbps）
  radio.setPALevel(RF24_PA_MAX);               // 0dBm最大发射功率
  radio.setCRCLength(RF24_CRC_8);              // 1字节CRC（Mirf: mirf_CONFIG = EN_CRC|CRCO_0）

  // ★ 地址设置（关键！）
  const uint8_t addr_RECVE[5] = {'R','E','C','V','E'};
  radio.openWritingPipe(addr_SEND1);           // TX_ADDR = RX_ADDR_P0 = "SEND1"
  radio.openReadingPipe(0, addr_SEND1);        // ★ 设置 _is_p0_rx = true！
  radio.openReadingPipe(1, addr_RECVE);        // RX_ADDR_P1 = "RECVE"，使能 Pipe 1

  radio.flush_rx();
  radio.flush_tx();
  radio.startListening();                       // 进入RX模式（Pipe 0+1 都保持使能）

  DBGLN(F("NRF OK"));

  // ----- 初始化数据变量 -----
  memset(&remoteData, 0, sizeof(RemoteData));
  memset(&controlData, 0, sizeof(ControlData));
  remoteData.channel = settings.channel;
  remoteData.throttleMax = 255;
  remoteData.steeringMax = 255;

  rcThrottle = 512;
  rcSteering = 512;
  rcRightStickY = 512;
  rcRightStickX = 512;
  rcInputSW = 0xFF;

  // ----- 初始化电机模式 -----
  isBrushlessMode = (settings.motorType == 1);
  if (isBrushlessMode) {
    enterBrushlessMode();
    DBGLN(F("M:BLDC"));
  } else {
    enterBrushedMode();
    DBGLN(F("M:BDC"));
  }

  // ----- 初始化定时器变量 -----
  lastReceiveTime = millis();
  lastDisplayTime = millis();
  lastSensorTime = millis();

  // ----- 就绪提示 -----
  if (oledAvailable) {
    display.clearDisplay();
    display.setCursor(45, 12);
    display.print(F("Ready!"));
    display.display();
  }

  // LED闪烁两次表示初始化完成
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, LOW);   // 亮
    IWatchdog.reload();
    delay(80);
    digitalWrite(LED_PIN, HIGH);  // 灭
    IWatchdog.reload();
    delay(80);
  }

  DBGLN(F("Setup OK"));
}

/* ============================================================
 *  loop() - 主循环
 * ============================================================
 * 优化说明：
 * 1. 移除了原版末尾的delay(10)，STM32主频72MHz不需要此降速措施
 * 2. NRF24L01通信使用RF24库，不再需要Mirf的手动配置
 * 3. BLDC使用Servo库（硬件定时器驱动），无中断冲突风险
 * 4. 所有非阻塞定时器保持不变
 */

void loop() {
  // 1. 读取传感器（限流：每SENSOR_INTERVAL毫秒一次）
  readSensors();

  // 2. 接收遥控器数据（含Fail-Safe检测）
  receiveRemoteData();

  // 3. 收到数据后立即回传（请求-响应模式）
  sendControlData();

  // 4. 解析遥控器数据
  parseRemoteData();

  // 5. 处理设置模式
  handleSetupMode();

  // 6. 电机控制
  controlMotors();

  // 7. OLED显示更新
  displayInfo();

  // 8. 喂看门狗
  IWatchdog.reload();
}
