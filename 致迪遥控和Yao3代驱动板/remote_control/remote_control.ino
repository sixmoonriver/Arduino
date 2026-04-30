/*
 * Arduino Pro Mini 车船模型遥控器
 * 使用NRF24L01无线模块进行数据传输
 * 使用0.91寸128x32 OLED显示屏显示数据
 * 使用PCF8574扩展IO口
 * 
 * 使用的库：
 * - SPI.h (Arduino内置)
 * - Wire.h (Arduino内置)
 * - Mirf.h
 * - MirfHardwareSpiDriver.h
 * - Adafruit_GFX.h (v1.11.9)
 * - Adafruit_SSD1306.h (v2.5.7)
 * - Adafruit_PCF8574.h (v1.1.1)
 * - avr/wdt.h (Arduino内置)
 * - EEPROM.h (Arduino内置)
 * 
 * 开发者：用户
 * 日期：2026-04-27
 * 
 * 硬件连接：
 * A0 → 右X电位器中点
 * A1 → 左Y电位器中点
 * A2 → 右Y电位器中点
 * A3 → 左X电位器中点
 * A6 → 电池电压检测
 * A7 → 镜头控制右
 * D2 → PCF8574中断输入
 * D6 → LED显示
 * 
 * NRF24L01连接：
 * D7 → CSN
 * D8 → CE
 * D11 → MOSI
 * D12 → MISO
 * D13 → SCK
 * 
 * OLED连接：
 * A4 → SDA
 * A5 → SCL
 * OLED I2C地址：0x3c
 * 
 * PCF8574连接：
 * A4 → SDA
 * A5 → SCL
 * I2C地址：0x27
 * P0 → 左前轻触式开关
 * P1 → 右前轻触式开关
 * P2 → 中左轻触式开关
 * P3 → 中右轻触式开关
 * P4 → 右侧3档开关（内）
 * P5 → 右侧3档开关（外）
 * P6 → 左侧3档开关（外）
 * P7 → 左侧3档开关（内）
 */

#include <SPI.h>
#include <Wire.h>
#include <Mirf.h>
#include <MirfHardwareSpiDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PCF8574.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// OLED显示配置
#define OLED_RESET 4
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);

// PCF8574配置
Adafruit_PCF8574 pcf; // I2C地址在begin()中设置

// NRF24L01配置
#define CE_PIN 8
#define CSN_PIN 7

// EEPROM地址定义
#define EEPROM_ADDR 0

// 设置结构体
struct Settings {
  uint8_t channel;       // 遥控器使用频道，默认90
  uint16_t sampleMax;    // 采样最大值，默认1024
  uint8_t batteryType;   // 电池类型：1=1s, 2=2s, 3=3s
  uint16_t throttleMax;  // 油门最大值
  uint16_t steeringMax;  // 转向最大值
  uint8_t joystickDir[4]; // 摇杆方向：0=正，1=反（左Y, 左X, 右X, 右Y）
};

// 引脚定义
const int LEFT_JOY_Y = A1;     // 左Y电位器中点
const int LEFT_JOY_X = A3;     // 左X电位器中点
const int RIGHT_JOY_X = A0;    // 右X电位器中点
const int RIGHT_JOY_Y = A2;    // 右Y电位器中点
const int BATTERY_PIN = A6;     // 电池电压检测
const int CAMERA_RIGHT = A7;    // 镜头控制右
const int LED_PIN = 6;          // LED显示
const int PCF_INTERRUPT = 2;    // PCF8574中断输入

// PCF8574引脚定义
const int PCF_BUTTON_P0 = 0;    // 左前轻触式开关
const int PCF_BUTTON_P1 = 1;    // 右前轻触式开关
const int PCF_BUTTON_P2 = 2;    // 中左轻触式开关
const int PCF_BUTTON_P3 = 3;    // 中右轻触式开关
const int PCF_SWITCH_P4 = 4;    // 右侧3档开关（内）
const int PCF_SWITCH_P5 = 5;    // 右侧3档开关（外）
const int PCF_SWITCH_P6 = 6;    // 左侧3档开关（外）
const int PCF_SWITCH_P7 = 7;    // 左侧3档开关（内）

// 设置相关变量
Settings settings;
bool inSettingMode = false;
int currentSettingIndex = 0;
unsigned long settingEnterTime = 0;
unsigned long p0PressTime = 0;

// PCF8574中断标志
volatile bool pcfChanged = false;

// PCF8574按键状态
int pcfButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
int lastPcfButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
int pcfSwitchState[4] = {HIGH, HIGH, HIGH, HIGH};

// 按键时间戳
unsigned long lastDebounceTime[4] = {0, 0, 0, 0};

// 防抖动延迟
const unsigned long debounceDelay = 50;

// 当前屏幕
int currentScreen = 0;

// OLED刷新控制
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 150; // 150ms刷新一次（~6.7fps）

// 数据结构
struct RemoteData {
  uint8_t buttons[8];    // 遥控器按钮状态（8个）
  int joystick[4];       // 摇杆数据：[0]左Y, [1]左X, [2]右X, [3]右Y
  uint8_t channel;       // 遥控器频道
  uint16_t throttleMax;  // 油门最大值
  uint16_t steeringMax;  // 转向最大值
};

struct RemoteResponse {
  uint16_t batteryVoltage; // 电池电压（放大100倍存储）
  uint16_t current;        // 电流（放大100倍存储）
  int8_t leftMotorPWM;     // 左侧电机PWM (-128~127)
  int8_t rightMotorPWM;    // 右侧电机PWM (-128~127)
  int16_t accelX;          // MPU6050 X轴加速度（放大1000倍）
  int16_t accelY;          // MPU6050 Y轴加速度（放大1000倍）
  int16_t accelZ;          // MPU6050 Z轴加速度（放大1000倍）
};

RemoteData data = {0};          // 初始化所有字段为0
RemoteResponse response = {0};    // 初始化所有字段为0
bool responseValid = false;       // 远端数据是否有效

// 加载设置
void loadSettings() {
  // 从EEPROM读取设置
  EEPROM.get(EEPROM_ADDR, settings);
  
  // 检查是否是首次使用或数据损坏（设置值不合理）
  if (settings.channel == 0 || settings.channel > 125 || 
      settings.sampleMax < 100 || settings.sampleMax > 1024 ||
      settings.throttleMax < 100 || settings.throttleMax > 1024 ||
      settings.steeringMax < 100 || settings.steeringMax > 1024 ||
      settings.batteryType < 1 || settings.batteryType > 3) {
    // 设置默认值
    settings.channel = 90;
    settings.sampleMax = 768;
    settings.batteryType = 1;
    settings.throttleMax = 255;
    settings.steeringMax = 255;
    for (int i = 0; i < 4; i++) {
      settings.joystickDir[i] = 0;
    }
    
    // 保存默认设置
    saveSettings();
  }
}

// 保存设置
void saveSettings() {
  EEPROM.put(EEPROM_ADDR, settings);
}

// PCF8574中断服务程序
void pcfISR() {
  pcfChanged = true;
}

bool oledAvailable = false; // OLED是否可用

void setup() {
  // 初始化LED引脚
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // 初始化串口
  Serial.begin(9600);
  delay(100);
  Serial.println("RC Controller v1.0");
  
  // 初始化Wire（I2C）
  Wire.begin();
  
  // 加载设置
  loadSettings();
  
  // 初始化OLED
  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Init...");
    display.display();
  }
  
  // 初始化PCF8574
  if (pcf.begin(0x27)) {
    for (uint8_t p = 0; p < 8; p++) {
      pcf.pinMode(p, INPUT_PULLUP);
    }
    // 配置D2中断（PCF8574 INT引脚，低电平有效）
    pinMode(PCF_INTERRUPT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PCF_INTERRUPT), pcfISR, FALLING);
  }
  
  // 初始化NRF24L01
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.configRegister(EN_AA, 0x00);  // ★ 关闭auto-ACK！避免NRF卡死在重试中
  Mirf.setRADDR((byte *)"RECVE");
  Mirf.setTADDR((byte *)"SEND1");
  Mirf.payload = sizeof(RemoteData);
  
  if (settings.channel == 0 || settings.channel > 125) {
    settings.channel = 90;
  }
  Mirf.channel = settings.channel;
  Mirf.config();
  
  // 初始化数据（显式清零，确保buttons不残留随机值）
  memset(&data, 0, sizeof(data));
  data.channel = settings.channel;
  data.throttleMax = settings.throttleMax;
  data.steeringMax = settings.steeringMax;
  
  // 初始化响应数据
  memset(&response, 0, sizeof(response));
  
  // 显示Ready
  if (oledAvailable) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Ready!");
    display.setCursor(0, 8);
    display.print("Ch:"); display.println(settings.channel);
    display.display();
  }
  
  // LED闪烁表示完成
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  
  // 启用看门狗
  wdt_enable(WDTO_2S);
  
  Serial.print("Ch:");
  Serial.println(settings.channel);
  Serial.println("Send 's' for settings");
}

void loop() {
  // 检查串口命令（用于测试设置模式）
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      Serial.println("Manual entry to setting mode!");
      inSettingMode = true;
      currentSettingIndex = 0;
      p0PressTime = millis();
      lastDisplayTime = 0; // 立即刷新
    }
    // 校准命令：发送 'c' 启动校准
    else if (cmd == 'c' || cmd == 'C') {
      Serial.println("Starting calibration mode...");
      calibrateJoysticks();
    }
  }

  // 读取电池电压
  readBatteryVoltage();

  // 读取按键状态
  readButtons();

  // 读取摇杆值
  readJoysticks();

  // 发送数据（限流50ms，给控制板处理+回传时间）
  static unsigned long lastTxTime = 0;
  if (millis() - lastTxTime >= 50) {
    lastTxTime = millis();
    sendData();
  }

  // 接收数据（始终读取，防止FIFO溢出）
  receiveData();

  // 显示当前屏幕
  displayScreen();

  // 喂狗
  wdt_reset();

  delay(5);
}

float localBattery = 0.00; // 本地电池电压
unsigned long lastBatteryCheck = 0; // 上次电池检查时间

// 读取电池电压（非阻塞）
void readBatteryVoltage() {
  // 每200ms检查一次电池
  if (millis() - lastBatteryCheck < 200) return;
  lastBatteryCheck = millis();
  
  int sensorValue = analogRead(BATTERY_PIN);
  
  // 电池电压计算
  // 分压电路：两个470K电阻串联，检测中间节点，4.7K电阻到引脚
  // 分压比 ≈ 0.5，所以实际电压 = 检测电压 * 2
  localBattery = sensorValue * (5.00 / 1023.0) * 2;
  
  // 电池低电压检测（使用非阻塞方式）
  // 1S锂电池：标称3.7V，充满4.2V，放电截止3.0V
  static unsigned long lastLedToggle = 0;
  static bool ledState = HIGH;
  
  if (localBattery < 3.5 && localBattery > 0.5) { // 电池低于3.5V（1S低电压）
    // 低电压时LED闪烁
    if (millis() - lastLedToggle > 200) {
      lastLedToggle = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    // 正常电压时LED关闭（低电平点亮，所以HIGH是关闭）
    digitalWrite(LED_PIN, HIGH);
  }
}

// 读取按键状态（带防抖动）
void readButtons() {
  // 一次读取PCF8574所有8个引脚（1次I2C事务代替8次）
  uint8_t pcfState = pcf.digitalReadByte();
  pcfChanged = false; // 读取后清除中断标志

  // 防抖处理按钮（P0-P3）
  for (int i = 0; i < 4; i++) {
    int reading = (pcfState >> i) & 1;
    if (reading != lastPcfButtonState[i]) {
      lastDebounceTime[i] = millis();
      lastPcfButtonState[i] = reading;
    }
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != pcfButtonState[i]) {
        pcfButtonState[i] = reading;
        
        if (i == 0 && pcfButtonState[i] == LOW) {
          p0PressTime = millis();
        }
      }
    }
  }
  
  // 正常模式下：P0-P3轻触翻转（toggle）
  // 注意：P0+P1同时按下是进入设置的手势，此时不翻转P0/P1
  if (!inSettingMode) {
    bool bothP0P1 = (pcfButtonState[0] == LOW && pcfButtonState[1] == LOW);
    for (int i = 0; i < 4; i++) {
      // P0/P1同时按下时跳过翻转
      if (bothP0P1 && (i == 0 || i == 1)) continue;
      static bool btnPrev[4] = {true, true, true, true};
      bool btnDown = (pcfButtonState[i] == LOW);
      if (btnDown && btnPrev[i]) {
        data.buttons[i] = !data.buttons[i]; // 翻转
      }
      btnPrev[i] = !btnDown;
    }
  }
  
  // 读取开关状态（P4-P7，无需防抖）
  pcfSwitchState[0] = (pcfState >> 4) & 1;
  pcfSwitchState[1] = (pcfState >> 5) & 1;
  pcfSwitchState[2] = (pcfState >> 6) & 1;
  pcfSwitchState[3] = (pcfState >> 7) & 1;
  
  // 将开关状态写入buttons数组（P4-P7）
  data.buttons[4] = (pcfSwitchState[0] == LOW) ? 1 : 0;
  data.buttons[5] = (pcfSwitchState[1] == LOW) ? 1 : 0;
  data.buttons[6] = (pcfSwitchState[2] == LOW) ? 1 : 0;
  data.buttons[7] = (pcfSwitchState[3] == LOW) ? 1 : 0;
  
  // 根据左侧3档开关切换屏幕（设置模式下忽略）
  if (!inSettingMode) {
    if (pcfSwitchState[2] == LOW) {
      if (currentScreen != 0) responseValid = false;
      currentScreen = 0;
    } else if (pcfSwitchState[2] == HIGH && pcfSwitchState[3] == HIGH) {
      currentScreen = 1;
    } else if (pcfSwitchState[3] == LOW) {
      if (currentScreen != 2) responseValid = false;
      currentScreen = 2;
    }
  }
  
  // 设置模式进入逻辑：同时按下P0和P1持续2秒
  if (!inSettingMode && pcfButtonState[0] == LOW && pcfButtonState[1] == LOW) {
    if (settingEnterTime == 0) {
      settingEnterTime = millis();
    } else if (millis() - settingEnterTime >= 2000) {
      inSettingMode = true;
      currentSettingIndex = 0;
      settingEnterTime = 0;
      p0PressTime = millis(); // 重置P0计时，防止立即触发长按保存
    }
  } else if (pcfButtonState[0] != LOW || pcfButtonState[1] != LOW) {
    settingEnterTime = 0;
  }
  
  // 处理设置模式
  if (inSettingMode) {
    // P0按下边沿：记录按下时间
    static bool p0PrevDown = false;
    bool p0Down = (pcfButtonState[0] == LOW);
    if (p0Down && !p0PrevDown) {
      p0PressTime = millis(); // 按下瞬间记录时间
    }
    p0PrevDown = p0Down;

    // P0长按2秒：保存设置并退出
    if (p0Down && (millis() - p0PressTime) >= 2000) {
      saveSettings();
      data.channel = settings.channel;
      data.throttleMax = settings.throttleMax;
      data.steeringMax = settings.steeringMax;
      Mirf.channel = settings.channel;
      Mirf.config();
      inSettingMode = false;
      p0PrevDown = false;
      lastDisplayTime = 0; // 立即刷新
    }

    // P0释放边沿：短按（<2秒）= 切换设置项
    static bool p0WasHeld = false;
    if (p0Down) {
      p0WasHeld = true;
    } else if (p0WasHeld) {
      // 刚释放
      p0WasHeld = false;
      if ((millis() - p0PressTime) < 2000) {
        currentSettingIndex = (currentSettingIndex + 1) % 9;
        lastDisplayTime = 0; // 立即刷新
      }
    }

    // P2按下边沿：+操作
    static bool p2Prev = true;
    bool p2Down = (pcfButtonState[2] == LOW);
    if (p2Down && p2Prev) {
      switch (currentSettingIndex) {
        case 0: settings.channel = min(settings.channel + 1, 125); break;
        case 1: settings.sampleMax = min(settings.sampleMax + 10, 1024); break;
        case 2: settings.batteryType = min(settings.batteryType + 1, 3); break;
        case 3: settings.throttleMax = min(settings.throttleMax + 10, 1024); break;
        case 4: settings.steeringMax = min(settings.steeringMax + 10, 1024); break;
        case 5: settings.joystickDir[0] = 1 - settings.joystickDir[0]; break;
        case 6: settings.joystickDir[1] = 1 - settings.joystickDir[1]; break;
        case 7: settings.joystickDir[2] = 1 - settings.joystickDir[2]; break;
        case 8: settings.joystickDir[3] = 1 - settings.joystickDir[3]; break;
      }
      lastDisplayTime = 0; // 立即刷新
    }
    p2Prev = !p2Down;

    // P3按下边沿：-操作
    static bool p3Prev = true;
    bool p3Down = (pcfButtonState[3] == LOW);
    if (p3Down && p3Prev) {
      switch (currentSettingIndex) {
        case 0: settings.channel = max(settings.channel - 1, 1); break;
        case 1: settings.sampleMax = max(settings.sampleMax - 10, 100); break;
        case 2: settings.batteryType = max(settings.batteryType - 1, 1); break;
        case 3: settings.throttleMax = max(settings.throttleMax - 10, 100); break;
        case 4: settings.steeringMax = max(settings.steeringMax - 10, 100); break;
        case 5: settings.joystickDir[0] = 1 - settings.joystickDir[0]; break;
        case 6: settings.joystickDir[1] = 1 - settings.joystickDir[1]; break;
        case 7: settings.joystickDir[2] = 1 - settings.joystickDir[2]; break;
        case 8: settings.joystickDir[3] = 1 - settings.joystickDir[3]; break;
      }
      lastDisplayTime = 0; // 立即刷新
    }
    p3Prev = !p3Down;
  }
}

// 摇杆校准状态变量（优化内存使用）
struct JoystickCalibration {
  int16_t centerValue[4];   // 中位值（使用int16_t节省内存）
  bool calibrated;          // 是否已校准
};

JoystickCalibration calibration = {0};

// 摇杆自动校准函数（简化版，减少内存使用）
void calibrateJoysticks() {
  Serial.println("Calibrating... Keep joysticks at center!");
  
  int samples = 0;
  long sum[4] = {0, 0, 0, 0};
  unsigned long startTime = millis();
  
  // 采集100个样本
  while (millis() - startTime < 1000) {
    sum[0] += analogRead(LEFT_JOY_Y);
    sum[1] += analogRead(LEFT_JOY_X);
    sum[2] += analogRead(RIGHT_JOY_X);
    sum[3] += analogRead(RIGHT_JOY_Y);
    samples++;
    delay(10);
    wdt_reset();
  }
  
  if (samples > 0) {
    calibration.centerValue[0] = sum[0] / samples;
    calibration.centerValue[1] = sum[1] / samples;
    calibration.centerValue[2] = sum[2] / samples;
    calibration.centerValue[3] = sum[3] / samples;
    calibration.calibrated = true;
    
    Serial.println("Calibration OK!");
    Serial.print("Center: ");
    Serial.print(calibration.centerValue[0]); Serial.print(",");
    Serial.print(calibration.centerValue[1]); Serial.print(",");
    Serial.print(calibration.centerValue[2]); Serial.print(",");
    Serial.println(calibration.centerValue[3]);
  }
}

// 读取摇杆值
void readJoysticks() {
  // 读取原始值
  int rawY = analogRead(LEFT_JOY_Y);
  int rawX = analogRead(LEFT_JOY_X);
  int rawRX = analogRead(RIGHT_JOY_X);
  int rawRY = analogRead(RIGHT_JOY_Y);

  // 防止溢出：限制最大值
  rawY = constrain(rawY, 0, settings.sampleMax);
  rawX = constrain(rawX, 0, settings.sampleMax);
  rawRX = constrain(rawRX, 0, settings.sampleMax);
  rawRY = constrain(rawRY, 0, settings.sampleMax);
  
  // 如果已校准，使用校准值
  if (calibration.calibrated) {
    // 映射到相对于校准中位的值
    int offsetY = rawY - calibration.centerValue[0];
    int offsetX = rawX - calibration.centerValue[1];
    int offsetRX = rawRX - calibration.centerValue[2];
    int offsetRY = rawRY - calibration.centerValue[3];
    
    // 映射到 -127 ~ +127
    int throttleHalf = settings.throttleMax / 2;
    int steeringHalf = settings.steeringMax / 2;
    
    data.joystick[0] = map(offsetY, -400, 400, -throttleHalf, throttleHalf);
    data.joystick[1] = map(offsetX, -400, 400, -steeringHalf, steeringHalf);
    data.joystick[2] = map(offsetRX, -400, 400, -throttleHalf, throttleHalf);
    data.joystick[3] = map(offsetRY, -400, 400, -throttleHalf, throttleHalf);
  } else {
    // 使用原始映射（假设中位是512）
    int midPoint = settings.sampleMax / 2;
    int throttleHalf = settings.throttleMax / 2;
    int steeringHalf = settings.steeringMax / 2;
    
    int leftY = map(rawY, 0, settings.sampleMax, -throttleHalf, throttleHalf);
    int leftX = map(rawX, 0, settings.sampleMax, -steeringHalf, steeringHalf);
    int rightX = map(rawRX, 0, settings.sampleMax, -throttleHalf, throttleHalf);
    int rightY = map(rawRY, 0, settings.sampleMax, -throttleHalf, throttleHalf);
    
    data.joystick[0] = leftY;
    data.joystick[1] = leftX;
    data.joystick[2] = rightX;
    data.joystick[3] = rightY;
  }
  
  // 应用摇杆正反设置
  if (settings.joystickDir[0] == 1) data.joystick[0] = -data.joystick[0];
  if (settings.joystickDir[1] == 1) data.joystick[1] = -data.joystick[1];
  if (settings.joystickDir[2] == 1) data.joystick[2] = -data.joystick[2];
  if (settings.joystickDir[3] == 1) data.joystick[3] = -data.joystick[3];
  
  // 再次限制范围，确保不超过限制
  int throttleHalf = settings.throttleMax / 2;
  int steeringHalf = settings.steeringMax / 2;
  data.joystick[0] = constrain(data.joystick[0], -throttleHalf, throttleHalf);
  data.joystick[1] = constrain(data.joystick[1], -steeringHalf, steeringHalf);
  data.joystick[2] = constrain(data.joystick[2], -throttleHalf, throttleHalf);
  data.joystick[3] = constrain(data.joystick[3], -throttleHalf, throttleHalf);
}

// 发送数据
void sendData() {
  // 发送数据（无ACK，<1ms完成）
  Mirf.send((byte *)&data);
  // ★ 加超时防止卡死
  unsigned long ts = millis();
  while(Mirf.isSending()) {
    if (millis() - ts > 50) break;
  }

  // ★ 立即轮询控制板回传（15ms超时）
  // 控制板收到后会在同一loop周期内回传，延迟<10ms
  unsigned long start = millis();
  while (millis() - start < 15) {
    if (Mirf.dataReady()) {
      byte rxBuf[21];
      Mirf.getData(rxBuf);
      memcpy(&response, rxBuf, sizeof(RemoteResponse));
      responseValid = true;
      break;
    }
  }
}

// 接收数据（始终读取，防止RX FIFO溢出）
// 注意：不在screen 1时也要读取，否则3个未读包后NRF停止接收
void receiveData() {
  while (Mirf.dataReady()) {  // 用while清空所有积压数据
    byte rxBuf[21];
    Mirf.getData(rxBuf);
    if (currentScreen == 1 && !inSettingMode) {
      // 只在Remote屏幕时更新数据（取最新一条）
      memcpy(&response, rxBuf, sizeof(RemoteResponse));
      responseValid = true;
    }
    // 其他屏幕：丢弃数据但清空FIFO
  }
}

// 上次显示的屏幕ID，用于检测屏幕切换
static int prevScreen = -1;

// 显示屏幕
void displayScreen() {
  if (!oledAvailable) return;
  // 帧率限制：避免OLED闪烁
  if (millis() - lastDisplayTime < displayInterval) return;
  lastDisplayTime = millis();

  // 检测屏幕是否切换，切换时需要全屏重绘
  bool screenChanged = (prevScreen != currentScreen || inSettingMode);
  int drawScreen = inSettingMode ? -1 : currentScreen;
  if (prevScreen != drawScreen) screenChanged = true;
  prevScreen = drawScreen;

  display.setTextSize(1);
  
  // 显示前验证数据（防止异常值）
  for (int i = 0; i < 4; i++) {
    if (data.joystick[i] > 255 || data.joystick[i] < -255) {
      data.joystick[i] = constrain(data.joystick[i], -255, 255);
    }
  }
  
  if (inSettingMode) {
    // 设置模式：全屏重绘（切换不频繁）
    display.clearDisplay();
    display.setCursor(30, 0);
    display.print("["); display.print(currentSettingIndex + 1); display.print("/9]");
    display.println(" Setting");
    
    display.setCursor(0, 10);
    switch (currentSettingIndex) {
      case 0:
        display.print(">Ch:"); display.print(settings.channel);
        display.print(" (1-125)"); break;
      case 1:
        display.print(">Sm:"); display.print(settings.sampleMax);
        display.print(" (100-1024)"); break;
      case 2:
        display.print(">Bt:"); display.print(settings.batteryType);
        display.print("s (1/2/3)"); break;
      case 3:
        display.print(">Tm:"); display.print(settings.throttleMax);
        display.print(" (100-1024)"); break;
      case 4:
        display.print(">St:"); display.print(settings.steeringMax);
        display.print(" (100-1024)"); break;
      case 5:
        display.print(">LY:"); display.print(settings.joystickDir[0] ? "Rev" : "Nor"); break;
      case 6:
        display.print(">LX:"); display.print(settings.joystickDir[1] ? "Rev" : "Nor"); break;
      case 7:
        display.print(">RX:"); display.print(settings.joystickDir[2] ? "Rev" : "Nor"); break;
      case 8:
        display.print(">RY:"); display.print(settings.joystickDir[3] ? "Rev" : "Nor"); break;
    }
    
    display.setCursor(0, 24);
    display.print("P0:Next P2:+ P3:-");
    display.display();
  } else if (currentScreen == 0) {
    // Local屏幕
    if (screenChanged) {
      display.clearDisplay();
      display.setCursor(45, 0);
      display.print("Local");
    }
    // 逐行擦除后重绘数据
    display.fillRect(0, 8, 128, 8, BLACK);
    display.setCursor(0, 8);
    display.print("Bat:");
    display.print(localBattery, 2);
    display.print("V Bt:");
    for(int i=0; i<4; i++) {
      display.print(data.buttons[i]);
    }
    
    display.fillRect(0, 16, 128, 8, BLACK);
    display.setCursor(0, 16);
    display.print("LY:");
    display.print(data.joystick[0]);
    display.print(" LX:");
    display.print(data.joystick[1]);
    
    display.fillRect(0, 24, 128, 8, BLACK);
    display.setCursor(0, 24);
    display.print("RY:");
    display.print(data.joystick[3]);
    display.print(" RX:");
    display.print(data.joystick[2]);
    display.display();
  } else if (currentScreen == 1) {
    // Remote屏幕
    if (screenChanged) {
      display.clearDisplay();
      display.setCursor(45, 0);
      display.print("Remote");
    }
    if (responseValid) {
      display.fillRect(0, 8, 128, 8, BLACK);
      display.setCursor(0, 8);
      display.print("Bat:");
      display.print(response.batteryVoltage / 100.0, 1);
      display.print("V Cur:");
      display.print(response.current / 100.0, 1);
      display.print("A");
      
      display.fillRect(0, 16, 128, 8, BLACK);
      display.setCursor(0, 16);
      display.print("LM:");
      int8_t leftMotorVal = (int8_t)response.leftMotorPWM;
      int8_t rightMotorVal = (int8_t)response.rightMotorPWM;
      display.print(leftMotorVal);
      display.print(" RM:");
      display.print(rightMotorVal);
      
      display.fillRect(0, 24, 128, 8, BLACK);
      display.setCursor(0, 24);
      display.print("Acc:");
      display.print(response.accelX / 1000.0, 1);
      display.print(",");
      display.print(response.accelY / 1000.0, 1);
    } else {
      display.fillRect(0, 8, 128, 24, BLACK);
      display.setCursor(20, 16);
      display.print("Waiting data...");
    }
    display.display();
  } else {
    // GPS屏幕
    if (screenChanged) {
      display.clearDisplay();
      display.setCursor(45, 0);
      display.print("GPS");
      display.setCursor(0, 8);
      display.print("GPS: No data");
      display.setCursor(0, 16);
      display.print("Ext Output");
    }
    display.fillRect(0, 24, 128, 8, BLACK);
    display.setCursor(0, 24);
    display.print("Ch:");
    display.print(settings.channel);
    display.display();
  }
}
