
/*
制作原理：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用两个舵机来控制炮塔（摄像头）的方位和俯仰；
3、另外，有4个开关（最多可以有8个），用于控制灯，音乐等设备； 
4、GT24，低8位传输油门信号，8~15位传输方向信号，16~24位，传输炮塔的方位和俯仰信号， 主帧传输方位信号，副帧传输俯仰信号，最高7位传输开关信号，最高位为帧标记位，0为主帧，1为副帧

接线方法：适用无人机遥控改装
  左x电位器中点接模拟IO A0；A3
  左Y电位器中点接模拟IO A1；A1
  右X电位器中点接模块IO A2；A2
  右Y电位器中点接模块IO A3；A0
  显示屏SDA   A4；
  显示屏SCL   A5；
  镜头控制左：A6；
  镜头控制右：A7

  
  NR24L01接线：
  CSN  <---> D7
  CE   <---> D8
  MOSI <---> D11
  MISO <---> D12
  SCK  <---> D13
  VCC  <---> 3.3V

  数据IO: 原先的拨动开关变成了轻触式开关。
  D2: 左前 改为中断输入
  D3：右前
  D4、左前
  D5：
  中左按键 改为pcf8574 P6
  中右按键 改为pcf8574 P7
  分别的4个开关 
  D6接LED指示灯，开机指示
*/

// 遥控器部分代码
#include <Arduino.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PCF8574.h>

#define ARDUINO_IRQ 2 
#define OLED_RESET 4


#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
// static const unsigned char PROGMEM logo16_glcd_bmp[] =
// { B00000000, B11000000,
//   B00000001, B11000000,
//   B00000001, B11000000,
//   B00000011, B11100000,
//   B11110011, B11100000,
//   B11111110, B11111000,
//   B01111110, B11111111,
//   B00110011, B10011111,
//   B00011111, B11111100,
//   B00001101, B01110000,
//   B00011011, B10100000,
//   B00111111, B11100000,
//   B00111111, B11110000,
//   B01111100, B11110000,
//   B01110000, B01110000,
//   B00000000, B00110000 };
#define __DEBUG__

#ifdef __DEBUG__
#define DEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGL(...) Serial.println(__VA_ARGS__)
//#define DEBUG(...) Serial.println(__VA_ARGS__); \
                   Serial.print(" @ [SRC]:      "); \
                   Serial.println(__FILE__); \
                   Serial.print(" @ [LINE]:     "); \
                   Serial.println(__LINE__); \
                   Serial.print(" @ [FUNCTION]: "); \
                   Serial.println(__func__); 
#else
#define DEBUG(...)
#define DEBUGL(...)
#endif

uint8_t convert[5]={0};
uint8_t fw_valueX,fy_valueY,last_fw_valueX,last_fy_valueY;
uint16_t xSourceValue,ySourceValue,zSourceValue,con2ValueX,con2ValueY=0;
uint8_t recBatVol,recCurrent,recSensorState1,recSensorState2,x_value,y_value,z_value,last_zvalue,last_xvalue,last_yvalue,con_value,last_convalue;
uint32_t trans_value, temp_value= 0;
uint32_t recValue = 0;
union transData
{
   long newvalue;
   byte buffer[4];
};

transData sendUnion;
transData recUnion;
//Serial.println(thisUnion.newvalue, HEX);

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_PCF8574 pcf;
bool swState[8] ={0};
bool ISmaster = true;  //主副帧
int inputLimit = 670; //摇杆的上限，根据遥控器的电路来定，如果摇杆是用3.3V分压，大约是710，用5V分压是1024；
int xpin = A3;  // 方向输入脚模拟A0
int ypin = A1;  // 油门输入脚模拟A1
int con2X = A0; // 第二摇杆X方向输入脚模拟A2
int con2Y = A2; // 第二摇杆Y方向输入脚模拟A3
// int volDect = A4; // 电压检测；
// int lowPower = 737; //电压告警阈值,3.6V
// int zpin = A5;  // 
// 3S电池告警电压3.6V，电池总电压为10.8V，检测处电压3.45V（4700/（10000上拉电阻+4700下拉电阻）* 10.8），对应读取值为707
//开关引脚定义
int swPin[] = {2,3,4,5};
//LED指示灯
int exLed = 6;
long lastLed = 0;
int ledState = 0;
//当前读取的开关状态
int pinState[5] ={0};
//开关状态
int swState[5] = {0};
//上一次开关的状态
int lastSwState[5] = {0};
//去抖动时间
long interval=100;
//上次开关变化的时间
long lastTM[5] = {0};
void setup()
{
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"SENDE"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = 4;     //设置传送位数，每通道使用8位；低8位是油门、次8位是方向，再高位是功能开关。
  Mirf.channel = 90;              //设置所用信道
  Mirf.config();
  Serial.begin(115200);
     // Read and print RF_SETUP 用于检测是否初始化正常；
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  for(int i=0;i<4;i++){
    pinMode(swPin[i], INPUT_PULLUP);
  }
  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);
  pinMode(con2X, INPUT);
  pinMode(con2Y, INPUT);
  // pinMode(zpin,INPUT);
  pinMode(exLed, OUTPUT);
  digitalWrite(exLed, LOW); //指示灯默认开
//显示屏
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
    // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);

  // // Clear the buffer.
  // display.clearDisplay();
  // // text display tests
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0,0);
  // display.println("Hello, world!");
  // display.setTextColor(BLACK, WHITE); // 'inverted' text
  // display.println(3.141592);
  // display.setTextSize(2);
  // display.setTextColor(WHITE);
  // display.print("0x"); display.println(0xDEADBEEF, HEX);
  // display.display();
  // delay(2000);
  // display.clearDisplay();
}

void loop()
{
  Mirf.setTADDR((byte *)"RECVE");           //设置接收端地址
  //读取摇杆的数值
  //电池电压检查，低于阈值就闪烁1秒为周期
  /*if(analogRead(volDect) <= lowPower){
    //指示灯闪烁
    if(millis()-lastLed >= 1000){
      digitalWrite(exLed, !ledState);
      lastLed = millis();
    }
  }
  else{
    digitalWrite(exLed, HIGH);
  }*/
  xSourceValue = analogRead(xpin);  //读取摇杆输入的数值
  // 以下1024是根据摇杆来的，如果使用其它的遥控器，要根据实际情况，确定上限值
  x_value = map(xSourceValue,0,inputLimit,0,255); //将模拟量的10位值转为8位值；
  ySourceValue = analogRead(ypin);
  y_value = map(ySourceValue,0,inputLimit,0,255);
  //zSourceValue = analogRead(zpin);
  z_value = map(zSourceValue,0,inputLimit,0,255);
  //value = random(255); 
  //读取第二摇杆X值
  con2ValueX = analogRead(con2X);
  fw_valueX = map(con2ValueX,0,inputLimit,0,255);
  //读取第二摇杆Y值
  con2ValueY = analogRead(con2Y);
  fy_valueY = map(con2ValueY,0,inputLimit,0,255);
  //camValue = (fy_valueY << 4) + fw_valueX;
  
  //开关状态检测，带去抖动功能
 /* for(int i=0;i<sizeof(swPin)/sizeof(swPin[0]);i++)
  {
    //读取开关状态，如果状态发生变化，记录时间和上一次开关状态；
    pinState[i] = digitalRead(swPin[i]);
    if( lastSwState[i] != pinState[i]) {
      lastTM[i] = millis();
      lastSwState[i] = pinState[i];
    }
    // 如果当前时间到上一次开关状态变化的时间差大于防抖动的时间，再判断当前的状态和上次的状态是否一致，如不一致，说明开关状态发生了变化，更新开关状态
    if(millis() - lastTM[i] >= interval) {
      if(pinState[i] == lastSwState[i]) {
        // swState[i] = pinState[i];拨动式开关使用当前开关的状态
        swState[i] = !swState[i]; // 轻触式开关进行翻转即可
        bitWrite(con_value,i,swState[i]);
      }
    }  */
// 轻触式开关的检测更加简单，只需要判断是否低电平即可
  for(int i=0;i<sizeof(swPin)/sizeof(swPin[0]);i++)
    {
      //读取开关状态，如果状态为低电平，记录时间和按下标记；
      pinState[i] = digitalRead(swPin[i]);
      if( !pinState[i])  {
        lastTM[i] = millis();
        lastSwState[i] = 1; //开关按下标记
      }
      // 如果开关按下标记正常，且当前时间到上一次开关按下的间差大于防抖动的时间，更新开关状态
      if(lastSwState[i]  && (millis() - lastTM[i] >= interval)) {
        swState[i] = !swState[i]; // 轻触式开关进行翻转即可
        bitWrite(con_value,i,swState[i]);
        lastSwState[i] = 0; //状态翻转后，将开关按下标记复位
      }
    }  
  /* 
  // 最低4位为开关状态，最低位为开关1的状态，由低到高分别为6、7、8、9脚的状态，高4位暂时未用
  Serial.print("swState:");
  for(int j=0; j<4; j++){
    Serial.print(swState[j]);
    Serial.print(",");
    //bitWrite(con_value,j,swState[j]);
  }
  Serial.println();*/
 
  //数据有变化才发送，防止电位器抖动导致的值变化
  if(abs(z_value - last_zvalue) > 5 |abs(x_value - last_xvalue) > 5 | abs(y_value - last_yvalue) > 5 | abs(fw_valueX - last_fw_valueX) > 5 | abs(fy_valueY - last_fy_valueY) > 5 | con_value != last_convalue)
  //| con_value != last_convalue
  {
    //发送的数据组装
    //主帧数据处理
    if(ISmaster){
      bitWrite(con_value,7,0); //写入帧标记位
      sendUnion.buffer[2] = fw_valueX;
    }
    //副帧
    else{
      bitWrite(con_value,7,1); //写入帧标记位
      sendUnion.buffer[2] = fy_valueY;
    }
    sendUnion.buffer[0] = y_value;
    sendUnion.buffer[1] = x_value;
    sendUnion.buffer[3] = con_value;
    ISmaster = !ISmaster;
    //发送数据并打印
    Mirf.send((byte *)&sendUnion.newvalue);     //发送指令，组合后的数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环 
      last_xvalue = x_value;   //last数据更新
      last_yvalue = y_value;
      last_fw_valueX = fw_valueX;
      last_fy_valueY = fy_valueY;
      last_zvalue = z_value;
      last_convalue = con_value;
      
      DEBUG("xSourceValue: ");
      DEBUGL(xSourceValue);
      DEBUG("ySourceValue: ");
      DEBUGL(ySourceValue);
      DEBUG("con2ValueX:");
      DEBUGL(con2ValueX);
      DEBUG("con2ValueY:");
      DEBUGL(con2ValueY);      
      DEBUG("x_value: ");
      DEBUGL(x_value);
      DEBUG("y_value: ");
      DEBUGL(y_value);
      // DEBUG("z_value: ");
      // DEBUGL(z_value);
      // DEBUG("fw_valueX: ");
      // DEBUGL(fw_valueX);
      // DEBUG("fy_valueY: ");
      // DEBUGL(fy_valueY);
      // DEBUG("con_value: ");
      // DEBUGL(con_value,BIN);
      // DEBUG("trans_value: ");
      DEBUGL(sendUnion.newvalue,BIN);
  }
  delay(50);
  // 查看接收数据
  if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &recUnion.newvalue);
    DEBUG("Recive Data: "); 
    DEBUGL(recUnion.newvalue,BIN); 
    // 接收数据的定义
    // 最低8位为电池电压，次8位为电流，第3、4个低8位为状态传感器
    recBatVol = recUnion.buffer[0];
    //fx = thisUnion.buffer[1];
    recCurrent = recUnion.buffer[1];
    recSensorState1 = recUnion.buffer[2];
    recSensorState2 = recUnion.buffer[3];
    DEBUG("recCurrent = ");
    DEBUG(recUnion.buffer[1]);
    DEBUG(",");
    DEBUG("recBatVol = ");
    DEBUGL(recUnion.buffer[0]);
    display.clearDisplay();
    // text display tests
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    // 电压计算公式：接收的值/255*5V得到采样值，再根据分压比（由接收机采样电路决定，上91K，下33K为0.266129），得到实际的电压
    // display.print("batterry: "); display.print(float(recBatVol)/255*5/0.266129); display.println("v");
    // display.print("recCurrent: "); display.print(float(recCurrent)); display.println("A");
    // display.print("leftspeed: "); display.println(recSensorState1); 
    // display.print("rightspeed: "); display.println(recSensorState2); 
    display.print("esc1: "); display.println(recUnion.buffer[0]);
    display.print("esc2: "); display.println(recUnion.buffer[1]);
    display.print("esc3: "); display.println(recUnion.buffer[2]);
    display.print("esc4: "); display.println(recUnion.buffer[3]);
    display.display();

 }
 delay(50); //这个延迟要放到这里，否则程序错乱，一直会有垃圾数据输出}
}