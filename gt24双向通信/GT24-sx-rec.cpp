/*
适用于四轮小车项目
制作原理：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用两个舵机来控制炮塔（摄像头）的方位和俯仰；
3、另外，有4个开关（最多可以有8个），用于控制灯，音乐等设备； 
4、GT24，低8位传输油门信号，8~15位传输方向信号，16~24位，各4位传输炮塔的方位和俯仰信号。最高8位传输开关信号

接线方法：
  D3、D9接左侧驱动的控制；
  A6、A7接右侧驱动的控制；
  D5、D6分别为左右侧PWM控制；
  D10接方位舵机；
  A0接俯仰舵机；
  A4 SDA接pcf8574
  A5 SCL接pcf8574

  NR24L01接线：
  CSN  <---> D7
  CE   <---> D8
  MOSI <---> D11
  MISO <---> D12
  SCK  <---> D13
  VCC  <---> 3.3V

  数据IO D2、D3、D4、D5分别的4个开关 
  枪控油门范围： 中点：130，上限185，下限80；
  枪控方向范围： 中点：125，上限205，下限45；
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h>
#include <Adafruit_PCF8574.h>
//#include <C:\Users\Administrator\Documents\Arduino\libraries\Adafruit-Motor-Shield-library\AFMotor.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// Ws2812 引脚和数量
#define PIN 4
#define NUM_LEDS 29 // LED数量
//#define __DEBUG__

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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

//SoftwareSerial S2(A0, A3); //A0为接收，A3为发送；

Adafruit_PCF8574 pcf;
Servo fwservo; // 建立方位舵机对象
Servo fyservo; // 创建俯仰舵机对象
//SoftwareSerial S2(A2, A3); //A2为接收，A3为发送；

//定义驱动板控制引脚
int left1 = 3;
int left2 = 9;
int right1 = 2;
int right2 = 10;
int leftPwm = 5;
int rightPwm = 6;
//速度
int leftSpeed = 128;
int rightSpeed = 128;
int forwardSpeed = 128;
int lowSpeed = 50;  //轮子最低速度，低于此值电机无法转动 
//轮子的转弯系数 数值越大，两轮的转速差距越大 2.1
float turnIndex = 2.1;
//方向的停止范围
int turnUp = 130;
int turnDown = 120;
//方向的上、下限
int fxUp = 205;
int fxDown = 45;  
int fxError = 5;
//油门的停止范围
int forwardUP = 135; //油门停止区间上限
int forwardDown = 128; //油门停止区间下限
//油门上、下限
int ymUp = 185;
int ymDown = 82;
//油门限速用于调试，防止电流过大
int ymlimit = 255;
//运行状态
bool isForward = 1;
//停机相关设置
long lastStopTime = 0;
long stopInterval = 1000; //1秒
long servoResetTime = 0;
uint8_t ym,fx,lastfx,con_value,ptValue,lastPtValue = 0;
uint32_t recValue = 0;
int fwval,fyval,camval,lastFyval,lastFwval = 0;
//舵机的误差，防止舵机抖动
int servoErr = 1;
//电池电压检测：A1脚
int lowBattery = 604; 
bool batteryState = 1;
long lastLowtime = 0;
// 数据上报间隔
long lastreport = 0; //记录上报时间
long reportTime = 2000; //2秒上报一次
//接收数据联合体
union transData
{ 
   long newvalue;
   byte buffer[4];
};

transData recUnion,sendUnion;

void forward(int speedL=128, int speedR=128);
void backward(int speedL=128, int speedR=128);
void speedControl(int speedL, int speedR);
void stop();
void yuanDiReturn(int speedL=90, int speedR=90);

void setup()
{
  Serial.begin(115200);
  //S2.begin(9600);
  // 初始化pcf8574对象，默认地址为0x20，如果地址改了，要改！！
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    pcf.pinMode(p, OUTPUT);
  }
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = 4;  // 设置传送位数，16位是2，32位是4；
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();

   // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  //ws2812初始化
  strip.begin();
  strip.setBrightness(50); // 初始亮度设为50%，避免过流
  
  //colorWipe(strip.Color(0, 0, 255), 50, strip.numPixels()); 
  //初始化为绿色
  for(int i=0; i<NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0));
  }
  strip.show();
  delay(500);
  
  //舵机初始化
  /*
  servo.attach(pin) 
  servo.attach(pin, min, max)
  min(可选)：脉冲宽度，以微秒为单位，对应于舵机上的最小(0度)角度(默认为544)
  max(可选)：脉冲宽度，以微秒为单位，对应于舵机上的最大(180度)角度(默认为2400)
  */
  fwservo.attach(A3,600,2000);
  fwservo.write(90);//回到中间位置
  fyservo.attach(A0,600,2000);
  fyservo.write(90);//回到中间位置
  //控制引脚设置
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(leftPwm, OUTPUT);
  pinMode(rightPwm, OUTPUT);
  stop();
  delay(200);
}
 
void loop()
{
  if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
  Mirf.getData((byte *) &recUnion.newvalue);
  //fx = (recValue & 0xff00) >> 8;
  fx = recUnion.buffer[1];
  //ym = recValue & 0xff;
  ym = recUnion.buffer[0];
  ptValue = recUnion.buffer[2];
  //ptValue = (recValue >> 16) & 0xff;
  if(ptValue != lastPtValue){
    //S2.write(ptValue);
    lastPtValue = ptValue;
  }
  //con_value = (recValue >> 24) & 0xff;
  con_value = recUnion.buffer[3];
  //串口发送的数据依次为方向、油门、控制、炮台，按最高位到最低位是：控制、炮台、方向、油门
 /*
  S2.write(155);
  S2.write(66);
  //S2.write(recUnion.buffer, 4);
  S2.write(uint8_t(recUnion.buffer[0]));
  S2.write(uint8_t(recUnion.buffer[1]));
  S2.write(155);
  S2.write(44);
  S2.write(uint8_t(recUnion.buffer[2]));
  S2.write(uint8_t(recUnion.buffer[3]));
  DEBUG("fx = ");
  DEBUG(recUnion.buffer[1]);
  DEBUG(",");
  DEBUG("ym = ");
  DEBUGL(recUnion.buffer[0]);
  DEBUG("con_value: "); 
  DEBUGL(recUnion.buffer[3],BIN);  
  DEBUG("PaoTa: ");
  DEBUGL(recUnion.buffer[2],BIN);
 */
  DEBUG("fx = ");
  DEBUG(fx);
  DEBUG(",");
  DEBUG("ym = ");
  DEBUGL(ym);
  DEBUG("con_value: "); 
  DEBUGL(con_value,BIN);  
  DEBUG("PaoTa: ");
  DEBUGL(ptValue,BIN);
 }
 delay(50); //这个延迟要放到这里，否则程序错乱，一直会有垃圾数据输出
  //有刷电机油门控制，控制器的中间位置，且距上一次停止时间间隔超过1s，电机停止且计时复位
  if(ym >= forwardDown and ym <= forwardUP) {
    if(millis()-lastStopTime >= stopInterval){
      stop();
      lastStopTime = millis();
    }
  }
  // 前进控制
  else if(ym > forwardUP and ym < ymUp) { // 前进 枪控
    //leftSpeed = rightSpeed = map(ym, 0, forwardDown, 255, lowSpeed); //摇杆控制
    leftSpeed = rightSpeed = map(ym, forwardUP, ymUp, lowSpeed, ymlimit); //枪控
    //转弯控制
    //开关控制是否使用差速转弯，开关第三位控制,高电平，不使用差速转弯
    if(bitRead(con_value,2)){
      if(fx > turnUp) { //如果方向偏左
        //leftSpeed = map(fx, turnDown, 0, rightSpeed/turnIndex, lowSpeed); //越往左，速度越慢  摇杆控制 
        fwservo.write(map(fx,turnUp, fxUp,90,145));
        servoResetTime = millis();
      }
      else if(fx <= turnDown) { //否则右转
         //rightSpeed = map(fx, turnUp, 255, lowSpeed, leftSpeed/turnIndex); //越往右，速度越慢 摇杆控制
        fwservo.write(map(fx,turnDown, fxDown,90,40));
        servoResetTime = millis();
      }
    }
    //使用差速转弯
    else{ 
      if(fx > turnUp) { //如果方向偏左
        //leftSpeed = map(fx, turnDown, 0, rightSpeed/turnIndex, lowSpeed); //越往左，速度越慢  摇杆控制 
        leftSpeed = map(fx, turnUp, fxUp, rightSpeed/turnIndex, lowSpeed); //越往左，速度越慢  枪控
        fwservo.write(map(fx,turnUp, fxUp,90,145));
        servoResetTime = millis();
      }
      else if(fx <= turnDown) { //否则右转
        rightSpeed = map(fx, turnDown, fxDown, leftSpeed/turnIndex, lowSpeed); //越往右，速度越慢
        //rightSpeed = map(fx, turnUp, 255, lowSpeed, leftSpeed/turnIndex); //越往右，速度越慢 摇杆控制
        fwservo.write(map(fx,turnDown, fxDown,90,40));
        servoResetTime = millis();
      }
    }
    //DEBUG("leftSpeed: ");
    //DEBUG(leftSpeed);
    //DEBUG(" rightSpeed: ");
    //DEBUGL(rightSpeed);
    if(!isForward){ //如果是倒退的状态，先翻转状态。
      isForward = !isForward;
      stop();
      delay(100);
    }
    forward(leftSpeed, rightSpeed);
    DEBUG("forward...");
    DEBUG("leftSpeed: ");
    DEBUG(leftSpeed);
    DEBUG(" rightSpeed: ");
    DEBUGL(rightSpeed);
    //speedControl(leftSpeed, rightSpeed);
  }
  // 后退控制
  else if(ym < forwardDown and ym > ymDown) {
    //leftSpeed = rightSpeed = map(ym, forwardUP, 255, ); //摇杆控制
    leftSpeed = rightSpeed = map(ym, forwardDown, ymDown,  lowSpeed, ymlimit);  //枪控
    //根据开关第3位确定是否使用差速转弯，高电平不使用
    if(bitRead(con_value,2)){
      if(fx > turnUp) { //如果方向偏左
        //leftSpeed = map(fx,turnDown,0,rightSpeed/turnIndex,lowSpeed); //越往左，速度越慢 //摇杆控制
        fwservo.write(map(fx,turnUp,fxUp,90,40));
        servoResetTime = millis();
      }
      else if(fx <= turnDown){ //否则右转
        //rightSpeed = map(fx, turnUp, 255, leftSpeed/turnIndex, lowSpeed); //越往左，速度越慢  //摇杆控制
        fwservo.write(map(fx,turnDown, fxDown,90,145));
        servoResetTime = millis();
      }
    }
    else{
      if(fx > turnUp) { //如果方向偏左
        //leftSpeed = map(fx,turnDown,0,rightSpeed/turnIndex,lowSpeed); //越往左，速度越慢 //摇杆控制
        leftSpeed = map(fx,turnUp,fxUp,rightSpeed/turnIndex,lowSpeed); //越往左，速度越慢 //枪控
        fwservo.write(map(fx,turnUp,fxUp,90,40));
        servoResetTime = millis();
      }
      else if(fx <= turnDown){ //否则右转
        //rightSpeed = map(fx, turnUp, 255, leftSpeed/turnIndex, lowSpeed); //越往左，速度越慢  //摇杆控制
        rightSpeed = map(fx, turnDown, fxDown, leftSpeed/turnIndex, lowSpeed); //越往左，速度越慢 //枪控
        fwservo.write(map(fx,turnDown, fxDown,90,145));
        servoResetTime = millis();
      }
  }
//如果是前进状态，状态翻转，延迟后再换方向
    if(isForward){  
      isForward = !isForward;
      stop();
      delay(100);
      backward();
    }
    else{
      backward(leftSpeed, rightSpeed);
    }
    DEBUG("backward...");
    DEBUG("leftSpeed: ");
    DEBUG(leftSpeed);
    DEBUG(" rightSpeed: ");
    DEBUGL(rightSpeed);
    //speedControl(leftSpeed, rightSpeed);
  }
  //舵机定时回位，1000ms
  if((millis()-servoResetTime) >= 1000){
    fwservo.write(90);
  } 
  // 控制值处理
  //pcf.digitalReadByte();
  pcf.digitalWriteByte(con_value);
  //炮台控制 低4位为方位，高4位为俯仰
  //方位控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  //fwval = ptValue & 0x0f;
  
  /*if(fwval != lastFwval){
    //DEBUG("paota FW: ");
    //DEBUGL(fwval);
    fwval = map(fwval,0,15,160,10);  //160,10为舵机的可转动范围，根据需要调整；
    fwservo.write(fwval);
    lastFwval = fwval;
  }*/
    //方位控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  //fwval = ptValue & 0x0f;
  /*if(abs(fwval - lastFwval) >= servoErr){
    // DEBUG("paota FW: ");
    // DEBUGL(fwval);
    lastFwval = fwval;
    //160,10为舵机的可转动范围，根据需要调整；
    //fwservo.write(map(fwval,0,15,45,135));
  }*/

  //俯仰控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  fyval = (ptValue>>4) & 0x0f;
  //俯仰控制用于设置油门的PWM上限，防止电流过大，方便调试
  if(fyval != lastFyval){
    //DEBUG("paota FY: ");
    //DEBUGL(fyval);
    ymlimit = map(fyval,0,15,0,255);
    //fyval = map(fyval,0,15,90,135);
    //fyservo.write(fyval);
    lastFyval = fyval;
    //利用俯仰值调整灯的数量
    //int ledNumber = map(fyval,0,15,1,strip.numPixels());
    //colorWipe(strip.Color(255,0,0), 0, ledNumber);
  }
  //电池电压检查，高于阈值显示绿色，低于阈值显示红色
  if(analogRead(A1) < lowBattery){
    //持续时间大于1.5S,电池低电压
    if(millis()-lastLowtime >= 1500){
      for(int i=0; i<NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
      strip.show();
      delay(100);
      DEBUGL("Battery is change Low");        
    }
  }
  else{
    lastLowtime=millis();
  }

  //发送数据返回
  Mirf.setTADDR((byte *)"SENDE");           //设置接收端地址
  if(millis() - lastreport > reportTime) {
    lastreport = millis();
    int batVol = analogRead(A1);
    int current = analogRead(A0);
    sendUnion.buffer[0] = map(batVol,0,1024,0,255);
    sendUnion.buffer[1] = map(current,0,1024,0,255);
    sendUnion.buffer[2] = 0;
    sendUnion.buffer[3] = 0;
    //发送数据并打印
    Mirf.send((byte *)&sendUnion.newvalue);     //发送指令，组合后的数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环 
    delay(50);
  }

}

//电机速度调整
void speedControl(int speedL, int speedR){
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
//电机前进
void forward(int speedL=128, int speedR=128){
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
// 电机后退
void backward(int speedL, int speedR){
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
// 电机停止
void stop(){
  //pcf.digitalWriteByte(0);
  digitalWrite(left1, LOW);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, LOW); 
}
//原地掉头
void yuanDiReturn(int speedL, int speedR){
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);  
  analogWrite(leftPwm, speedL);
  analogWrite(rightPwm, speedR);
}
