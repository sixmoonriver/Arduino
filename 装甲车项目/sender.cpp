
/*
制作原理：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用两个舵机来控制炮塔（摄像头）的方位和俯仰；
3、另外，有4个开关（最多可以有8个），用于控制灯，音乐等设备； 
4、GT24，低8位传输油门信号，8~15位传输方向信号，16~24位，传输炮塔的方位和俯仰信号， 主帧传输方位信号，副帧传输俯仰信号，最高7位传输开关信号，最高位为帧标记位，0为主帧，1为副帧

接线方法：适用无人机遥控改装
  左x电位器中点接模拟IO A1；
  左Y电位器中点接模拟IO A0；
  右X电位器中点接模块IO A2；
  右Y电位器中点接模块IO A3；
  电压检测脚接   A4；
  key2旋钮输入   A5；
  指示灯接D6； 

  
  NR24L01接线：
  CSN  <---> D7
  CE   <---> D8
  MOSI <---> D11
  MISO <---> D12
  SCK  <---> D13
  VCC  <---> 3.3V

  数据IO: 原先的拨动开关变成了轻触式开关。
  D2: 一键降落
  D3：左上
  D4、右上
  D5：右下
  D9: 开关
  分别的4个开关 
  数据IO D6接LED指示灯
*/

// 遥控器部分代码
#include <Arduino.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

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
uint8_t cam_valueX,cam_valueY,last_cam_valueX,last_cam_valueY,camValue= 0;
uint16_t xSourceValue,ySourceValue,zSourceValue,camSourceValueX,camSourceValueY=0;
uint8_t x_value,y_value,z_value,last_zvalue,last_xvalue,last_yvalue,con_value,last_convalue,last_camValue=0;
uint32_t trans_value, temp_value= 0;
union transData
{
   long newvalue;
   byte buffer[4];
};

transData thisUnion;
//Serial.println(thisUnion.newvalue, HEX);

bool ISmaster = true;  //主副帧
int xpin = A0;  // 方向输入脚模拟A0
int ypin = A1;  // 油门输入脚模拟A1
int camControlX = A2; // 方向输入脚模拟A2
int camControlY = A3; // 方向输入脚模拟A3
int volDect = A4; // 电压检测；
int lowPower = 737; //电压告警阈值,3.6V
int zpin = A5;  // 
// 3S电池告警电压3.6V，电池总电压为10.8V，检测处电压3.45V（4700/（10000上拉电阻+4700下拉电阻）* 10.8），对应读取值为707
//开关引脚定义
int swPin[] = {2,3,4,5,9};
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
  Mirf.setRADDR((byte *)"SENDE2"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = 4;     //设置传送位数，每通道使用8位；低8位是油门、次8位是方向，再高位是功能开关。
  Mirf.channel = 80;              //设置所用信道
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
  pinMode(camControlX, INPUT);
  pinMode(camControlY, INPUT);
  pinMode(zpin,INPUT);
  pinMode(exLed, OUTPUT);
  digitalWrite(exLed, LOW); //指示灯默认开
}
void loop()
{
  Mirf.setTADDR((byte *)"RECVE2");           //设置接收端地址
  //读取摇杆的数值
  //电池电压检查，低于阈值就闪烁1秒为周期
  if(analogRead(volDect) <= lowPower){
    //指示灯闪烁
    if(millis()-lastLed >= 1000){
      digitalWrite(exLed, !ledState);
      lastLed = millis();
    }
  }
  else{
    digitalWrite(exLed, HIGH);
  }
  xSourceValue = analogRead(xpin);  //读取摇杆输入的数值
  x_value = map(xSourceValue,0,710,0,255); //将模拟量的10位值转为8位值；
  ySourceValue = analogRead(ypin);
  y_value = map(ySourceValue,0,710,0,255);
  zSourceValue = analogRead(zpin);
  z_value = map(zSourceValue,0,710,0,255);
  //value = random(255); 
  //读取摄像头方位角度控制值
  camSourceValueX = analogRead(camControlX);
  cam_valueX = map(camSourceValueX,0,710,0,255);
  //读取摄像头俯仰角度控制值
  camSourceValueY = analogRead(camControlY);
  cam_valueY = map(camSourceValueY,0,710,0,255);
  //camValue = (cam_valueY << 4) + cam_valueX;
  
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
  if(abs(z_value - last_zvalue) > 5 |abs(x_value - last_xvalue) > 5 | abs(y_value - last_yvalue) > 5 | abs(cam_valueX - last_cam_valueX) > 5 | abs(cam_valueY - last_cam_valueY) > 5 | con_value != last_convalue)
  //| con_value != last_convalue
  {
    //发送的数据组装
    //trans_value = 0;
    //主帧数据处理
    if(ISmaster){
      bitWrite(con_value,7,0); //写入帧标记位
      thisUnion.buffer[2] = cam_valueX;
      // thisUnion.buffer[2] = z_value;
      // thisUnion.buffer[3] = con_value;
    }
    //副帧
    else{
      bitWrite(con_value,7,1); //写入帧标记位
      // slaveUnion.buffer[0] = cam_valueX;
      // slaveUnion.buffer[1] = cam_valueY;
      thisUnion.buffer[2] = cam_valueY;
      // slaveUnion.buffer[3] = con_value;
    }
    thisUnion.buffer[0] = y_value;
    thisUnion.buffer[1] = x_value;
    thisUnion.buffer[3] = con_value;
    ISmaster = !ISmaster;
    // thisUnion.buffer[0] = y_value;
    // thisUnion.buffer[1] = x_value;
    // thisUnion.buffer[2] = camValue;
    // thisUnion.buffer[3] = con_value; 
    //发送数据并打印
    Mirf.send((byte *)&thisUnion.newvalue);     //发送指令，组合后的数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环 
      last_xvalue = x_value;   //last数据更新
      last_yvalue = y_value;
      last_cam_valueX = cam_valueX;
      last_cam_valueY = cam_valueY;
      last_zvalue = z_value;
      last_convalue = con_value;
      
      // DEBUG("xSourceValue: ");
      // DEBUGL(xSourceValue);
      // DEBUG("ySourceValue: ");
      // DEBUGL(ySourceValue);
      DEBUG("x_value: ");
      DEBUGL(x_value);
      DEBUG("y_value: ");
      DEBUGL(y_value);
      DEBUG("z_value: ");
      DEBUGL(z_value);
      DEBUG("cam_valueX: ");
      DEBUGL(cam_valueX);
      DEBUG("cam_valueY: ");
      DEBUGL(cam_valueY);
      DEBUG("con_value: ");
      DEBUGL(con_value,BIN);
      DEBUG("trans_value: ");
      DEBUGL(thisUnion.newvalue,BIN);
  }
  delay(100);
}