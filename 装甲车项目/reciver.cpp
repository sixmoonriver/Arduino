/*
  船模控制2.0版本
  1、GT24模块进行通信，传输24位的数据，低8位是油门、次8位是方向，再高位是功能开关。
  2、接线方法：设置主电机（电调）的控制脚为6，方向舵机的控制引脚为5，摄像头舵机的控制引脚为3，第1位开关的控制输出为4；
  注意：用串口调试可能会产生干扰，导致调整一个舵机，另一个也会动。
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h>
#include <Adafruit_PCF8574.h>

//Adafruit_PCF8574 pcf;
Servo fwservo; // 建立方位舵机对象
Servo fyservo; // 创建俯仰舵机对象


//定义驱动板控制引脚
int left1 = 2;
int left2 = 3;
int right1 = 4;
int right2 = 5;

uint8_t ym,fx,con_value,ptValue = 0;
uint32_t recValue = 0;
int motorspeed,fwval,fyval,camval,lastFyval,lastFwval = 0;
//接收数据联合体
union transData
{
   long newvalue;
   byte buffer[4];
};

transData thisUnion;

void forward();
void backward();
void stop();
void turnLeft();
void turnRight();

void setup()
{
  // 初始化pcf8574对象，默认地址为0x20，如果地址改了，要改！！
 /* if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    pcf.pinMode(p, OUTPUT);
  }*/
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = 4;  // 设置传送位数，16位是2，32位是4；
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();
  Serial.begin(115200);
   // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  //舵机初始化
  fwservo.attach(6);
  fwservo.write(90);//回到中间位置
  fyservo.attach(7);
  fyservo.write(90);//回到中间位置
  //控制引脚设置
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);

}
 
void loop()
{
  if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
  Mirf.getData((byte *) &recValue);
  Serial.print("Recive Data: "); 
  Serial.println(recValue,BIN); 
  fx = (recValue & 0xff00) >> 8;
  //fx = thisUnion.buffer[1];
  ym = recValue & 0xff;
  //ym = thisUnion.buffer[0];
  con_value = (recValue >> 24) & 0xff;
  //con_value = thisUnion.buffer[3];
  ptValue = (recValue >> 16) & 0xff;
  //ptValue = thisUnion.buffer[2];
  Serial.print("fx = ");
  Serial.print(fx);
  Serial.print(",");
  Serial.print("ym = ");
  Serial.println(ym);
  Serial.print("con_value: "); 
  Serial.println(con_value,BIN);  
  Serial.print("PaoTa: ");
  Serial.println(ptValue,BIN);
 
 }
 delay(100);
/*
  //有刷电机油门控制
  if(ym >= 120 and ym <= 136) {
    stop();
  }
  if(ym > 136 and ym < 256) {
    forward();
  }
  if(ym >= 0 and ym < 120) {
    backward();
  }
  //方向控制
  if(fx > 136 and fx < 256) {
    turnRight();
  }
  if(fx >= 0 and fx < 120) {
    turnLeft();
  }
  // 控制值处理
  //pcf.digitalWriteByte(con_value);
  //炮台控制 低4位为方位，高4位为俯仰
  //方位控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  fwval = ptValue & 0x0f;
  if(fwval != lastFwval){
    fwval = map(fwval,0,15,160,10);  //160,10为舵机的可转动范围，根据需要调整；
    fwservo.write(fwval);
    lastFwval = fwval;
  }

  //俯仰控制 如果同上次的值对比，有变化，再操作舵机，避免每次对舵机进行操作。
  fyval = ptValue & 0xf0;
  if(fyval != lastFyval){
    fyval = map(fyval,0,15,90,135);
    fyservo.write(fyval);
    lastFyval = fyval;
  }
*/
}
// 电机前进
void forward(){
    digitalWrite(left1, HIGH);
    digitalWrite(left2, LOW);
    digitalWrite(right1, HIGH);
    digitalWrite(right2, LOW);
}
// 电机后退
void backward(){
    digitalWrite(left1, LOW);
    digitalWrite(left2, HIGH);
    digitalWrite(right1, LOW);
    digitalWrite(right2, HIGH);
}
// 电机停止
void stop(){
    digitalWrite(left1, LOW);
    digitalWrite(left2, LOW);
    digitalWrite(right1, LOW);
    digitalWrite(right2, LOW);
}
//左转，左轮后退，右轮前进
void turnLeft(){
    digitalWrite(left1, LOW);
    digitalWrite(left2, HIGH);
    digitalWrite(right1, HIGH);
    digitalWrite(right2, LOW);
}
//右转 左轮前进，右轮后退
void turnRight(){
    digitalWrite(left1, HIGH);
    digitalWrite(left2, LOW);
    digitalWrite(right1, LOW);
    digitalWrite(right2, HIGH);    
}