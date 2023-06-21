/*
  船模控制2.0版本
  1、GT24模块进行通信，传输24位的数据，低8位是油门、次8位是方向，再高位是功能开关。
  2、接线方法：设置主电机（电调）的控制脚为3，方向舵机的控制引脚为5，摄像头舵机的控制引脚为6，第1位开关的控制输出为4；
  
*/

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h>

Servo mastermotor;  // create servo object to control a servo
Servo fxservo; // 建立方向舵机对象
Servo camservo; // 建立摄像头方向舵机对象

uint8_t ym,fx,con_value;
uint32_t value;
int motorspeed,fxval,camval;
int switch1;
int switch1Pin = 4;

void setup()
{

    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = 4;  // 设置传送位数，16位是2，32位是4；
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();
  Serial.begin(9600);
   // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  mastermotor.attach(6);  // 设置主电机的控制脚为6，arduino 上只能是3 5 6 9 10 11这几个支持PWM输出的引脚。
  //以下为电调初始化，首先把油门行程设置为最大，延迟2秒后，把油门行程设置为最小。
  mastermotor.writeMicroseconds(2000);
  delay(2000);
  mastermotor.writeMicroseconds(10);
  fxservo.attach(5); // 设置方向舵机的控制引脚为5；
  fxservo.write(90); // 舵板初始化到中间的位置；
  camservo.attach(3); // 设置摄像头舵机的控制引脚；
  Serial.println("Init OK!");
}
 
void loop()
{
   if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &value);
  //fx = highByte(value);
  //ym = lowByte(value);
  fx = (value & 0xff00) >> 8;
  ym = value & 0xff;
  con_value = (value & 0xffff0000) >>16;
  Serial.print("fx = ");
  Serial.print(fx);
  Serial.print(",");
  Serial.print("ym = ");
  Serial.println(ym);
  Serial.print("Switch and CAM: ");
  Serial.println(con_value,BIN);
  delay(100);
 }
  //转换油门值为油门实际值，进行油门控制
  motorspeed = map(ym, 130, 255, 1000,2000); 
  mastermotor.writeMicroseconds(motorspeed); 
  fxval = map(fx,0,255,60,120); //转换方向舵的值； 控制舵的偏转范围，45-->60 
  fxservo.write(fxval);
  //控制数据处理
  //1、摄像头方向控制
  camval = con_value & 0x0f;
  camval = map(camval,0,255,45,135);
  camservo.write(camval);
  //2、开关信号处理
  switch1 = bitRead(con_value,7); 
  digitalWrite(switch1Pin,switch1);
}
