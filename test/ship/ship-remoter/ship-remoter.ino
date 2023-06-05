/*
船模制作：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用舵机控制方向；
3、用电调无刷电机控制速度； 

*/

// 遥控器部分代码
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
uint8_t convert[5]={0};
uint8_t value,lastvalue = 0;
uint8_t last_xvalue,last_yvalue,xf_value,yf_value;
uint16_t trans_value;
uint16_t x_value,y_value;
int xpin = 0;  // 方向输入脚
int ypin = 1;  // 油门输入脚

void setup()
{
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"SENDE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = sizeof(16);      //设置传送位数，每通道使用8位；
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();
    Serial.begin(9600);
     // Read and print RF_SETUP 用于检测是否初始化正常；
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
}
 
void loop()
{
  Mirf.setTADDR((byte *)"RECVE");           //设置接收端地址
  x_value = analogRead(xpin);  //读取摇杆输入的数值
  xf_value = map(x_value,0,1023,0,255); //将模拟量的10位值转为8位值；
  y_value = analogRead(ypin);
  yf_value = map(y_value,0,1023,0,255);
  //value = random(255); 
  trans_value = ((xf_value&0xff)<<8) | yf_value;
  if(xf_value != last_xvalue | yf_value != last_yvalue)
  {
    Mirf.send((byte *)&trans_value);                //发送指令，组合后的数据，低8位为油门数据，高8位为方向数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环
    last_xvalue = xf_value;   //last数据更新
    last_yvalue = yf_value;
    Serial.print("xf_value: ");
    Serial.println(xf_value);
    Serial.print("yf_value: ");
    Serial.println(yf_value);
    Serial.print("trans_value: ");
    Serial.print(highByte(trans_value));
    Serial.print(",");
    Serial.println(lowByte(trans_value));
  }
    delay(100);
}
