/*
船模制作：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用舵机控制方向；
3、用电调无刷电机控制速度； 

*/
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
uint8_t convert[5]={0};
uint8_t value,lastvalue = 0;
uint8_t x_value,y_value,last_xvalue,last_yvalue;
uint16_t trans_value = 0
int xpin = 0;  // 方向输入脚
int ypin = 0;  // 油门输入脚

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
  x_value = map(x_value,0,1023,0,255); //将模拟量的10位值转为8位值；
  y_value = analogRead(ypin);
  y_value = map(y_value,0,1023,0,255);
  //value = random(255); 
  trans_value = x_value<<8+y_value;
  if(x_value != last_xvalue | y_value != last_yvalue)
  {
    Mirf.send((byte *)&trans_value);                //发送指令，组合后的数据，低8位为油门数据，高8位为方向数据
    while(Mirf.isSending()) delay(1);         //直到发送成功，退出循环
    last_xvalue = x_value;   //last数据更新
	last_yvalue = y_value;
  }
    delay(100);
}
int debug=1;
Servo fxservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  fxservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  //方向控制 debug方式，读取0脚的电位器分压值，否则读取其它方式的输入值；
  if(debug){
	val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
	val = map(val, 0, 1023, 0,180);     // scale it to use it with the servo (value between 0 and 180)  
  } 
  else{
	val = 90; //如果需要接遥控设备，需要处理
  }

  fxservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}