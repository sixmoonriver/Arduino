#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Servo.h>

Servo mastermotor;  // create servo object to control a servo
Servo fxservo; // 建立方向舵机对象
//Servo qxservo; // 建立倾斜舵机对象

uint8_t ym,fx;
uint16_t value;
int motorspeed,fxval;

void setup()
{

    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();
    Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
    Mirf.payload = sizeof(16);
    Mirf.channel = 90;              //设置所用信道
    Mirf.config();
  Serial.begin(9600);
   // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  mastermotor.attach(6);  // 设置主电机的控制脚为3，arduino 上只能是3 5 6 9 10 11这几个支持PWM输出的引脚。
  //以下为电调初始化，首先把油门行程设置为最大，延迟2秒后，把油门行程设置为最小。
  mastermotor.writeMicroseconds(2000);
  delay(2000);
  mastermotor.writeMicroseconds(10);
  fxservo.attach(5); // 设置方向舵机的引脚为5；
  //qxservo.attach(11);
  Serial.println("Init OK!");
}
 
void loop()
{
   if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
    Mirf.getData((byte *) &value);
  fx = highByte(value);
  ym = lowByte(value);
    Serial.print(highByte(value));
    Serial.print(",");
  Serial.println(lowByte(value));
    delay(100);
 }
  //转换油门值为油门实际值，进行油门控制
  motorspeed = map(ym, 130, 0, 1000,2000); 
    mastermotor.writeMicroseconds(motorspeed); 
  fxval = map(fx,0,255,45,135); //转换方向舵的值；
  fxservo.write(fxval);
}
