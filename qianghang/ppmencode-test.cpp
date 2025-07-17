#include "PPMEncoder.h"
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
//#include <C:\Users\Administrator\Documents\Arduino\libraries\Adafruit-Motor-Shield-library\AFMotor.h>
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
#define OUTPUT_PIN 10

//使用16位传输数据，最高4位为帧编号，用于区分不同的信号，最多16种，暂时只用1~6
#define ROLLFRAM 1
#define PITCHFRAM 2
#define THROTFRAM 3
#define YAWFRAM 4
#define JTFRAM 5
#define CTRFRAM 6 

//接收数据联合体
union transData
{ 
   long newvalue;
   byte buffer[4];
};
transData recUnion,sendUnion;

int fw,fy,ym,fx=50;
uint16_t recValue = 0;
int channel1,channel2,channel3,channel4,channel5,con_value=0;
long lastprint=0;

void setup() {
  Serial.begin(115200);
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"RECVE"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = 2;  // 设置传送位数，16位是2，32位是4；
  Mirf.channel = 85;              //设置所用信道 无人机控80，枪控、致迪90
  Mirf.config();

  // Read and print RF_SETUP 无线模块初始化检查
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
  delay(100);
  ppmEncoder.begin(OUTPUT_PIN);
}

void loop() {
  if(Mirf.dataReady()) {  //当接收到程序，便从串口输出接收到的数据
  Mirf.getData((byte *) &recValue);
  recUnion.newvalue = recValue;
  // DEBUG("Recive Data: "); 
  // DEBUGL(recValue,BIN); 
  // 提取第11~14位（共4位）的值
  unsigned int frameType = (recValue >> 10) & 0xF;  // 右移11位，取低4位
  
  // 根据提取的值执行对应分支
  switch (frameType) {
      case ROLLFRAM:
          channel1 = recValue & 0x3FF;
          break;
      case PITCHFRAM:
          channel2 = recValue & 0x3FF;
          break;
      case THROTFRAM:
          channel3 = recValue & 0x3FF;
          break;
      case YAWFRAM:
          channel4 =  recValue & 0x3FF;
          break;
      case JTFRAM:
          channel5 = recValue & 0x3FF;
          break;
      case CTRFRAM:
          con_value = recValue & 0x3FF;
          break;
      // case 0:
      //     break;
      default:
          DEBUG("unknown data type: ");
          DEBUGL(frameType);
          break;
  }
  //0.5秒打印一次，
  if(millis()-lastprint >= 1000){
    lastprint = millis();
    DEBUG("channel1 = ");
    DEBUGL(channel1);
    DEBUG("channel2 = ");
    DEBUGL(channel2);
    DEBUG("channel3 = ");
    DEBUGL(channel3);
    DEBUG("channel4 = ");
    DEBUGL(channel4);
    DEBUG("channel5 = ");
    DEBUGL(channel5);
    DEBUG("con_value = ");
    DEBUGL(con_value);
  }

//   ppmEncoder.setChannel(0,map(channel1,0,1023,1000,2500));
//   ppmEncoder.setChannel(1,map(channel2,0,1023,1000,2500));
//   ppmEncoder.setChannel(2,map(channel3,0,1023,1000,2500));
//   ppmEncoder.setChannel(3,map(channel4,0,1023,1000,2500));
//   ppmEncoder.setChannel(4,map(channel5,0,1023,1000,2500));
  ppmEncoder.setChannelPercent(0,map(channel1,985,0,0,55));
  ppmEncoder.setChannelPercent(1,map(channel2,985,0,0,55));
  ppmEncoder.setChannelPercent(2,map(channel3,985,0,0,55));
  ppmEncoder.setChannelPercent(3,map(channel4,985,0,0,55));
  ppmEncoder.setChannelPercent(4,map(channel5,985,0,0,55));
  for(int i=5;i<8;i++){
    if(bitRead(con_value,i-5)){
         ppmEncoder.setChannelPercent(i,55);
      }
      else{
         ppmEncoder.setChannel(i,0);
      }
  }
  // ppmEncoder.setChannel(5,map(bitRead(con_value,0),0,1,500,2500));
  // ppmEncoder.setChannel(6,map(bitRead(con_value,1),0,1,500,2500));
  // ppmEncoder.setChannel(7,map(bitRead(con_value,2),0,1,500,2500));
 }
 delay(2); //这个延迟要放到这里，否则程序错乱，一直会有垃圾数据输出
  // fw=analogRead(A1);
  // fy=analogRead(A2);
  // ym=analogRead(A3);
  // fx=analogRead(A4);
  // Min value
  // ppmEncoder.setChannel(0, 500);
  // ppmEncoder.setChannel(0, PPMEncoder::MIN);
  // ppmEncoder.setChannelPercent(0, 0);

  // // Max value
  // ppmEncoder.setChannel(0, 2500);
  // ppmEncoder.setChannel(0, PPMEncoder::MAX);
  // ppmEncoder.setChannelPercent(0, 100);

}