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
/*
    A0  4通道
    A1  3通道
    A2  1通道
    A3  2通道


*/


int channel1,channel2,channel3,channel4;

void setup() {
  ppmEncoder.begin(OUTPUT_PIN);
}

void loop() {
 channel1 = analogRead(A2);
 channel2 = analogRead(A3);
 channel3 = analogRead(A1);
 channel4 = analogRead(A0);
 
//   ppmEncoder.setChannel(0,map(channel1,0,1023,1000,2500));
//   ppmEncoder.setChannel(1,map(channel2,0,1023,1000,2500));
//   ppmEncoder.setChannel(2,map(channel3,0,1023,1000,2500));
//   ppmEncoder.setChannel(3,map(channel4,0,1023,1000,2500));
//   ppmEncoder.setChannel(4,map(channel5,0,1023,1000,2500));
  ppmEncoder.setChannelPercent(0,map(channel1,1023,0,0,55));
  ppmEncoder.setChannelPercent(1,map(channel2,1023,0,0,55));
  ppmEncoder.setChannelPercent(2,map(channel3,1023,0,0,55));
  ppmEncoder.setChannelPercent(3,map(channel4,1023,0,0,55));
  //ppmEncoder.setChannelPercent(4,map(channel5,985,0,0,55));
  // ppmEncoder.setChannel(5,map(bitRead(con_value,0),0,1,500,2500));
  // ppmEncoder.setChannel(6,map(bitRead(con_value,1),0,1,500,2500));
  // ppmEncoder.setChannel(7,map(bitRead(con_value,2),0,1,500,2500));
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