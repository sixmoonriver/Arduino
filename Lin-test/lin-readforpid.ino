#include "Arduino.h"
#include <SoftwareSerial.h>
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 250;
SoftwareSerial Serial1(10, 11); // RX, TX

//Lin Initailisation
#define linspeed                19200
unsigned long Tbit = 1000000/linspeed;

#define uartlenght                 10

//Tbits Header
#define breakfield                 13
#define breakdelimiter              1
#define breakfieldinterbytespace    2

int frameerrordelay = ((breakfield + breakdelimiter) - uartlenght)*Tbit;
//int frameerrordelay = (breakfield + breakdelimiter + breakfieldinterbytespace) - uartlenght;

#define syncfield                  uartlenght
#define PIDfield                   uartlenght
#define syncfieldPIDinterbytedelay  0
int syncfieldPIDinterbytespace = syncfieldPIDinterbytedelay*Tbit;

//Tbit Response
#define responsedelay               8
int responsespace = responsedelay*Tbit;
#define interbytedelay              0
int interbytespace = interbytedelay*Tbit;

#define numbers  4
byte message[numbers],sending[numbers];
byte linb,sync,idbyte,PID,checksum;
byte myPID= 26;
int n = 0;
/*  前面的抓包的PID，除0以外，可以试着指定PID抓包，看下能否找到收音机的数据； 
PID:  0
PID:  1E
PID:  26
PID:  36
PID:  38
PID:  3E
PID:  6
*/

void setup() {
  // initialize serial port and LIN:
  Serial.begin(19200);
  Serial1.begin(19200);  
  pinMode(13, OUTPUT);
}


void loop() {

if (Serial1.available() > 0) {
  sync = Serial1.read();
  if (sync != 0x55) {
    sync = 0x00;
  }
  delayMicroseconds(syncfieldPIDinterbytespace);  //Interbyte Space
  if (Serial1.available() > 0) {
    PID = Serial1.read();
    if (PID == myPID){    // 如果pid不是指定的，直接退出，进入下个
      Serial.print("PID:  ");
      Serial.println(PID & 0x3F,HEX);   
    
    delayMicroseconds(responsespace);               //after PID Tbit space
    for (int i=0;i<numbers;i++) {
      message[i] = Serial1.read();
      Serial.print("  Message:  ");
    Serial.print(message[i],HEX);
      Serial.println(";");
      if (interbytespace == 0) {                    //Interbyte Space
        delayMicroseconds(1);
      } else {
        delayMicroseconds(interbytespace);
      }
    }
    checksum = Serial1.read();
    Serial.print("  Checksum:  ");
    Serial.println(checksum,HEX);
    }
  }
}
//  LinWriting();
//  LinResponding();
}