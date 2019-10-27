#include "Arduino.h"
#include <SoftwareSerial.h>
int Rdinput = 12; 
int recmd = 0;
int ledState = 0;
SoftwareSerial mySerial1(10, 11); // RX, TX 软串口对象

int lightState = 0;             // 记录灯的状态
int controlState = 0;   // 控制状态，0表示自动控制，1表示手动控制。
// 以下代码以long类型声明，因为时间值以毫秒为单位(用整型会很快溢出)
long startTime = 0; //灯亮的开始时间；
long durationTime = 10000; //灯持续的时间；10000为10秒


// 获取串口指令，开机返回1，关机返回0；
int getremotecmd(){
  if (mySerial1.available() > 0 ){
    recmd =  mySerial1.read(); 
    if((recmd >= 48) and (recmd <= 49)){
    return recmd;   
  }
  }
  else
    return 0;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(12,INPUT);
  Serial.begin(115200);
  mySerial1.begin(115200);
}
/*
  逻辑分析：
  1、如果雷达检测到人或者远程指令开机，且灯关的状态，执行开机指令，并且状态置为1，记录开始时间；
  2、只是检测到远程开、关机指令，执行开、关机指令，手动打开只能手动关闭；
  3、如果雷达检测不到人，判断是否超过延迟时间，超过就关机，没超退出；

*/
void loop() {
  // put your main code here, to run repeatedly:
  if( getremotecmd() == 49 )
  {
    digitalWrite(13,HIGH);
    controlState = 1; 
  //startTime = millis();
    mySerial1.println("Light is turning ON"); 
  }
  else if( getremotecmd() == 48 )
  {
  digitalWrite(13,LOW);
    mySerial1.println("Light is turning OFF"); 
    }
  if(digitalRead(Rdinput))
  {
    if(!digitalRead(13)){
    digitalWrite(13,HIGH);
    controlState = 0; 
    startTime = millis();
    mySerial1.println("Light is auto ON");
    }
  }
  else
   {
//  Serial.println("rdinput low");
  if(digitalRead(13)){
  /*    Serial.println("Light has already on");
      Serial.print("Current:");
      Serial.println(millis());
      Serial.print("starttime:");
      Serial.println(startTime);      */
    if(((millis()-startTime) > durationTime) and (!controlState))
    {
      digitalWrite(13,LOW); 
      mySerial1.println("Light is auto OFF");
    }   
  }

    }
  delay(1500);
}



