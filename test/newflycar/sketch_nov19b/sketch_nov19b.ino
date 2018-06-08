// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  // turn on motor
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}  

void loop() {
  uint8_t i;
  char FangXiang = "FORWARD";
/* 读取控制引脚电压，A5为X轴数值，A4为Y轴数值；
   正常情况500为中间值，小于520，小车前进，大于520，小车倒退；

*/

  int R_LValue = analogRead(A5);
  R_LValue = map(R_LValue,0,1023,0,70);
  int F_BValue = analogRead(A4);
  if (F_BValue < 520) {
     F_BValue = map(F_BValue,0,520,255,50);
     FangXiang = FORWARD;
  }   else {
       F_BValue = map(F_BValue,1024,520,255,50);  
     FangXiang = BACKWARD; 
  }

  motor1.run(FangXiang);
  motor2.run(FangXiang);
  motor3.run(FangXiang);
  motor4.run(FangXiang);
  motor1.setSpeed(F_BValue);
  motor2.setSpeed(F_BValue);
  motor3.setSpeed(F_BValue);
  motor4.setSpeed(F_BValue);
 
 Serial.print("FangXiang is ");
 Serial.println(FangXiang);
 Serial.print("Right or left  ");
 Serial.println(R_LValue);
 Serial.print("Forward backward  ");
 Serial.println(F_BValue);
 
 delay(200);
}
