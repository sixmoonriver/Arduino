#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 1;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
int stopPoint=1000;
int forwardPin = 3;
int backwardPin = 10;
void setup() {
  pinMode(forwardPin,OUTPUT);
  pinMode(backwardPin,OUTPUT);
  myservo.attach(9);  // 设置舵机的控制脚为9，arduino 上只能是3 5 6 9 10 11这几个支持PWM输出的引脚。
  //以下为电调初始化，首先把油门行程设置为最大，延迟2秒后，把油门行程设置为最小。
  // myservo.writeMicroseconds(2000);
  // delay(2000);
  // myservo.writeMicroseconds(10);
  // delay(2000);
  myservo.writeMicroseconds(1000);
  Serial.begin(115200);
  Serial.println("Init OK!");
}

void loop() {
  val = analogRead(potpin); 
  Serial.println(val);           // reads the value of the potentiometer (value between 0 and 1023)
  // 有刷电机，使用IBT-4电机驱动
  if(val>520){
    int forwardPwm = map(val,520,1023,0,255);
  //   val = map(val,512,1023,stopPoint,1500);
  //   myservo.writeMicroseconds(val);
    analogWrite(forwardPin,forwardPwm);
    analogWrite(backwardPin,0);
  }
  else if(val<504){
    int backwardPwm = map(val,504,0,0,255);
  //   val = map(val,512,1023,stopPoint,1500);
  //   myservo.writeMicroseconds(val);
    analogWrite(forwardPin,0);
    analogWrite(backwardPin,backwardPwm);
  }
  else{
    analogWrite(forwardPin,0);
    analogWrite(backwardPin,0);
  }
  //单向无刷电调
  //val = map(val, 512, 0, 1000,1500);     // 设置电调和输入控制的映射关系，电调在1000是停止状态，2000是最大转速；
  // myservo.writeMicroseconds(val);
  //针对双向无刷电调
  val = map(val,1023,0,45,135); 
  myservo.write(val);

  // else if(val<500){
  //   val = map(val,500,0,stopPoint,1500);
  
  // }
  // else{
  //   myservo.writeMicroseconds(stopPoint);
  // }
  // val = map(val, 1023, 0, 5,2000);     // 设置电调和输入控制的映射关系，电调在1000是停止状态，2000是最大转速；
  // myservo.writeMicroseconds(val); 
 // sets the servo position according to the scaled value
  delay(15);                           // 这个延迟为影响控制的响应时间。
}