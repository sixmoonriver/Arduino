#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 1;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // 设置舵机的控制脚为9，arduino 上只能是3 5 6 9 10 11这几个支持PWM输出的引脚。
  //以下为电调初始化，首先把油门行程设置为最大，延迟2秒后，把油门行程设置为最小。
  myservo.writeMicroseconds(2000);
  delay(2000);
  myservo.writeMicroseconds(10);
  Serial.begin(115200);
  Serial.println("Init OK!");
}

void loop() {
  val = analogRead(potpin); 
  Serial.println(val);           // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 512, 0, 1000,1500);     // 设置电调和输入控制的映射关系，电调在1000是停止状态，2000是最大转速；
  
  myservo.writeMicroseconds(val); 
 // sets the servo position according to the scaled value
  delay(15);                           // 这个延迟为影响控制的响应时间。
}