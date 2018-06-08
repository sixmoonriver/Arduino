#include <Servo.h>
Servo servo_pin_12;
int pos = 0;
void setup()
{
servo_pin_12.attach(12);
}

void loop()
{
  for(pos = 0; pos < 180; pos += 1)  // 从0度到180度运动   
  {
    servo_pin_12.write( pos );
    delay(15);
  }
   for(pos = 180; pos>=1; pos-=1)   //从180度到0度运动  
  {                                
    servo_pin_12.write( pos );    // 指定舵机转向的角度 
    delay(15);                        // 等待15ms让舵机到达指定位置 
  } 
}
