#define motor1in1 5                             //定义IN1引脚
#define motor1in2 7                              //定义IN2引脚
#define motor1pwm 6                             //定义ENA引脚（我的是Mega2560的板子）
#define motor2in1 2                              //定义IN3引脚
#define motor2in2 4                               //定义IN4引脚
#define motor2pwm 3                              //定义ENB引脚
void motor(int motorpin1,int motorpin2,int motorpwm,int val)  //定义一个电机转动函数
{
pinMode(motorpin1,OUTPUT);                              //输出第一个引脚
pinMode(motorpin2,OUTPUT);                             //输出第二个引脚
digitalWrite(motorpin2,0);                                    //将第二个引脚置低
digitalWrite(motorpin1,1);                                     //将第一个引脚抬高
analogWrite(motorpwm,val);                                //给EN引脚设定模拟值，设定转速
}
void setup()
{  
}
void loop()
{
  int i;
  for(i=150;i<=250;i++) //让电机的转速从100到255转动，A，B转速不一样，完成转向
  {
    motor(motor1in1,motor1in2,motor1pwm,i);             //电机A保持255匀速转动
    motor(motor2in2,motor2in1,motor2pwm,i);                  //电机B从100到255转动
 delay(200);                                                              //间隔500ms
  }
  /*delay(1000);
    for(i=100;i<=220;i++) //让电机的转速从100到255转动，A，B转速不一样，完成转向
  {
 //   motor(motor1in1,motor1in2,motor1pwm,100);             //电机A保持255匀速转动
    motor(motor2in2,motor2in1,motor2pwm,i);                  //电机B从100到255转动
 delay(200);                                                              //间隔500ms
  }
  */
 /* for(i=100;i<=255;i++)                                  //让电机从100到255转动，功能同上
  {
    motor(motor1in2,motor1in1,motor1pwm,100);
    motor(motor2in2,motor2in1,motor2pwm,i);
    delay(200);
  if (i<255); 
  {
  delay(2000);
  } 
  
  }
 /* motor(motor1pin1,motor1pin2,motor1pwm,255);                 //电机A以最大转速转动
  delay(5000);
  motor(motor1pin2,motor1pin1,motor1pwm,255);                //电机A反向转动
  delay(5000);
  */
}
