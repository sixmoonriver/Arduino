/*
yaofy
日期:2016.10.07
IDE 版本:1.0.1
功能：红外遥控小车
*/
 
#include <IRremote.h>
#include <Servo.h>
//Servo servo_pin_12; //创建舵机控制对象，舵机引脚为12
int RECV_PIN = 8; // 红外一体化接收头连接到Arduino 11号引脚
IRrecv irrecv(RECV_PIN);
int Linput1 = 4; //左电机输入通道1；
int Linput2 = 2; //左电机输入通道2；
int Lpwm = 5; //左电机PWM控制；
int Rinput1 = 7; //右电机输入通道1；
int Rinput2 = 3; //右电机输入通道2；
int Rpwm = 6; //右电机PWM控制;
int LSensor = A2; //设置左侧探测器输入引脚A0；
int MSensor = A1; //设置中间探测器输入引脚A1；
int RSensor = A0; //设置右侧探测器输入引脚A2；
int fazhi = 125;
decode_results results; // 用于存储编码结果的对象

/* 以下为超声波测距函数
int SuperWave(int trigPin, int echoPin)
{
  long duration;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  duration = duration / 59;
  if ((duration < 2) || (duration > 300)) return false;
  return duration;
}
*/
// 以下为前进函数
int forward(int Leftspeed,int Rightspeed)
{
  //servo_pin_12.write( Turnangle ); //Turnangle 直进直退为85度，前进左转为70度，右转为100度；
  digitalWrite( Rinput1 , LOW );
  digitalWrite( Rinput2 , HIGH );
  digitalWrite( Linput1 , HIGH );
  digitalWrite( Linput2 , LOW );
  analogWrite(Lpwm, Leftspeed);
  analogWrite(Rpwm, Rightspeed);
  }

// 以下为倒车函数
int backward(int Leftspeed,int Rightspeed)
{
  //servo_pin_12.write( Turnangle );  //Turnangle 直进直退为90度
  digitalWrite( Rinput1 , HIGH );
  digitalWrite( Rinput2 , LOW );
  digitalWrite( Linput1 , LOW );
  digitalWrite( Linput2 , HIGH );
  analogWrite(Lpwm, Leftspeed);
  analogWrite(Rpwm, Rightspeed);
  }

// 检测传感器
int detecksesor(int LS,int MS,int RS)
{
  int sensor=0;
  if(analogRead(LS) > fazhi)
    sensor = 4;
  else 
    sensor = 0;
  if(analogRead(MS) > fazhi)
    sensor += 2;
  else 
    sensor += 0;  
  if(analogRead(RS) > fazhi)
    sensor += 1;
  else 
    sensor += 0;  
  return sensor;
}
// 左转弯就是前进带一个左速度为零的，舵机带一个角度

void setup()
{
  irrecv.enableIRIn(); // 初始化红外解码
  pinMode( Rinput2 , OUTPUT);
  pinMode( Rinput1 , OUTPUT);
  pinMode( Linput1 , OUTPUT);
  pinMode( Linput2 , OUTPUT);
  //servo_pin_12.attach(12);
  Serial.begin(115200);
}
 
void loop() {
int lspeed = 70;
int rspeed = 70; 
int sensorstate = detecksesor(LSensor,MSensor,RSensor); 
switch(sensorstate)
{
  case 0: ;
  case 7: ;
  case 2: forward(lspeed,rspeed); break;
  case 1: 
    forward(0,0);
    delay(300);
    forward(lspeed-10,rspeed+10);
    break; //快左转
  case 3: 
    forward(0,0);
    delay(300);
    forward(lspeed-5,rspeed+5);break; //慢左转
  case 4: 
    forward(0,0);
    delay(300);
    forward(lspeed+10,rspeed-10);break; //快右转
  case 6: 
    forward(0,0);
    delay(300);
    forward(lspeed+5,rspeed-5);break; //慢右转
}

Serial.print("sensorstate:");
Serial.println(sensorstate);

delay(500);
if (irrecv.decode(&results))
{

forward(85,85);
if( results.value == 0x707042BD) //若接收到前进的指令，小车前进；
{
/*
  */
  forward(85,85);
}
else if(results.value == 0x7070C23D) //接收到后退的指令的命令，小车先停止再后退；
{
  backward(70,70); 
}
else if(results.value == 0x7070629D) //接收到停止的指令的命令，小车停止；
{
  forward(0,0);
}
else if(results.value == 0x707022DD) //接收到左转指令的命令，小车左转；
{
  forward(0,70);
}
else if(results.value == 0x7070A25D) //接收到右转指令的命令，小车右转；
{
  forward(70,0);
}
irrecv.resume(); // 接收下一个编码
}
}
