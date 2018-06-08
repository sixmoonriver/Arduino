/*
[url=http://www.openjumper.com/]www.openjumper.com[/url]
日期:2013.5.18
IDE 版本:1.0.1
功能：红外遥控小车
*/
 
#include <IRremote.h>
#include <Servo.h>
Servo servo_pin_12; //创建舵机控制对象，舵机引脚为12
int RECV_PIN = 11; // 红外一体化接收头连接到Arduino 11号引脚
IRrecv irrecv(RECV_PIN);
int Linput1 = 2; //左电机输入通道1；
int Linput2 = 4; //左电机输入通道2；
int Lpwm = 5; //左电机PWM控制；
int Rinput1 = 3; //右电机输入通道1；
int Rinput2 = 7; //右电机输入通道2；
int Rpwm = 6; //右电机PWM控制；
decode_results results; // 用于存储编码结果的对象
 
void setup()
{
  irrecv.enableIRIn(); // 初始化红外解码
  pinMode( Rinput2 , OUTPUT);
  pinMode( Rinput1 , OUTPUT);
  pinMode( Linput1 , OUTPUT);
  pinMode( Linput2 , OUTPUT);
  servo_pin_12.attach(12);
}
 
void loop() {
if (irrecv.decode(&results))
{
Serial.println( results.value);
if( results.value == 0x707042BD) //若接收到前进的指令，小车前进；
{
 // servo_pin_12.write( 90 );
  digitalWrite( Rinput1 , LOW );
  digitalWrite( Rinput2 , HIGH );
  digitalWrite( Linput1 , HIGH );
  digitalWrite( Linput2 , LOW );
  analogWrite(Lpwm, 80);
  analogWrite(Rpwm, 80);
}
else if(results.value == 0x7070C23D) //接收到后退的指令的命令，小车先停止再后退；
{
  servo_pin_12.write( 90 );
  digitalWrite( Rinput1 , HIGH );
  digitalWrite( Rinput2 , LOW );
  digitalWrite( Linput1 , LOW );
  digitalWrite( Linput2 , HIGH );
  analogWrite(Lpwm, 70);
  analogWrite(Rpwm, 70);
}
else if(results.value == 0x7070629D) //接收到停止的指令的命令，小车停止；
{
  servo_pin_12.write( 90 );
  digitalWrite( Rinput1 , HIGH );
  digitalWrite( Rinput2 , LOW );
  digitalWrite( Linput1 , LOW );
  digitalWrite( Linput2 , HIGH );
  analogWrite(Lpwm, 0);
  analogWrite(Rpwm, 0);
}
else if(results.value == 0x707022DD) //接收到左转指令的命令，小车左转；
{
  servo_pin_12.write( 75 );
  digitalWrite( Rinput1 , LOW );
  digitalWrite( Rinput2 , HIGH );
  digitalWrite( Linput1 , HIGH );
  digitalWrite( Linput2 , LOW );
  analogWrite(Rpwm, 0);
  analogWrite(Lpwm, 0);
}
else if(results.value == 0x7070A25D) //接收到右转指令的命令，小车右转；
{
  servo_pin_12.write( 105 );
  digitalWrite( Rinput1 , LOW );
  digitalWrite( Rinput2 , HIGH );
  digitalWrite( Linput1 , HIGH );
  digitalWrite( Linput2 , LOW );
  analogWrite(Rpwm, 75);
  analogWrite(Lpwm, 0);
}
irrecv.resume(); // 接收下一个编码
}
}
