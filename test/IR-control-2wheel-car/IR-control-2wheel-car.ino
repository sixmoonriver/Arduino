/*
[url=http://www.openjumper.com/]www.openjumper.com[/url]
日期:2013.5.18
IDE 版本:1.0.1
功能：红外遥控小车
*/
 
#include <IRremote.h>
#include <Servo.h>
int RECV_PIN = 11; // 红外一体化接收头连接到Arduino 11号引脚
int LEDpin = 13; //定义LED输出引脚
IRrecv irrecv(RECV_PIN);
 
decode_results results; // 用于存储编码结果的对象
 
void setup()
{
  pinMode(LEDpin,OUTPUT);
  irrecv.enableIRIn(); // 初始化红外解码
  pinMode( 2 , INPUT);
  pinMode( 7 , OUTPUT);
  pinMode( 10 , OUTPUT);
  pinMode( 3 , INPUT);
  pinMode( 5 , OUTPUT);
  pinMode( 8 , OUTPUT);
}
 
void loop() {
if (irrecv.decode(&results))
{
Serial.println( results.value);
if( results.value == 0x707042BD) //若接收到前进的指令，小车前进；
{

  digitalWrite( 5 , LOW );
  digitalWrite( 7 , HIGH );
  digitalWrite( 8 , HIGH );
  digitalWrite( 10 , LOW );
  analogWrite(6, 150);
  analogWrite(9, 150);
}
else if(results.value == 0x7070C23D) //接收到后退的指令的命令，小车先停止再后退；
{

  digitalWrite( 5 , HIGH );
  digitalWrite( 7 , LOW );
  digitalWrite( 8 , LOW );
  digitalWrite( 10 , HIGH );
  analogWrite(6, 80);
  analogWrite(9, 80);
}
else if(results.value == 0x7070629D) //接收到停止的指令的命令，小车停止；
{

  digitalWrite( 5 , HIGH );
  digitalWrite( 7 , LOW );
  digitalWrite( 8 , LOW );
  digitalWrite( 10 , HIGH );
  analogWrite(6, 0);
  analogWrite(9, 0);
}
else if(results.value == 0x707022DD) //接收到左转指令的命令，小车左转；
{

  digitalWrite( 5 , LOW );
  digitalWrite( 7 , HIGH );
  digitalWrite( 8 , HIGH );
  digitalWrite( 10 , LOW );
  analogWrite(6, 80);
  analogWrite(9, 60);
}
else if(results.value == 0x7070A25D) //接收到右转指令的命令，小车右转；
{

  digitalWrite( 5 , LOW );
  digitalWrite( 7 , HIGH );
  digitalWrite( 8 , HIGH );
  digitalWrite( 10 , LOW );
  analogWrite(6, 60);
  analogWrite(9, 80);
}
irrecv.resume(); // 接收下一个编码
}
}
