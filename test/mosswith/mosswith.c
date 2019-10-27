/*
2019.10.27
测试大功率MOS管 场效应管 触发开关驱动模块 PWM调节电子开关控制板，通过红外遥控控制。
test-Nmos
arduino6----> trig

测试结果：
	输出需要接电容，否则无法工作 ，电容容量大小很重要，否则输出控制的线性不准确。
	但是如果只使用开关功能，可以不接电容。
*/
#include <IRremote.h>

// 定义引脚：

int SW = 6;
int speed = 50;
int RECV_PIN = 9; // 红外一体化接收头连接到Arduino 9号引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象

void setup()
{

irrecv.enableIRIn(); // 初始化红外解码
pinMode(SW,OUTPUT);
Serial.begin(115200);
}
 
void loop() {
 if (irrecv.decode(&results))
{
// SONY N50 遥控器上键，前进,实际测试效果N20电机逆时针；
if(results.value == 22058){
  if(speed <= 250){
	  speed += 5;
  }
  else{
	  Serial.println("speed has already been the fastest!");
  }
  analogWrite(SW, speed);
  Serial.print("speed up,speed is");
  Serial.println(speed);
}
// SONY N50 遥控器下键，后退，实际测试效果N20电机顺时针；
else if(results.value == 13866){
  if(speed >= 5){
	  speed -= 5;
  }
  else{
	  Serial.println("speed has already been the lowest!");
  }
  analogWrite(SW, speed);
  Serial.print("speed down,speed is");
  Serial.println(speed);
}
// SONY N50 遥控器Enter键，停止；
else if(results.value == 11562){
  analogWrite(SW, 0);
  Serial.println("stop");  
}


irrecv.resume(); // 接收下一个编码
} 
//Serial.println(powerstate);
//Serial.println(lastpowerstate); 

}