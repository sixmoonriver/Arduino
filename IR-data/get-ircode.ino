/*
[url=http://www.openjumper.com/]www.openjumper.com[/url]
日期:2013.5.18
update 2020.7.22 编译通过，未验证。待测试
IDE 版本:1.0.1

*/
 
 #include <IRremote.h>
 
int RECV_PIN = 9; // 红外一体化接收头连接到Arduino 8号引脚
int LEDpin = 13; //定义LED输出引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象
void setup()
{
Serial.begin(115200);
pinMode(LEDpin,OUTPUT);
irrecv.enableIRIn(); // 初始化红外解码
}
 
void loop() {
if (irrecv.decode(&results))
{
Serial.println(results.value);
/*
if(results.value == 21546)
  Serial.println( results.value);
   
if( results.value == 0x707042BD) //若接收到前进的指令，小车前进；
{
}
else if(results.value == 0xFFE21D) //接收到后退的指令的命令，小车先停止再后退；
{
}
else if(results.value == 0x7070629D) //接收到停止的指令的命令，小车先停止；
{
}
*/
irrecv.resume(); // 接收下一个编码
}
}


