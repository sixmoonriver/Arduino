/*
encore-radio脱机运行 arduino+mcp2515
日期:2018.6.9
*/

#include <mcp_can.h>
#include <SPI.h> 
#include <IRremote.h>

MCP_CAN CAN0(10);     // Set CS to pin 10

byte data4[1] = {};
byte data5[1] = {0x05};
//byte data[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00};
byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte data22[1] = {0x02};
byte data23[1] = {0x03};
byte data24[1] = {0x04};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bool currentpowerstate = false;
bool powerstate = false;
int ledpin = 4;
 
int RECV_PIN = 9; // 红外一体化接收头连接到Arduino 9号引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象

void setup()
{
Serial.begin(115200);
// Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
else Serial.println("Error Initializing MCP2515...");
CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
irrecv.enableIRIn(); // 初始化红外解码
pinMode(ledpin,OUTPUT);
digitalWrite(ledpin, HIGH);
}
 
void powerControl(bool powerstate)
{
//Serial.println(powerstate);
//Serial.println(lastpowerstate); 
if(powerstate) //如果状态是开机,执行开机
  {  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    digitalWrite(ledpin, LOW);
    byte sndStat3 = CAN0.sendMsgBuf(0x100, 0, 0, data);
    byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
    byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
    delay(100);
    Serial.println("Powerstate  is ON!");
  }
else //反之，判断上次状态是不是开机，如果是执行关机程序，同时将上次关机状态设置为关机；
{
  digitalWrite(ledpin, HIGH);
  byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
    Serial.println("Power is turning off!");
  delay(10000);
}
}
 
void loop() {
 if (irrecv.decode(&results))
{
switch(results.value){
	case 21546:{ 
		currentpowerstate = !currentpowerstate; 
		powerControl(currentpowerstate);
		break;
	}
	case 9258:{
		  byte sndStat4 = CAN0.sendMsgBuf(0x220, 0, 1, data2);
		  Serial.println("vol up");
		  break;
	}
	case 25642:{
		  byte sndStat4 = CAN0.sendMsgBuf(0x220, 0, 1, data22);
		  Serial.println("vol down");
		  break;
	}
	case 5674:{
		  byte sndStat4 = CAN0.sendMsgBuf(0x220, 0, 1, data23);
		  Serial.println("left");
		  break; 
	}
	case 26154:{
		  byte sndStat4 = CAN0.sendMsgBuf(0x220, 0, 1, data24);
		  Serial.println("right");
		  break;
	}
	case 29994:{
		  byte sndStat4 = CAN0.sendMsgBuf(0x220, 0, 1, data5);
		  Serial.println("input");
		  break;
	}
}

irrecv.resume(); // 接收下一个编码
} 
if(currentpowerstate)
{
    byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
    byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
    delay(100);
    Serial.println("Power on!");	
}
//Serial.print("Power state is:  ");
//Serial.println(currentpowerstate);
}