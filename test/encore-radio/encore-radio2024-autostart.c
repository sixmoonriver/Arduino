/*
2024年基于新电路板的功能优化
1、根据aux切换开关（接D5）的高电平判断开机，定时输出can心跳信号，启动风扇（D4）；
2、通过关机按键（D6）进行关机；


*/

#include <mcp_can.h>
#include <SPI.h> 
//#include <IRremote.h>

MCP_CAN CAN0(10);     // Set CS to pin 10

byte data4[1] = {};
//byte data[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00};
byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bool powerstate = false;
bool lastpowerstate = false;

unsigned long count=0;
unsigned long lasttime =0;
int interval=7000; 

// 定义引脚
int powerOnPin = 5;  //aux电平检测
int offButtonPin = 6; //关机引脚
int fanPin=4; //风扇控制

//int buttonState = HIGH;
// 定义记录按键最近一次状态变化的变量，
// 并初始化状态为LOW。
//int lastButtonState = HIGH;
// 定义记录最近一次抖动的时间变量，
// 并初始化时间为0毫秒。
long lastDebounceTime = 0;
// 定义延迟抖动的时间变量，
// 并初始化为50毫秒。
long debounceDelay = 50;
//Bounce debouncer = Bounce();

void setup()
{
  Serial.begin(115200);
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  //irrecv.enableIRIn(); // 初始化红外解码
  pinMode(powerOnPin,INPUT);
  pinMode(offButtonPin, INPUT_PULLUP);
  pinMode(fanPin,OUTPUT);
  digitalWrite(fanPin, LOW);
}
 

void loop() {
// 开机控制，如果aux开关是开机，状态是关机，就开机；
if(digitalRead(powerOnPin)){
  if(!powerstate){
    powerstate = true;
    digitalWrite(fanPin, HIGH);
    byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
		byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
		digitalWrite(fanPin, HIGH);
		delay(100);
		Serial.println("Power turns ON!");
		//记录时间开机时间，便于下一步进行持续的发送心跳信号
		lasttime=millis();
  }
}
if(!digitalRead(offButtonPin)){
	delay(debounceDelay);
	if(!digitalRead(offButtonPin)){
		//Serial.print("buttonState: ");
		//Serial.println(buttonState);
		//如果电源状态是开机，执行关机
		if(powerstate){
			byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
			digitalWrite(fanPin, LOW);
			Serial.println("Power turns off!");
			delay(300);
      powerstate = false;
		}
	}
}
	//判断按键状态是否变化，如果没变化，继续循环

	
	if(powerstate){ //状态如果是开机，如果持续时间超过间隔，发送开机代码，重置lasttime；
		if((millis()-lasttime)>=interval){
			byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
			byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
			delay(100);
			Serial.println("Power is staying ON!");
			lasttime=millis();
  }
}
	
} 