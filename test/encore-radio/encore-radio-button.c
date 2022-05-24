/*
encore-radio脱机运行 arduino+mcp2515
日期:2021.6.13,使用按键来控制开关机

*/
/* 
 *  4脚接运行灯，绿色，控制二级管阴极
 *  3脚接按键，按下按键为低电平
 */
#include <mcp_can.h>
#include <SPI.h> 
#include <Bounce2.h>

// 定义按键输入针脚号常量，
// 并初始化为2号针脚。
const int buttonPin = 3;
const int ledPin = 4;
int fanPin=5;
// 定义记录LED神灯当前状态的变量，
// 并初始化状态为LOW（关）。
int ledState = LOW;
// 定义记录按键当前状态的变量
int buttonState = HIGH;
// 定义记录按键最近一次状态变化的变量，
// 并初始化状态为LOW。
int lastButtonState = HIGH;
// 定义记录最近一次抖动的时间变量，
// 并初始化时间为0毫秒。
long lastDebounceTime = 0;
// 定义延迟抖动的时间变量，
// 并初始化为50毫秒。
long debounceDelay = 50;
Bounce debouncer = Bounce();


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
 

void setup()
{
Serial.begin(115200);
// Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
else Serial.println("Error Initializing MCP2515...");
CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
pinMode(ledPin,OUTPUT);
pinMode(fanPin,OUTPUT);
// 设置按键的针脚为输入状态
pinMode(buttonPin, INPUT_PULLUP);
//绑定按键引脚
//debouncer.attach(buttonPin);
//设置去抖动时间； 
//debouncer.interval(debounceDelay);
//初始化输出引脚状态为关，灯灭
digitalWrite(fanPin, LOW);
digitalWrite(ledPin, ledState);
}

void loop() {
	//debouncer.update();//更新
	//buttonState=debouncer.read();
if(!digitalRead(buttonPin)){
	delay(debounceDelay);
	if(!digitalRead(buttonPin)){
		//Serial.print("buttonState: ");
		//Serial.println(buttonState);
		//如果电源状态是开机，执行关机
		if(powerstate){
			byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
			digitalWrite(fanPin, LOW);
			digitalWrite(ledPin, LOW);
			Serial.println("Power turns off!");
			delay(300);
		}
		else{//如果电源是关机，执行开机
			byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
			byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
			digitalWrite(fanPin, HIGH);
			digitalWrite(ledPin, LOW);
			delay(100);
			Serial.println("Power turns ON!");
			//记录时间开机时间，便于下一步进行持续的发送心跳信号
			lasttime=millis();
		}
		//无执行开机还是关机后，状态都要翻转
		powerstate = !powerstate; 
	//	buttonState != lastButtonState	
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
