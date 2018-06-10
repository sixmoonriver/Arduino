/* 
 *  4脚接运行灯，绿色，控制二级管阴极
 *  3脚接按键，按下按键为低电平
 */
#include <mcp_can.h>
#include <SPI.h> 

MCP_CAN CAN0(10);     // Set CS to pin 10
byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

const int buttonPin = 3;
const int ledPin = 4;
int ledState = HIGH;
int lastledState = HIGH;
int buttonState;
int lastButtonState = LOW;
// 定义记录最近一次抖动的时间变量，
// 并初始化时间为0毫秒。
long lastDebounceTime = 0;
// 定义延迟抖动的时间变量，
// 并初始化为50毫秒。
long debounceDelay = 50;

// 对Arduino电路板或相关状态进行初始化方法
void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  Serial.begin(115200);
// Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
    CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    // 如果按键发生了变化，重新设置最近一次抖动的时间。
    lastDebounceTime = millis();
  }
  // 判断按键按下或抬起的状态时间间隔是否大于延迟抖动的时间长度。
  // 方法millis()可以获取当前时间，单位统一为毫秒。
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // 判断当前的按键状态是否和之前有所变化
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
		lastledState = ledState;
        ledState = !ledState;
      }
    }
  }
  // 最终改变电路板上LED神灯的状态
  digitalWrite(ledPin, ledState);
  if(ledState == HIGH)
  {
	if(lastledState == LOW)
	{
		byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
		Serial.println("Power is turning off!");
		delay(30000);
		lastledState = ledState;
	}
	Serial.println("Power has already OFF");
  }
  else
  {
	byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
    byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
    delay(100);
    Serial.println("Power is ON!");
  }
  // 更新按键最近一次状态变化的变量
  lastButtonState = reading;
}