/*
encore-radio�ѻ����� arduino+mcp2515
����:2018.6.9
*/

#include <mcp_can.h>
#include <SPI.h> 
#include <IRremote.h>

MCP_CAN CAN0(10);     // Set CS to pin 10
int fanPin=4;
int fanSpeed=200;
byte data4[1] = {};
byte data5[1] = {0x05};
//632  0x00, 0x48, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00
byte data6[8] = {0x00, 0x48, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte data[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00};
byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte datavolup[4] = {0x00,0x00,0x00,0x01};
byte datavoldown[4] = {0x00,0x00,0x00,0x1E};
byte datasource[4] = {0x02,0x00,0x00,0x00};
byte datakong[4] = {0x00,0x00,0x00,0x00};
byte data22[1] = {0x02};
byte data23[1] = {0x03};
byte data24[1] = {0x04};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bool currentpowerstate = false;
bool powerstate = false;
int ledpin = 13;
 
int RECV_PIN = 9; // ����һ�廯����ͷ���ӵ�Arduino 9������
IRrecv irrecv(RECV_PIN);
decode_results results; // ���ڴ洢�������Ķ���

void setup()
{
Serial.begin(115200);
// Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
else Serial.println("Error Initializing MCP2515...");
CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
irrecv.enableIRIn(); // ��ʼ���������
pinMode(ledpin,OUTPUT);
pinMode(fanPin,OUTPUT);
digitalWrite(ledpin, HIGH);
}
 
void powerControl(bool powerstate)
{
//Serial.println(powerstate);
//Serial.println(lastpowerstate); 
if(powerstate) //���״̬�ǿ���,ִ�п���
  {  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    digitalWrite(fanPin, HIGH);
    byte sndStat3 = CAN0.sendMsgBuf(0x100, 0, 0, data);
    byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
    byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
    delay(100);
    Serial.println("Powerstate  is ON!");
  }
else //��֮���ж��ϴ�״̬�ǲ��ǿ����������ִ�йػ�����ͬʱ���ϴιػ�״̬����Ϊ�ػ���
{
  digitalWrite(fanPin, LOW);
  byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
    Serial.println("Power is turning off!");
  delay(10000);
}
}
 
void loop() {
 if (irrecv.decode(&results))
{
switch(results.value){
  case 1886400719:{ 
    currentpowerstate = !currentpowerstate; 
    powerControl(currentpowerstate);
    break;
  }
  case 1886398679:{
    //  byte sndStat6 = CAN0.sendMsgBuf(0x632, 0, 8, data6);
      if(fanSpeed<255){
        analogWrite(fanPin, fanSpeed++);  
      }
      Serial.println("vol up");
      break;
  }
  case 1886431319:{
      if(fanSpeed>0){
        analogWrite(fanPin, fanSpeed--);  
      }
      Serial.println("vol down");
      break;
  }
  case 5674:{
//      byte sndStat6 = CAN0.sendMsgBuf(0x632, 0, 8, data6);
      byte sndStat4 = CAN0.sendMsgBuf(0x10220081, 1, 1, data23);
      Serial.println("left");
      break; 
  }
  case 26154:{
   //   byte sndStat6 = CAN0.sendMsgBuf(0x632, 0, 8, data6);
      byte sndStat4 = CAN0.sendMsgBuf(0x10220081, 1, 1, data24);
      Serial.println("right");
      break;
  }
  case 29994:{
    //  byte sndStat6 = CAN0.sendMsgBuf(0x632, 0, 8, data6);
      byte sndStat4 = CAN0.sendMsgBuf(0x100D0060, 1, 4, datasource);
      byte sndStat6 = CAN0.sendMsgBuf(0x100D0060, 1, 4, datakong);
      Serial.println("input");
      break;
  }
}

irrecv.resume(); // ������һ������
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