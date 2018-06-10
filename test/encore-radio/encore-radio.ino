/*
encore-radio脱机运行 arduino+mcp2515
日期:2018.6.9

*/

#include <mcp_can.h>
#include <SPI.h> 
#include <IRremote.h>

MCP_CAN CAN0(10);     // Set CS to pin 10

byte data4[1] = {};
byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bool poweron = false;
/* 
int RECV_PIN = 9; // 红外一体化接收头连接到Arduino 9号引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象
*/
void setup()
{
Serial.begin(115200);
// Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
else Serial.println("Error Initializing MCP2515...");
CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
irrecv.enableIRIn(); // 初始化红外解码
}
 
void loop() {
/* if (irrecv.decode(&results))
{
if(results.value == 21546)
  poweron = !poweron;
*/   
if(poweron) //如果当前状态是开机，发送开机信号；
{  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  //byte sndStat3 = CAN0.sendMsgBuf(0x100, 0, 0,data4);
  byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
  byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
  delay(100);
  Serial.println("PowerON!");
}
else //反之，发送关机信号；
{
  byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
  delay(100);
  Serial.println("PowerOFF!");
}
/*irrecv.resume(); // 接收下一个编码
} */
}
