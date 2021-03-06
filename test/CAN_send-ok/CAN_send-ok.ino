// CAN Send Example
//

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10);     // Set CS to pin 10

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_33K3BPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

byte data[8] = {0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte data[8] = {0x00, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data2[1] = {0x01};
byte data3[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int count=1;
void loop()
{

  delay(100);   // send data per 100ms
  count+=1;
  if(count>=500)
  {
   // count=0;
     byte sndStat3 = CAN0.sendMsgBuf(0x62c, 0, 8, data3);
     Serial.println("data3 sent.");
     delay(100000);
    }
   else
   {
      // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x621, 0, 8, data);
  byte sndStat2 = CAN0.sendMsgBuf(0x10242040, 1, 1, data2);
  if(sndStat == CAN_OK){
    Serial.println("Message Sent Successfully!");

  } else {
    Serial.println("Error Sending Message...");
    Serial.println(sndStat);   
  }

    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
