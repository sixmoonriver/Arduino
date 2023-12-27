
/*
船模制作：

1、用NR24L01实现遥控功能，一个arduino做遥控器，一个做为接收器。用一个摇杆控制油门和方向；
2、用舵机控制方向；
3、用电调无刷电机控制速度； 
4、用一个舵机控制摄像头的方向；
5、灯的控制
*/

// 遥控器部分代码
#include <Arduino.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
uint8_t convert[5]={0};
uint8_t cam_valueX,cam_valueY,camValue= 0;
uint16_t xSourceValue,ySourceValue,camSourceValueX,camSourceValueY=0;
uint8_t x_value,y_value,last_xvalue,last_yvalue,con_value,last_convalue,last_camValue=0;
uint32_t trans_value, temp_value= 0;
union transData
{
   long newvalue;
   byte buffer[4];
};

transData thisUnion;
//Serial.println(thisUnion.newvalue, HEX);

int xpin = 0;  // 方向输入脚模拟A0
int ypin = 1;  // 油门输入脚模拟A1
int camControlX = 2; // 方向输入脚模拟A2
int camControlY = 3; // 方向输入脚模拟A3
//开关引脚定义
int switchPin1 = 6;
int switchPin2 = 7;
int switchPin3 = 8;
int switchPin4 = 9;

//开关状态
int pin1Reading,pin2Reading,pin3Reading,pin4Reading = 0;
int switchState1,switchState2,switchState3,switchState4 = 0;
int lastSwitchState1,lastSwitchState2,lastSwitchState3,lastSwitchState4 = 0;
//去抖动时间
long interval,lastTime1,lastTime2,lastTime3,lastTime4=0;
void setup()
{
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"SENDE"); //设置自己的地址（发送端地址），使用5个字符
  Mirf.payload = 4;     //设置传送位数，每通道使用8位；低8位是油门、次8位是方向，再高位是功能开关。
  Mirf.channel = 90;              //设置所用信道
  Mirf.config();
  Serial.begin(115200);
     // Read and print RF_SETUP 用于检测是否初始化正常；
  byte rf_setup = 0;
  Mirf.readRegister( RF_SETUP, &rf_setup, sizeof(rf_setup) );
  Serial.print( "rf_setup = " );
  Serial.println( rf_setup, BIN );
}
void loop()
{
  Mirf.setTADDR((byte *)"RECVE");           //设置接收端地址
  //读取摇杆的数值
  xSourceValue = analogRead(xpin);  //读取摇杆输入的数值
  x_value = map(xSourceValue,0,1023,0,255); //将模拟量的10位值转为8位值；
  ySourceValue = analogRead(ypin);
  y_value = map(ySourceValue,0,1023,0,255);
  //value = random(255); 
  //读取摄像头方位角度控制值
  camSourceValueX = analogRead(camControlX);
  cam_valueX = map(camSourceValueX,0,1023,0,15);
  //读取摄像头俯仰角度控制值
  camSourceValueY = analogRead(camControlY);
  cam_valueY = map(camSourceValueY,0,1023,0,15);
  camValue = (cam_valueY << 4) + cam_valueX;
  /* 读取开关1状态，如果状态发生变化，记录时间
  pin1Reading = digitalRead(switchPin1);
  if( lastSwitchState1 != switchState1) {
    lastTime1 = millis();
  }
  // 如果当时时间到上一次开关状态变化的时间差大于防抖动的时间，再判断当前的状态和上次的状态是否一致，如不一致，说明开关状态发生了变化。将开关状态
  if(millis() - lastTime1 >= interval) {
    if(pin1Reading != lastSwitchState1) {
      switchState1 = pin1Reading;
    }
  }
  // 控制数据处理
  // 最高4位为开关状态，最高位为开关1的状态，以此类推6、5、4为开关2、3、4
  //bitWrite(con_value,7,switchState1);
  lastSwitchState1 = pin1Reading;
  //开关2~4位处理；
  //控制数据组装
  con_value =  cam_valueX;
  */
  con_value = 0xCC; //11001100BIN
  if(x_value != last_xvalue | y_value != last_yvalue | con_value != last_convalue | camValue != last_camValue)
  {
    //发送的数据组装
	//trans_value = 0;
  thisUnion.buffer[0] = y_value;
  thisUnion.buffer[1] = x_value;
  thisUnion.buffer[2] = camValue;
  thisUnion.buffer[3] = con_value; 
  //trans_value = y_value | (x_value<<8) | (camValue<<16) | (con_value<<24);
  //trans_value = (con_value<<24) | trans_value;
  //Serial.println(trans_value,BIN);
  //trans_value = (camValue<<16) |  trans_value; 
  //Serial.println(trans_value,BIN);
  //trans_value = (x_value<<8) |  trans_value;
  //Serial.println(trans_value,BIN);
  //trans_value = trans_value | y_value;
  //Serial.println(trans_value,BIN);
	/*if(con_value > 0){
	  trans_value = con_value + trans_value;
		trans_value = trans_value<<24; //控制数据左移24位，到25~32位；	
	}
  if(camValue > 0 ){
    temp_value = 0;
    temp_value = (temp_value + camValue) << 16; //cam数据左移16位，到16~24位；
    trans_value = temp_value + trans_value;
  } 
	if(x_value > 0){
		temp_value = 0;
		temp_value = (temp_value + x_value) << 8;
		trans_value =  temp_value + trans_value; //方向数据左移8位，8~16位；
	}
    if(y_value > 0){
		trans_value = trans_value + y_value; //油门数据组装
	}
 */
	Mirf.send((byte *)&thisUnion.newvalue);                //发送指令，组合后的数据，低8位为油门数据，高8位为方向数据
    while(Mirf.isSending()) delay(1);          //直到发送成功，退出循环
    last_xvalue = x_value;   //last数据更新
    last_yvalue = y_value;
    last_camValue = camValue;
    last_convalue = con_value;
    Serial.print("x_value: ");
    Serial.println(x_value);
    Serial.print("y_value: ");
    Serial.println(y_value);
	  Serial.print("cam_value: ");
    Serial.println(camValue);
    Serial.print("con_value: ");
    Serial.println(con_value,BIN);
    Serial.print("trans_value: ");
    Serial.println(thisUnion.newvalue,BIN);
  }
    delay(100);
}