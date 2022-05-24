#include "Wire.h"
// ak4490 4118 4493等I2c总线控制测试 
//byte IC_addr = 0x23; 
byte IC_addr = 0x10;  //4118的地址,cad0/1都接地了,地址为7位，前5位固定为“00100”,后面两位由硬件引脚的电压决定：CAD1/CAD0
byte register_add = 0x00;  //寄存器地址，从0x00开始，不同的芯片，寄存器数量不同，ak4490为00~15，ak4118为00~28。每读取一次，内部计算器加1，只到最后一个寄存器地址再回到最开始。
byte ak4490 = 0x11;  

void vol_control(byte reg_addr,byte volume);
byte get_value(byte reg_addr);

void setup(){
  Serial.begin(115200); 
  Wire.begin();
  while(!Serial){} // Waiting for serial connection
  Serial.println("");
  for(byte i=0x00;i<=0x28;i++)
	  get_value(i);
  //PrintDebugReg();
  Serial.println("Input '+'  or '-' adj register");
}

void loop() {
//通过串口输入字符进行操作
  if(Serial.available()){
	char key = Serial.read();
    if(key == '+'){
		if(register_add < 0x28){
			register_add++;
		}
		else{
			register_add = 0x00;
		}
		get_value(register_add);
      }
	if(key == '-'){
		if(register_add > 0x00)
		{
			register_add--;
		}
		else{
			register_add = 0x28;
		}
		get_value(register_add);
	}
	}
}


byte get_value(byte reg_addr){
	//获取当前寄存器的值
	Wire.beginTransmission(IC_addr);  //开如传输数据
	Wire.write(reg_addr); //
	Wire.endTransmission();
	byte vol = 0;
	Wire.requestFrom(IC_addr,1);
	//delay(1);
	while(Wire.available()){
		vol = Wire.read();
			Serial.print("IC_addr: ");
			Serial.print(IC_addr,HEX);
			Serial.print(" reg_addr: ");
			Serial.print(reg_addr,HEX);
			Serial.print(" current value: ");
			Serial.println(vol,HEX);
	}
	return vol;
}
//读寄存器
byte ReadRegister(int devaddr, byte regaddr)                                // Read a data register value
  {                              
    Wire.beginTransmission(devaddr);
    Wire.write(regaddr);
    Wire.endTransmission();
    Wire.requestFrom(devaddr, 1);                 // only one byte
    if(Wire.available())                          // Wire.available indicates if data is available
      return Wire.read();                         // Wire.read() reads the data on the wire
    else
    return 9999;                                  // In no data in the wire, then return 9999 to indicate error
  }
//写寄存器
void WriteRegister(int devaddr, byte regaddr, byte dataval)                // Write a data register value
  {
    Wire.beginTransmission(devaddr); // device
    Wire.write(regaddr); // register
    Wire.write(dataval); // data
    Wire.endTransmission();
  }
  


