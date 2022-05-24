#include "Wire.h"
// stac9460 I2c总线控制  
// 使用环境：创新 SB0490声卡
byte IC_addr = 0x2A; //通过I2C扫描程序扫描到的地址
byte master_vol = 0x28; //音量00为最大值，数字越大，音量越小
byte audio_format_reg = 0x0E; 
//byte cur_volume = 0;
void vol_control(byte reg_addr,byte volume);
byte get_value(byte reg_addr);

void setup(){
  Serial.begin(115200); 
  while(!Serial){} // Waiting for serial connection
  Wire.begin();
  Wire.beginTransmission(IC_addr); //开始传输数据，I2C地址为0x2A;
  Wire.write(audio_format_reg); //移动指针到audio data format寄存器位置
  Wire.write(0x00); //修改音频数据格式为16 bit right justified；00010
  Wire.write(0x04);//写入主时钟设置,SR<48Khz 00,MCLK mode 100。因为音频数据格式寄存器的下一个就是时钟设置寄存器，这里利用上一步写完后指针自动后移一步，省略了寻址部分。
  //Serial.println("");
  delay(1);
  Wire.endTransmission();
  Wire.beginTransmission(IC_addr); //开始传输数据，I2C地址为0x2A;
  Wire.write(0x02); //移动指针到主音量的寄存器位置
  Wire.write(master_vol); //修改主音量的值为16进制的28，10进制的40；
  //Serial.println("");
  delay(1);
  Wire.endTransmission();
  Serial.println("");
  Serial.println("Input '+' or '-' adj volume");
}

void loop() {
  if(Serial.available()){
	char key = Serial.read();
    if(key == '+'){
      if(master_vol <= 100){
		master_vol += 5;
        vol_control(0x02,master_vol);
      }
	}
	if(key == '-'){
		if(master_vol >= 0){
			master_vol -= 5;
			vol_control(0x02,master_vol);
		}     
	  }
  }
}

void vol_control(byte reg_addr,byte volume){
  Wire.beginTransmission(IC_addr); //开始传输数据，I2C地址为0x2A;
  Wire.write(reg_addr); //移动指针到主音量的寄存器位置
  Wire.write(volume); //修改主音量的值为16进制的28，10进制的40；
  //Serial.println("");
  delay(1);
  Wire.endTransmission();
  byte cur_volume = 0;
  cur_volume = get_value(reg_addr);
  if( cur_volume == volume){
	Serial.println(" Modify OK!");  
  }
  else{
	Serial.println(" Modify failue!");
	Serial.print(volume,HEX);
	Serial.println(" but:  ");
	Serial.print(cur_volume,HEX);
  }
  //if(Wire.endTransmission() == 0) Serial.println(" Modify OK!");
}

byte get_value(byte reg_addr){
	//获取当前寄存器的值
	Wire.beginTransmission(IC_addr);
    Wire.write(reg_addr);
	Wire.endTransmission();
	Wire.requestFrom(IC_addr,1);
	//delay(1);
	byte vol = 0;
	while(Wire.available()){
		vol = Wire.read();
		Serial.print(vol,HEX);
	}

	return vol;
}