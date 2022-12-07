// 
#include "Wire.h"
//#include <LCD_I2C.h>
#include <IRremote.h>
#include<stdlib.h>
/*
pioneer F-Z93 radio+arduino+IRremote
IR remoter is skyworth YK-60HB
 2021.3 */
/* 
功能设计：
  1、实现数字选台，预存一部分本地电台，按数字键即可选择相应的电台（暂不考虑AM）；
  2、实现手动调谐，基于当前的电台频率，按步长进行选择调谐，可以向上也可以向下；
电路构成：
  1、pioneer Z93组合音响收音头主板；
  2、arduino(UNO/promini);
  3、1602显示屏配I2c模块（PCF8574）；
  4、遥控系统采用创维YK-60HB；
  5、普通红外接收头；
*/

/*
	2022.9.17更新
	修复了开机强制设置为单声道的问题；
	增加了上拉电阻和到GPIO的引脚电阻，解决了电台误别和立体声状态显示不正常的问题； 
	在电台首行简易显示是否收到电台和是否为立体声的问题；有电台首行首位显示1，否则显示0；立体声在尾部显示ST，否则显示MO
	更换LA1265的中周（第一音响拆机），槽路电路还是有些发黑，飞扬971出了立体声。
*/
/*
	2022.11.18支持原装VFD显示屏
	1、整体思路：
	调频模式：
		1、9~5G为频率显示，9G对应百位，8G对应十位，7G对应个位，6G对应小数点和-1位，5G对应-2位。
		2、4~1G为时间显示。
		3、10G为单独的功能显示

*/
//#################以下为收音相关设置###########################。
//设置电台刷新时间为1000ms
long interval = 1000;
//设置操作后停留的时间为5秒
long sleeptime = 5000;
//各栅极之间的刷新间隔，防止闪烁。
long freshtime = 2; 
long lastoper = 0;
long last = 0;
//LCD_I2C lcd(0x27); //创建lcd显示对象，0x27为默认地址，可以通过示例程序I2c-scanner来扫描是否已经连接或者地址
int RECV_PIN = 4; // 红外一体化接收头连接到Arduino 7号引脚


const int mono = 2;  //强制单声道切换
const int Mute = 3; 
const int PLL_data = 7;
const int PLL_clock = 5 ;
const int PLL_CE = 6 ;
const int tune = 8;  // 调谐状态指示，收到电台为低电平；
const int stled = 9; // 立体声指示，低电平为音体声状态
int monostate = 0; //强制单声道状态，默认为关，如打开，立体声状态不起作用。
int muteState = 0;
int recevState = 0; //是否收到电台
int stereoState = 0; //是否立体声
int disIndex = 0; //循环显示的频道索引
int curIndex = 0; //当前频道索引
int curfreq = 875; //当前频道频率
int band = 0;//0为FM，1为AM；
int FMstepSize = 100; //单位为KHz
int AMstepSize = 50; //单位为KHz
unsigned int  Config = 0x83 ;//1000 0011 代表FM模式：BO1高，BO2、BO3低，步长为100KHz，参考LM7001手册和具体电路，BO1、BO2、BO3由电路决定，
unsigned int  D_high;
unsigned int  D_low;
long tm;

int radiolist[]{
  1071,898,905,971,1062,879,1076,907,918,939,930,943,958,988,991,996,1000,1012,1043,1075};//单位0.1MHz

IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象

//#################以下为VFD控制部分设置###################

//显示屏笔画控制扩展器
byte vfdAddrLow = 0x20;
byte vfdAddrHigh = 0x21;
//显示控制栅极扩展
byte gAddrLow = 0x22;
byte gAddrHigh = 0x23;
// 按键中断输入
int bottonInt1 = 10;
int bottonInt2 = 11;
// 按键控制扩展器
byte key_row = 0x26;
byte key_col = 0x27;


// 数码管显示
byte duan7[] = {0x3F,0x6,0x5B,0x4F,0x66,0x6D,0x7D,0x7,0x7F,0x6F};
byte duan14[] = {0x3F,0x6,0x5B,0xCF,0xE6,0xED,0xFD,0x7,0xFF,0xEF};
// 栅极地址（1G接0位，10G接9位，为了让索引和栅极保持一致，0设为全开，不使用）
word gAddr[] = {0xFFFF,0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,0x0100,0x0200}; 


byte get_value(byte ic_addr){
  Wire.beginTransmission(ic_addr);  //开如传输数据
  Wire.requestFrom(ic_addr,1);
  //delay(1);
  while(Wire.available()){
    uint8_t val = Wire.read();
    return val;
  }
}

byte set_value(byte ic_addr, uint8_t num1){
  //获取当前寄存器的值
  Wire.beginTransmission(ic_addr);  //开如传输数据
  Wire.write(num1); //
  Wire.endTransmission();
  //delay(10);
}

//消隐函数
void xiaoyin(){
	set_value(vfdAddrLow,0xFF);
	set_value(vfdAddrHigh,0xFF);
	set_value(gAddrLow,0xFF);
	set_value(gAddrHigh,0xFF);
}
// 10G print  TnSt 调谐状态，Mo单声道，TV不用，band 0为FM，1为AM；
void view_10g(int TnSt, int Mo, int TV=0, int band=0){
	//根据状态设置各位的值
	byte value_10 = 0x00;
	if (TnSt) bitSet(value_10,0);
	if (Mo) bitSet(value_10,1);
	if (TV) bitSet(value_10,2);
	 band ? bitSet(value_10,4): bitSet(value_10,3);
	//控制栅极导通，因为uln2803是反向输出，需要取反操作。
	set_value(gAddrHigh, ~(highByte(gAddr[10])));
	set_value(gAddrLow, ~(lowByte(gAddr[10])));
	set_value(vfdAddrLow, ~value_10);
}
// 6~9G print 9G为频率的百位，8G为频率的十位，7G为频率的个位，6G为小数点＋小数点后１位，5G为小数点后2位＋单位。
// index为6~9中的某个，State对应S位的状态，即Local(6G),SUB(7G),MAIN(8G),STEREO(9G)的状态。num为要入的数字。 dotState小数点，6、7G才有。
void view_6_9g(int index, int state, byte num2, int dotState=0){
	byte value_6_9 = 0x00;
	if(dotState) bitSet(value_6_9, 7);
	if(state) bitSet(value_6_9, 6);
	//控制栅极导通
	set_value(gAddrHigh, ~(highByte(gAddr[index])));
	set_value(gAddrLow, ~(lowByte(gAddr[index])));
	//显示数字和S位的状态
	set_value(vfdAddrLow, ~num2);
	set_value(vfdAddrHigh, ~value_6_9);
}

// 5G print
// Mhz单位是否是Mhz, num3为要显示的数字；
void view_5g(int Mhz, byte num3 ){
	byte value_5 = 0x00;
	if(Mhz) bitSet(value_5, 6);
	else bitSet(value_5, 7);
	//控制栅极导通
	set_value(gAddrHigh, ~(highByte(gAddr[5])));
	set_value(gAddrLow, ~(lowByte(gAddr[5])));
	//显示数字和S位的状态
	set_value(vfdAddrLow, ~num3);
	set_value(vfdAddrHigh, ~value_5);	
}


// 4_1G通用函数，需要 print //输出,功能显示参数可以不传，默认全部不显示 index 为哪个G，1G为1，2G为2，4G为4；
void view_4_1g(int index, byte num4, byte func4=0x00){
	//控制栅极导通
	set_value(gAddrHigh, ~highByte(gAddr[index]));
	set_value(gAddrLow, ~lowByte(gAddr[index]));
	//显示数字和功能设置，
	set_value(vfdAddrLow, ~func4);
	set_value(vfdAddrHigh, ~num4);		
}


//静态信息显示函数，在没有操作的时候显示电台频率、时间要不要显示？
void staticDisplay()
{
  //如果超过刷新间隔时间，进行一次刷新，
  //if (millis()-last >= interval){
  if (1){	  
    //显示10G
    view_10g(recevState, monostate);
    delay (freshtime);//ms
	//关断
	xiaoyin();
    //显示9G 立体声 百位数字, 首位如果为0，不显示。
    int hundredBit = 0x00; //数字值
    if (curfreq > 999) {
        hundredBit = duan14[(curfreq/1000)%10];
    }
    view_6_9g(9, stereoState, hundredBit);
    delay (freshtime);//ms
	//关断
	xiaoyin();	
    //显示8G 状态位用不上不显示，十位数字
    view_6_9g(8, 0, duan14[(curfreq/100)%10]);
    delay (freshtime);//ms
	//关断
	xiaoyin();
	//显示7G; 状态位用不上不显示，个位数。
    view_6_9g(7, 0, duan14[(curfreq/10)%10]);
    delay (freshtime);//ms
	//关断
	xiaoyin();
    //显示6G 状态位用不上不显示，最后一位，小数点
    view_6_9g(6, 0, duan14[curfreq%10], 1);
    delay (freshtime);//ms
	//关断
	xiaoyin();
    //显示5G 只显示Mhz(S) ,数字不显示
    view_5g(!band, duan14[0]);
    delay (freshtime);//ms
	//关断
	xiaoyin();
    }
}

//设置电台频率（AM收不到台，暂时只支持FM，如需要AM功能，带band参数1，config参数需要根据电路设置）
void pll_set_frequency(long pllfreq,int band=0,int config=Config) { 
  if (band == 0) {
    D_high=highByte((pllfreq*100+107*100)/FMstepSize);
    D_low=lowByte((pllfreq*100+107*100)/FMstepSize);
  }

  if (band == 1) {
    D_high=highByte(pllfreq+450)/AMstepSize;
    D_low=lowByte(pllfreq+450)/AMstepSize;
  }
  
  digitalWrite(PLL_CE, HIGH);
  digitalWrite(Mute, HIGH);
  //digitalWrite(PLL_clock,LOW);
  //tm = micros();
  shiftOut(PLL_data, PLL_clock, LSBFIRST, D_low);
  //tm = micros()-tm;
  shiftOut(PLL_data, PLL_clock, LSBFIRST, D_high);
  shiftOut(PLL_data, PLL_clock, LSBFIRST, config);
  digitalWrite(PLL_CE, LOW);
  //Serial.println(tm);
  delay(800);
  digitalWrite(Mute, LOW);
  lastoper = millis();
}
//设置电台频率函数（没用shiftOut实现，自己根据时序图写的）
void pll_set_frequency2(long pllfreq) { 
  if (band == 0) {
    D_high=highByte((pllfreq*100+107*100)/FMstepSize);
    D_low=lowByte((pllfreq*100+107*100)/FMstepSize);
  }

  if (band == 1) {
    D_high=highByte(pllfreq+450)/AMstepSize;
    D_low=lowByte(pllfreq+450)/AMstepSize;
  }

  digitalWrite(PLL_CE, HIGH);
  
  for(int i=0;i<8;i++){
    int Data=(D_low>>i) & 0x1;
    digitalWrite(PLL_clock,LOW);
    digitalWrite(PLL_data,Data);
    delayMicroseconds(5);
    digitalWrite(PLL_clock,HIGH);
    delayMicroseconds(5);
   // Serial.print(Data);
  }

  for(int i=0;i<8;i++){
    int Data=(D_high>>i) & 0x1;
    digitalWrite(PLL_clock,LOW);
    digitalWrite(PLL_data,Data);
    delayMicroseconds(5);
    digitalWrite(PLL_clock,HIGH);
    delayMicroseconds(5);
 //   Serial.print(Data);
  }
  for(int i=0;i<8;i++){
    int Data=(Config>>i) & 0x1;
    digitalWrite(PLL_clock,LOW);
    digitalWrite(PLL_data,Data);
    delayMicroseconds(5);
    digitalWrite(PLL_clock,HIGH);
    delayMicroseconds(5);
    //Serial.print(Data);
  }
  delayMicroseconds(5);
  digitalWrite(PLL_CE, LOW);
  delay(200);
  Serial.println("#######");
}

void setup(){
	Serial.begin(115200);
    irrecv.enableIRIn(); // 初始化红外解码
    //初始化收音机控制引脚
    pinMode(PLL_data, OUTPUT);//!
    pinMode(PLL_clock, OUTPUT);
    pinMode(PLL_CE, OUTPUT);
    pinMode(Mute, OUTPUT);
    pinMode(mono, OUTPUT);
    pinMode(tune, INPUT);
    pinMode(stled, INPUT);
    digitalWrite(mono,monostate);
    pll_set_frequency(radiolist[3]);
    delay (30);//ms

	Wire.begin(vfdAddrLow);
	Wire.begin(vfdAddrHigh);
	Wire.begin(gAddrLow);
	Wire.begin(gAddrHigh);
	//初始化全屏点亮
	set_value(vfdAddrLow,0x00);
	set_value(vfdAddrHigh,0x00);
	set_value(gAddrLow,0x00); //数码管为共阴，低电平，VFD为0x00;
	set_value(gAddrHigh,0x00);
	delay(5000);
	set_value(vfdAddrLow,0xff);
	set_value(vfdAddrHigh,0xff);
	set_value(gAddrLow,0xFF);//数码管为共阴，低电平，VFD为0xFF;
	set_value(gAddrHigh,0xFF);
	pinMode(bottonInt1,INPUT);
	pinMode(bottonInt1,INPUT);
}


void loop(){

 if (irrecv.decode(&results))
{
  char chanel[10];
  switch(results.value){
    //按键1~0,读取频道列表数组对应的频率，调用PLL函数，设置频率，并且在屏幕上显示频道号和频率，重置lastoper；
    case 1886421119: curIndex = 0; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886404799: curIndex = 1; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886437439: curIndex = 2; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886396639: curIndex = 3; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886429279: curIndex = 4; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886412959: curIndex = 5; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886445599: curIndex = 6; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886392559: curIndex = 7; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886425199: curIndex = 8; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    case 1886388479: curIndex = 9; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);staticDisplay();lastoper = millis();break;
    //按键频道+-，判断有没有达到频道列表上、下限，如果达到，就重置为最大或最小。获取当前频道的频率，设置频率，串口和显示屏输出,重置lastoper；
    case 1886406839: if(curIndex >= sizeof(radiolist)/sizeof(radiolist[0])) curIndex=0; curfreq = radiolist[++curIndex];pll_set_frequency(curfreq);Serial.print("+>>");Serial.println(curIndex,DEC);sprintf(chanel,"Chanel %d",curIndex);staticDisplay();lastoper = millis();break;
    case 1886439479: if(curIndex <= 0) curIndex=sizeof(radiolist)/sizeof(radiolist[0]); curfreq = radiolist[--curIndex];pll_set_frequency(curfreq);Serial.print("->>");Serial.println(curIndex,DEC);sprintf(chanel,"Chanel %d",curIndex);staticDisplay();lastoper = millis();break;
    //搜索上、下,判断频率有没有超过FM的上、下限，如果达到，重置为最大或者最小。获取当前的频道的频率并增加1（FM步长），设置频率，串口和显示屏输出，重置lastoper；
    case 1886405309: if(curfreq >= 1080) curfreq =  875; curfreq = ++radiolist[curIndex]; pll_set_frequency(curfreq);Serial.print(">>>up");Serial.println(curfreq,DEC);staticDisplay();lastoper = millis();break;
    case 1886437949: if(curfreq <= 875) curfreq =  1080; curfreq = --radiolist[curIndex]; pll_set_frequency(curfreq);Serial.print(">>>down");Serial.println(curfreq,DEC);staticDisplay();lastoper = millis();break;
   //静音控制，收到后对静音状态进行翻转，将静音状态写入静音控制脚，即按一下静音，再按一下解除。串口和显示屏输出，重置lastoper；
    case 1886433359: muteState = !muteState;digitalWrite(Mute,muteState);Serial.println("Mute");staticDisplay();lastoper = millis();break;
  //强制单声道切换，收到后设置为单声道状态，再按一下解除，同时修改状态；
    case 1886394599: 
  {
    monostate = !monostate;  
    digitalWrite(mono,monostate);
    if(monostate){Serial.println("Mono");staticDisplay();lastoper = millis();}
    else{Serial.println("NoMono");staticDisplay();lastoper = millis();}
    break;
  }
   //case 1886437439: pll_set_frequency(radiolist[2]);Serial.println("3");break;
    //default: pll_set_frequency2(radiolist[0]);
  }
irrecv.resume(); // 接收下一个编码
}
else {
  //如无操作,刷新一次状态 
  if(millis()-lastoper >= sleeptime){
        lastoper = millis();
        //Serial.println("staticDisplay");
 	 //是否收到电台,设置recevState
	 if(digitalRead(tune))
	 {
		recevState = 1;
	 } 
	 else
	 {
		recevState = 0; 
	 }
	//如果是强制单声道模式，立体声状态置0，如果不，读取立体声状态
	if(monostate){
		stereoState = 0;
	}
	else {
	//判断是否立体声
	 if(digitalRead(stled))
	 {
		stereoState = 1;
	 }
	 else{
		stereoState = 0;
	 }
	}
  }
  staticDisplay();
}

}

/*
创维1~0号按键
1886421119 
1886404799 
1886437439 
1886396639 
1886429279 
1886412959 
1886445599 
1886392559 
1886425199 
1886388479 

返回  1886444069
快捷键 1886423669
频道+ 1886406839
频道- 1886439479
静音  1886433359
音量+ 1886398679
音量- 1886431319
上 1886405309
下 1886437949
声音模式（立体声）：1886394599
*/