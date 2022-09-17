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


#include <LCD_I2C.h>
#include <IRremote.h>
#include<stdlib.h>


//设置电台刷新时间为1000ms
long interval = 1000;
//设置操作后停留的时间为5秒
long sleeptime = 5000;
//保存上次操作和刷新的时间，上次操作的时间用于启动静态频率显示
long lastoper = 0;
long last = 0;
LCD_I2C lcd(0x27); //创建lcd显示对象，0x27为默认地址，可以通过示例程序I2c-scanner来扫描是否已经连接或者地址
int RECV_PIN = 7; // 红外一体化接收头连接到Arduino 7号引脚
IRrecv irrecv(RECV_PIN);
decode_results results; // 用于存储编码结果的对象

const int mono = 2;  //强制单声道切换
const int Mute = 3; 
const int PLL_data = 4;
const int PLL_clock = 5 ;
const int PLL_CE = 6 ;
const int tune = 8;  // 调谐状态指示，收到电台为低电平；
const int stled = 9; // 立体声指示，低电平为音体声状态
int muteState = 0;
int stereoState = 0; //默认立体声
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

long radiolist[]{
  1071,898,905,971,1062,879,1076,907,918,939,930,943,958,988,991,996,1000,1012,1043,1075};//单位0.1MHz

//静态信息显示函数，在没有操作的时候显示电台列表，便于查看换台
void staticDisplay()
{
  //如果超过刷新间隔时间，进行一次刷新，
  if (millis()-last >= interval){
     lcd.clear();
     lcd.setCursor(0,0);
	 //是否收到电台,在开始显示0或者1
	 if(digitalRead(tune))
	 {
		lcd.print("0");
	 }
	 else{
		lcd.print("1");
	 }
     lcd.print(F(" LY-Radio:"));
     lcd.print(curIndex);
	 //判断是否立体声，是否收到电台
	 if(digitalRead(stled))
	 {
		lcd.print(" MO");
	 }
	 else{
		lcd.print(" ST");
	 }
     //如果电台遍历完成，重置电台索引
     if(disIndex >=  sizeof(radiolist)/sizeof(radiolist[0]))
     {
         disIndex = 0;
     }
     //显示电台列表
     lcd.setCursor(0,1);
     lcd.print(disIndex);
     lcd.print(":");
     lcd.print(radiolist[disIndex++]);
     //显示第2个电台列表
     lcd.setCursor(8,1);
     lcd.print(disIndex);
     lcd.print(":");
     lcd.print(radiolist[disIndex++]);
     last = millis();
    }
}

//操作信息显示函数，显示操作的动作、频道和频率
void operDisplay(char *cmd,int freq)
{
  //刷新显示标题
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print(F("LY-Radio: "));
    lcd.print(curIndex);
    //显示电台列表
    lcd.setCursor(0,1);
    lcd.print(cmd);
    lcd.print(":");
    lcd.print(freq);
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
  //初始化控制引脚
  pinMode(PLL_data, OUTPUT);//!
  pinMode(PLL_clock, OUTPUT);
  pinMode(PLL_CE, OUTPUT);
  pinMode(Mute, OUTPUT);
  pinMode(mono, OUTPUT);
  pinMode(tune, INPUT);
  pinMode(stled, INPUT);
  digitalWrite(mono,stereoState);
 // pll_set_frequency(radiolist[0]);
  delay (30);//ms
  lcd.begin();
  lcd.backlight();
}


void loop(){

 
 if (irrecv.decode(&results))
{
  char chanel[10];
  switch(results.value){
    //按键1~0,读取频道列表数组对应的频率，调用PLL函数，设置频率，并且在屏幕上显示频道号和频率，重置lastoper；
    case 1886421119: curIndex = 0; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 0 ",curfreq);lastoper = millis();break;
    case 1886404799: curIndex = 1; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 1 ",curfreq);lastoper = millis();break;
    case 1886437439: curIndex = 2; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 2 ",curfreq);lastoper = millis();break;
    case 1886396639: curIndex = 3; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 3 ",curfreq);lastoper = millis();break;
    case 1886429279: curIndex = 4; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 4 ",curfreq);lastoper = millis();break;
    case 1886412959: curIndex = 5; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 5 ",curfreq);lastoper = millis();break;
    case 1886445599: curIndex = 6; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 6 ",curfreq);lastoper = millis();break;
    case 1886392559: curIndex = 7; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 7 ",curfreq);lastoper = millis();break;
    case 1886425199: curIndex = 8; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 8 ",curfreq);lastoper = millis();break;
    case 1886388479: curIndex = 9; curfreq = radiolist[curIndex];pll_set_frequency(curfreq);Serial.println(curIndex+1);operDisplay("Chanel 9 ",curfreq);lastoper = millis();break;
    //按键频道+-，判断有没有达到频道列表上、下限，如果达到，就重置为最大或最小。获取当前频道的频率，设置频率，串口和显示屏输出,重置lastoper；
    case 1886406839: if(curIndex >= sizeof(radiolist)/sizeof(radiolist[0])) curIndex=0; curfreq = radiolist[++curIndex];pll_set_frequency(curfreq);Serial.print("+>>");Serial.println(curIndex,DEC);sprintf(chanel,"Chanel %d",curIndex);operDisplay(chanel,curfreq);lastoper = millis();break;
    case 1886439479: if(curIndex <= 0) curIndex=sizeof(radiolist)/sizeof(radiolist[0]); curfreq = radiolist[--curIndex];pll_set_frequency(curfreq);Serial.print("->>");Serial.println(curIndex,DEC);sprintf(chanel,"Chanel %d",curIndex);operDisplay(chanel,curfreq);lastoper = millis();break;
    //搜索上、下,判断频率有没有超过FM的上、下限，如果达到，重置为最大或者最小。获取当前的频道的频率并增加1（FM步长），设置频率，串口和显示屏输出，重置lastoper；
    case 1886405309: if(curfreq >= 1080) curfreq =  875; curfreq = ++radiolist[curIndex]; pll_set_frequency(curfreq);Serial.print(">>>up");Serial.println(curfreq,DEC);operDisplay(">>>up",curfreq);lastoper = millis();break;
    case 1886437949: if(curfreq <= 875) curfreq =  1080; curfreq = --radiolist[curIndex]; pll_set_frequency(curfreq);Serial.print(">>>down");Serial.println(curfreq,DEC);operDisplay(">>>down",curfreq);lastoper = millis();break;
   //静音控制，收到后对静音状态进行翻转，将静音状态写入静音控制脚，即按一下静音，再按一下解除。串口和显示屏输出，重置lastoper；
    case 1886433359: muteState = !muteState;digitalWrite(Mute,muteState);Serial.println("Mute");operDisplay("Mute",curfreq);lastoper = millis();break;
  //立体声控制，收到后对静音状态进行翻转，将静音状态写入静音控制脚，即按一下静音，再按一下解除。串口和显示屏输出，重置lastoper；
    case 1886394599: 
  {
    stereoState = !stereoState;
    digitalWrite(mono,stereoState);
    if(stereoState){Serial.println("Stereo");operDisplay("Stereo",curfreq);lastoper = millis();}
    else{Serial.println("Mono");operDisplay("Mono",curfreq);lastoper = millis();}
    break;
  }
   //case 1886437439: pll_set_frequency(radiolist[2]);Serial.println("3");break;
    //default: pll_set_frequency2(radiolist[0]);
  }
irrecv.resume(); // 接收下一个编码
}
else {
  //如无操作，滚动显示电台列表和频率，首行显示当前频道
  if(millis()-lastoper >= sleeptime){
        lastoper = millis();
        //Serial.println("staticDisplay");
    staticDisplay();
  }
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