#include <WiFiUdp.h>
#include <NTPClient.h>                      
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <TM1650.h>
#include <TimeLib.h>    


#ifndef STASSID
#define STASSID "SYIOT"
#define STAPSK  "shangY00IT"
#endif

const char* ssid = STASSID;
const char* password = STAPSK; 
bool mhState = false;
long lastTime = 0;
 
WiFiUDP ntpUDP;
 
// serveraddress ，offset BJ 28800=8*60*60，updateinterval 
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 28800, 60000);
 
char Time[ ] = "0000";
char Date[ ] = "1231";
byte last_second, second_, minute_, hour_, day_, month_;
int year_;
 
TM1650 d; 
 
void setup() {
  Wire.begin(); //Join the bus as master 
  Serial.begin(115200);
  d.init();
  WiFi.begin(ssid, password);
  Serial.print("Connecting.");
 
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }
  d.displayOff();
  d.displayString("____");
  d.setBrightness(TM1650_MIN_BRIGHT);
  d.displayOn();  
}
  
void loop() {

  timeClient.update(); //这个更新60秒才执行一次，库里会判断上次更新的间隔
  unsigned long unix_epoch = timeClient.getEpochTime();    // Get Unix epoch time from the NTP server
  
  // 如果在晚上(晚8点以后，早7点以前)，亮度调低
  if(hour(unix_epoch) > 20 and hour(unix_epoch) < 7){
	d.setBrightness(TM1650_MIN_BRIGHT);
  }
  else{
	d.setBrightness(TM1650_MAX_BRIGHT);  
  }
  //hour_   = hour(unix_epoch);
  // 如果距离上次超过1秒，闪烁
  if ( second(unix_epoch) != lastTime  ) {
		d.displayString(Time);
		mhState = !mhState;
//		for(int i=0;i<4;i++){
			d.setDot(1,mhState);
			//delay(1000);
	//	}
    //Serial.print(hour(unix_epoch));
	//Serial.print(":");
	//Serial.println(minute(unix_epoch));
//计算时间并显示
    minute_ = minute(unix_epoch);
    hour_   = hour(unix_epoch);
    day_    = day(unix_epoch);
    month_  = month(unix_epoch);
    //year_   = year(unix_epoch);
 
    //Time[12] = second_ % 10 + 48;
    //Time[11] = second_ / 10 + 48;
    Time[3]  = minute_ % 10 + 48;
    Time[2]  = minute_ / 10 + 48;
    Time[1]  = hour_   % 10 + 48;
    Time[0]  = hour_   / 10 + 48;
  
    Date[0]  = day_   / 10 + 48;
    Date[1]  = day_   % 10 + 48;
    Date[2]  = month_  / 10 + 48;
    Date[3]  = month_  % 10 + 48;
    //Date[13] = (year_   / 10) % 10 + 48;
    //Date[14] = year_   % 10 % 10 + 48;
    Serial.print(Time);
	Serial.println(lastTime);
	lastTime = second(unix_epoch); 
  }

  
}