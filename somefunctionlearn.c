#include <MotorDriver.h>


MotorDriver m;


void setup()
{

}


void loop()
{
  m.motor(1,FORWARD,255);
  m.motor(4,FORWARD,255);  

}

######################################################################
int iptpin = 4;
int optpwmpin = 5; 
int pgpin = 7;
unsigned long duration,pg;
int youmen1;
void setup()
{
  
  Serial.begin(115200);
  pinMode(iptpin, INPUT);
  pinMode(pgpin,INPUT);
  pinMode(optpwmpin , OUTPUT);
}

void loop()
{
//duration = pulseIn(iptpin,HIGH);
pg = pulseIn(pgpin,LOW);
//Serial.println(duration);
//youmen1 = map(duration,1002,1825,255,1);
Serial.print("pg =");
Serial.println(pg);
analogWrite(optpwmpin,250);
//delay(100);
//analogWrite(optpwmpin,5);
//delay(1000);
}
// 以上代码可以读取电机的转速，但是有个奇怪的问题，转速越高，读出来的值越小
#####################################################################






#############################################################################

volatile byte half_revolutions;
 unsigned int rpm;
 unsigned long timeold;
 void setup()
 {
	 
   Serial.begin(9600);
   attachInterrupt(0, rpm_fun, RISING);
   half_revolutions = 0;
   rpm = 0;
   timeold = 0;
 }
 void loop()
 {
   if (half_revolutions >= 20) { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     timeold = millis();
     half_revolutions = 0;
     Serial.println(rpm,DEC);
   }
 }
 void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
 }
 这段代码可以通过时间的方式，可以正确的读出转速
##############################################################################
 
 
 
 
 
 