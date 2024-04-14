#include <SoftwareSerial.h>
#include <Arduino.h>

SoftwareSerial S2(A2,A3);
int xpin = 1;
int ypin = 0;
int yValue,xValue,lastXval,lastYval = 0;


void setup(){
Serial.begin(115200);
S2.begin(9600);
}

void loop(){
xValue = analogRead(A0);
yValue = analogRead(A1);
if(abs(xValue -lastXval)>=20  or abs(yValue - lastYval)>=20){
    S2.write(map(xValue,0,1023,0,16)<<4 + map(lastYval,0,1023,0,16));
    lastXval = xValue;
    lastYval = yValue;
    Serial.print("X: ");
    Serial.print(xValue);
    Serial.print("Y: ");
    Serial.println(yValue);
}
} 