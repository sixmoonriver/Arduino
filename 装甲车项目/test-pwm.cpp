
#include <Arduino.h>
#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("9,10 ++");
  for(int i=100; i<255; i++){
    analogWrite(9,i);
    analogWrite(10,i);
    delay(200);

  }
  delay(2000);
  Serial.println("9,10 --");
  for(int i=255; i>100; i--){
    analogWrite(9,i);
    analogWrite(10,i);
    delay(200);
  }
  delay(2000);
}