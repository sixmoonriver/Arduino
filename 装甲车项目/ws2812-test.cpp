#include <Adafruit_NeoPixel.h>

#define PIN 4

Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);
int inputX = A1;
int inputY = A0;
int swPin = 7;
int lastXvalue, xValue = 0;
int ledNumber = 6;
uint32_t color = strip.Color(0,128,0);;
void setup() {
  // put your setup code here, to run once:
    pinMode(inputY, INPUT);
    pinMode(inputX, INPUT);
    pinMode(swPin, INPUT_PULLUP);
    Serial.begin(115200);
    strip.begin();
    strip.show();
    Serial.println("INPUT '+' or '-'");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.print("X = ");
  // Serial.println(analogRead(inputX));
  // Serial.print("Y = ");
  // Serial.println(analogRead(inputY));
  // Serial.print("swState: ");
  // Serial.println(digitalRead(swPin));  
  if(Serial.available()){
    char command = Serial.read();
    switch (command)
    {
      case '+' :  ledNumber++; strip.clear(); break;
      case '-' :  ledNumber--; strip.clear(); break;  
      default: ; 
    break;
  }

  }
  // xValue = analogRead(inputX);
  // if(xValue != lastXvalue){
  //   if(xValue >= 600){
  //     strip.clear();
  //     color = strip.Color(0,128,0);
  //     //ledon(map(xValue, 0, 1024, 0, 12),'R');
  //   }
  //   else
  //   {
  //     //ledon(map(xValue, 0, 1024, 0, 12),'G');
  //     strip.clear();
  //     color = strip.Color(128,0,0);
  //     //colorWipe(strip.Color(0,255,0),map(xValue,0,1024,0,12));
  //   }
  //   lastXvalue = xValue;
  // }
  //color = strip.Color(0,128,0);
  colorWipe(color, ledNumber);
  delay(20);
}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t ledcount) {
  for(uint16_t i=0; i<ledcount; i++) {
    strip.setPixelColor(i, c);
    strip.show(); //没有这一行，全是黑的；
    //delay(10);
  }
}

void ledon(int ledcount, char color){
  Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(ledcount, PIN, NEO_GRB + NEO_KHZ800);
  strip1.begin();
  uint32_t c;
  switch (color)
  {
  case 'R': c = strip1.Color(255,0,0); break;
  case 'G': c = strip1.Color(0,255,0); break;  
  default: c = strip1.Color(0,0,255); 
    break;
  }
  for(uint16_t i=0; i<ledcount; i++){
    strip1.setPixelColor(i, c);
    strip1.show();
  }
  
}