#include <Arduino.h>

#include <Adafruit_PCF8574.h>

/* Example for 8 output LEDs that are connected from power to the GPIO expander pins
 * Note the LEDs must be connected with the CATHODES to the expander, to SINK current!
 * The PCF8574 cannot SOURCE current!
 */
#define ARDUINO_IRQ 2  // make sure this pin is possible to make IRQ input

// We use a flag to make sure we don't enter the interrupt more than once


Adafruit_PCF8574 pcf;

void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println("Adafruit PCF8574 LED blink test");

  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
  for (uint8_t p=0; p<8; p++) {
    //pcf.pinMode(p, OUTPUT);
  pcf.pinMode(p, INPUT_PULLUP);
  }
    // set up the interrupt pin on IRQ signal toggle
  pinMode(ARDUINO_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ARDUINO_IRQ), button_detect, CHANGE);
}

void loop() {
  /*for (uint8_t p=0; p<8; p++) {
    pcf.digitalWriteByte();
    pcf.digitalWrite(p, LOW);  // turn LED on by sinking current to ground
    delay(100);
    pcf.digitalWrite(p, HIGH); // turn LED off by turning off sinking transistor
  }*/
  // Serial.println(pcf.digitalReadByte(), BIN);
  delay(200);
}
volatile bool in_irq = false;
// called when the button is pressed!
void button_detect(void) {
  if (in_irq) return; // we are already handling an irq so don't collide!
  
  in_irq = true;
  interrupts(); // Arduino UNO seems to require that we turn on interrupts for I2C to work!
  //bool val = pcf.digitalRead(PCF_BUTTON);
  //pcf.digitalWrite(PCF_LED, val);
  Serial.println(pcf.digitalReadByte(), BIN);
  in_irq = false;
}