#include <IRremote.h>
#include <IRremoteInt.h>

#include <IRremote.h>
int RECV_PIN=8;
IRrecv irrecv(RECV_PIN);
decode_results results;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(12,1);
}

void loop() {
  // put your main code here, to run repeatedly:

}
