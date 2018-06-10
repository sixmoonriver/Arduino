char poweron;
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Please input 0/1:");
  while(Serial.available()>0) 
    poweron = Serial.read();
  if(int(poweron))
  {
    Serial.print("Pown state is ");
    Serial.println(poweron);
  }
  else
   {
    Serial.print("Pown state is ");
    Serial.println(poweron);
   }
  delay(500);
  
}
