void setup() {
  // put your setup code here, to run once:

  for(int i=2;i<6;i++)
  {
    pinMode(i,OUTPUT);
  } 
}

void loop() {
  // put your main code here, to run repeatedly:

  int a;
  a=512;
  while(a--)
  {
   for(int i=2;i<6;i++)
   {
    digitalWrite(i,1);
    delay(100);
   digitalWrite(i,0); 
   }
  }
}
