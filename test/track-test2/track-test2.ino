int ardublockUltrasonicSensorCodeAutoGeneratedReturnCM(int trigPin, int echoPin)
{
  int duration;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  duration = duration / 59;
  return duration;
}


void setup()
{
pinMode( 13 , OUTPUT);
digitalWrite( 0 , LOW );

}

void loop()
{
if (( ( ardublockUltrasonicSensorCodeAutoGeneratedReturnCM( 0 , 1 ) ) >= ( 20 ) ))
{
digitalWrite( 13 , LOW );
}
else
{
if (( ( ardublockUltrasonicSensorCodeAutoGeneratedReturnCM( 0 , 1 ) ) <= ( 10 ) ))
{
digitalWrite( 13 , HIGH );
delay( 1000 );
digitalWrite( 13 , LOW );
delay( 1000 );
}
if (( ( ardublockUltrasonicSensorCodeAutoGeneratedReturnCM( 0 , 1 ) ) < ( 5 ) ))
{
digitalWrite( 13 , HIGH );
delay( 200 );
digitalWrite( 13 , LOW );
delay( 200 );
}
}
}


