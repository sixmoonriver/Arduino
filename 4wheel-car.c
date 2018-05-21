int ardublockUltrasonicSensorCodeAutoGeneratedReturnCM(int trigPin, int echoPin)
{
  long duration;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  duration = duration / 59;
  if ((duration < 2) || (duration > 300)) return false;
  return duration;
}


void setup()
{
  digitalWrite( 8 , LOW );

  pinMode( 13 , OUTPUT);
}

void loop()
{
  if (( ( 	ardublockUltrasonicSensorCodeAutoGeneratedReturnCM( 8 , 9 ) ) > ( 20 ) ))
  {
    digitalWrite( 13 , HIGH );
    delay( 1000 );
  }
  else
  {
    digitalWrite( 13 , LOW );
    delay( 1000 );
  }
}

