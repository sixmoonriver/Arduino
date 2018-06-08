void setup()
{
Serial.begin(9600);
pinMode( 12 , INPUT);
}

void loop()
{
Serial.print( digitalRead( 12) );
Serial.println("");
}


