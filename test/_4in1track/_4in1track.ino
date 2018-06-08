void setup()
{
Serial.begin(9600);
pinMode( 3 , INPUT);
}

void loop()
{
Serial.print( "The level is" );
Serial.print( digitalRead( 3) );
Serial.println("");
}


