void setup()
{
Serial.begin(9600);
}

void loop()
{
Serial.print( "test" );
Serial.print( analogRead(A0) );
Serial.println("");
}


