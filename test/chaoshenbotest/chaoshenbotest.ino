void setup()
{
Serial.begin(9600);
}

void loop()
{
Serial.print( "message" );
Serial.print( analogRead(A1) );
Serial.println("");
}


