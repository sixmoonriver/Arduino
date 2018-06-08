void setup()
{
pinMode( 7 , OUTPUT);
pinMode( 10 , OUTPUT);
pinMode( 8 , OUTPUT);
pinMode( 5 , OUTPUT);
}

void loop()
{
digitalWrite( 8 , HIGH );
digitalWrite( 10 , LOW );
digitalWrite( 5 , HIGH );
digitalWrite( 7 , LOW );
analogWrite(9, 160);
analogWrite(6, 160);
}


