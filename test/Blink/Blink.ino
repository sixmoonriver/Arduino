bool _ABVAR_1_light;

void setup()
{
pinMode( 13 , OUTPUT);
_ABVAR_1_light = false;
pinMode( 1 , INPUT);
}

void loop()
{
digitalWrite( 13 , HIGH );
delay( 500 );
digitalWrite( 13 , LOW );
delay( 500 );
}


