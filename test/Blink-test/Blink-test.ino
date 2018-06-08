bool _ABVAR_1_light;

void setup()
{
pinMode( 13 , OUTPUT);
_ABVAR_1_light = false;
pinMode( 2 , INPUT);
}

void loop()
{
if (_ABVAR_1_light)
{
digitalWrite( 13 , HIGH );
}
else
{
digitalWrite( 13 , LOW );
}
if (!( digitalRead( 2) ))
{
delay( 5 );
if (!( digitalRead( 2) ))
{
_ABVAR_1_light = !( _ABVAR_1_light ) ;
while ( !( digitalRead( 2) ) )
{
}

}
}
}


