#include <zaberx.h>

int pinPyro = 0;
int iter8 = 500;
int voltage;

String comm;

void setup()
{
  Serial.begin(9600);
  delay(1000);
}

void loop() 
{
  if(Serial.available() > 0)
  {
    comm = Serial.readStringUntil('\n');
    if(comm == "meas")
    {
      voltage = readAnalog(pinPyro, iter8);
      Serial.println(voltage);
    }
  }
}
