#include <SoftwareSerial.h>

#include <zaberx.h>

int pinADC = 0;
int voltage;
int iter8 = 100;

void setup()
{
  Serial.begin(9600);
  //analogReference(EXTERNAL);
  delay(500);
}

void loop() 
{
  voltage = readAnalog(pinADC, iter8);
  Serial.println(voltage);
  delay(250);
}
