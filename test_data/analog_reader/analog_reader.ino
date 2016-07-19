#include <SoftwareSerial.h>

#include <zaberx.h>

int pinADC = 0;
int voltage;
int iter8 = 100;

String serialCom;

boolean enable = true;

void setup()
{
  Serial.begin(9600);
  //analogReference(EXTERNAL);
  delay(500);
}

void loop() 
{
  if(Serial.available() > 0)
  {
    serialCom = Serial.readStringUntil('\n');
    if (serialCom == "stop")
    {
      enable = false;
    }
    else if (serialCom == "start")
    {
      enable = true;
    }
  }
  if (enable == true)
  {
     voltage = readAnalog(pinADC, iter8);
     Serial.println(voltage);
  }
  delay(250);
}
