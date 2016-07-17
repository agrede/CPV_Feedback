#include <SoftwareSerial.h>

int pinADC = 1;   //Analog pin used to read voltage of feedback variable
int voltage1 = 0;   //Voltage of lower bound
int voltage2 = 0;   //Voltage of midpoint
int voltage3 = 0;   //Voltage of upper bound
long offsetX = 3880000;    //tracking the starting and current absolute positions of the stages
long offsetY = 1000000;
long posX = offsetX;
long posY = offsetY;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// Define common command numbers
int axisX = 1;
int axisY = 2;
String Stop = "stop";
String SetMaxspeed = "set maxspeed";
String GetPos = "get pos";
String moveAbsX = "/1 move abs ";
String moveAbsY = "/2 move abs ";
String moveRelX = "/1 move rel ";
String moveRelY = "/2 move rel ";
String comm;

// Define resolution (0.000047625 for X-LRM200A-E03)
float mmResolution = 0.000047625; 
float umResolution = 0.047625; 

//Period of feedback iterations
const int interval = 5000;

int dLay = 80;   //time between incremental movement and photodiode voltage read
int iter8 = 100;   //number of reads the photodiode voltage is averaged over

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 11;
int TXpin = 4;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

void setup()
{
  Serial.begin(9600);
  rs232.begin(115200);
  delay(2000);
  /*
  rs232.println("/renumber");
  delay(1000);
  rs232.println("/home");
  /delay(10000);
  rs232.println("/1 set maxspeed 200000");  // Maxspeed = (0.02906799 * <value>) μm/sec
  rs232.println("/2 set maxspeed 200000");  // Maxspeed = (0.02906799 * <value>) μm/sec
  */
  comm = moveAbsX + offsetX;
  rs232.println(comm);
  comm = moveAbsY + offsetY;
  rs232.println(comm);
  delay(5000);
}

void loop() 
{
  currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    sectorSearch(um(100));
    sectorSearch(um(10));
  }
}

int readAnalog(int analogPin, int iterations)
{
  long volTemp = 0;
  for(int i = 0; i < iterations; i++)
  {
    volTemp += analogRead(analogPin);
  }
  int voltage = volTemp/iterations;
  Serial.println(voltage);
  return voltage;
}

void zMove(int axis, long pos)
{
  String command;
  if(axis == 1)
  {
    posX = offsetX + pos;
    command = moveAbsX + posX;    
  }
  else if(axis == 2)
  {
    posY = offsetY + pos;
    command = moveAbsY + posY;
  }  
  rs232.println(command);
}

void zMoveRel(int axis, long dist)
{
  String command;
  if(axis == 1)
  {
    posX += dist;
    command = moveRelX + dist;    
  }
  else if(axis == 2)
  {
    posY += dist;
    command = moveRelY + dist;
  }  
  rs232.println(command);
}

long mm(float mmValue)
{
  long dataValue;
  dataValue = mmValue / mmResolution;
  return dataValue;
} 

long um(float umValue)
{
  long dataValue;
  dataValue = umValue / umResolution;
  return dataValue;
} 

void sectorSearch(long dist)
{ 
  //  X-axis
  //  Taking the three initial voltage reads at three consecutive points
  long trump = dist/2;
  
  voltage2 = readAnalog(pinADC, iter8);
  zMoveRel(axisX, dist);
  delay(dLay);
  voltage3 = readAnalog(pinADC, iter8);
  zMoveRel(axisX, (-2)*dist);
  delay(dLay);
  voltage1 = readAnalog(pinADC, iter8);
  zMoveRel(axisX, dist);

  //  Finding the interval that contains a maximum
  while((voltage2 <= voltage1) || (voltage2 <= voltage3))
  {
    if(voltage1 >= voltage3)
    {
      voltage3 = voltage2;
      voltage2 = voltage1;
      zMoveRel(axisX, (-2)*dist);
      delay(dLay);
      voltage1 = readAnalog(pinADC, iter8);
    }
    else if(voltage1 < voltage3)
    {
      voltage1 = voltage2;
      voltage2 = voltage3;
      zMoveRel(axisX, 2*dist);
      delay(dLay);
      voltage3 = readAnalog(pinADC, iter8);
    }
  }
    
  while((abs(voltage2 - voltage1) > 3) && (abs(voltage2 - voltage3) > 3))
  {    
    if(voltage1 > voltage3)
    {
      voltage3 = voltage2;
      zMoveRel(axisX, (-1)*trump);
      delay(dLay);
      voltage2 = readAnalog(pinADC, iter8);
    }
    else if(voltage3 > voltage1)
    {
      voltage1 = voltage2;
      zMoveRel(axisX, trump);
      delay(dLay);
      voltage2 = readAnalog(pinADC, iter8);
    }
    trump /= 2;
  } 
  
  //  Y-axis
  //  Taking the three initial voltage reads at three consecutive points
  trump = dist/2;
  voltage2 = readAnalog(pinADC, iter8);
  zMoveRel(axisY, dist);
  delay(dLay);
  voltage3 = readAnalog(pinADC, iter8);
  zMoveRel(axisY, (-2)*dist);
  delay(dLay);
  voltage1 = readAnalog(pinADC, iter8);
  zMoveRel(axisY, dist);

  //  Finding the interval that contains a maximum
  while((voltage2 <= voltage1) || (voltage2 <= voltage3))
  {
    if(voltage1 >= voltage3)
    {
      voltage3 = voltage2;
      voltage2 = voltage1;
      zMoveRel(axisY, (-2)*dist);
      delay(dLay);
      voltage1 = readAnalog(pinADC, iter8);
    }
    else if(voltage1 < voltage3)
    {
      voltage1 = voltage2;
      voltage2 = voltage3;
      zMoveRel(axisY, 2*dist);
      delay(dLay);
      voltage3 = readAnalog(pinADC, iter8);
    }
  }

  trump = dist/2;
  
  while((abs(voltage2 - voltage1) > 3) && (abs(voltage2 - voltage3) > 3))
  {    
    if(voltage1 > voltage3)
    {
      voltage3 = voltage2;
      zMoveRel(axisY, (-1)*trump);
      delay(dLay);
      voltage2 = readAnalog(pinADC, iter8);
    }
    else if(voltage3 > voltage1)
    {
      voltage1 = voltage2;
      zMoveRel(axisY, trump);
      delay(dLay);
      voltage2 = readAnalog(pinADC, iter8);
    }
    trump /= 2;
  } 
}

