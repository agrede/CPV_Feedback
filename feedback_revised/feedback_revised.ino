#include <zaberx.h>

#include <SoftwareSerial.h>

int pinMPPT = 0;   //Analog pin used to read voltage across MPPT load resistor
int voltage = 0;   //value read from MPPT
int previousVoltage = 0;  //MPPT value from previous iteration
int offsetX = 0;    //tracking the starting and current absolute positions of the stages
int offsetY = 0;
int posX = 0;
int posY = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// Define common command numbers
int axisX = 1;
int axisY = 2;
String Home = "home";
String moveAbsX = "/1 move abs ";
String moveAbsY = "/2 move abs ";
String moveRelX = "/1 move rel ";
String moveRelY = "/2 move rel ";
String Stop = "stop";
String SetMaxspeed = "set maxspeed";
String GetPos = "get pos";
String comm;

//Period of feedback iterations
const int interval = 15000;

int dLay = 75;   //time between incremental movement and photodiode voltage read
int iter8 = 100;   //number of reads the photodiode voltage is averaged over

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 3;
int TXpin = 4;

int interruptPin = 2;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

void setup()
{
  Serial.begin(9600);
  rs232.begin(115200);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), zStop, FALLING);
  delay(200);
  rs232.println("/renumber");
  delay(5000);

}

void loop()
{
  currentMillis = millis();
  if((currentMillis - previousMillis >= interval) && (analogRead(pinMPPT) < 1010))
  {   
    previousMillis = currentMillis;
    optimize(um(50));
    optimize(um(5));
  }

}

void zStop()
{
  rs232.println("/stop");
}

void zMove(int axis, long pos)
{
  String command;
  if(axis == 1)
  {
    posX = pos;
    command = moveAbsX + posX;    
  }
  else if(axis == 2)
  {
    posY = pos;
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


void optimize(long increment)
{ 
  //Get starting conditions before optimizing
  voltage = readAnalog(pinMPPT, iter8); 

  //Move one increment in +x and get new voltage and position
  zMoveRel(axisX, increment);
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8); 
  
  //Start optimizing along X axis
  if(voltage > previousVoltage)         
  {
     while(voltage > previousVoltage)
      {        
        previousVoltage = voltage;
        zMoveRel(axisX, increment);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }
    else if(voltage <= previousVoltage)
    {
      previousVoltage = voltage;
      zMoveRel(axisX, (-2)*increment);
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 
      while(voltage >= previousVoltage)
      {
        previousVoltage = voltage;
        zMoveRel(axisX, (-1)*increment);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }

  //Start optimizing along Y axis
  voltage = readAnalog(pinMPPT, iter8); 
  previousVoltage = voltage;  
  zMoveRel(axisY, increment);
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8);
  
  if(voltage > previousVoltage)           
  {
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;      
        zMoveRel(axisY, increment);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }
    else if(voltage <= previousVoltage)
    {
      previousVoltage = voltage;
      zMoveRel(axisY, (-2)*increment);
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 
      while(voltage >= previousVoltage) 
      {
        previousVoltage = voltage;
        zMoveRel(axisY, (-1)*increment);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }      
    
}
