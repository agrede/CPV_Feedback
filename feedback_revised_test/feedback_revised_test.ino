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
const int interval = 2500;

int dLay = 500;   //time between incremental movement and photodiode voltage read
int iter8 = 500;   //number of reads the photodiode voltage is averaged over

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
  //analogReference(EXTERNAL);
  delay(200);
  rs232.println("/renumber");
  delay(2000);
  rs232.println("/set maxspeed 200000");
  delay(1000);
}

void loop()
{
  currentMillis = millis();
  if((currentMillis - previousMillis >= interval) && (analogRead(pinMPPT) < 1010))
  {   
    previousMillis = currentMillis;
    //optimize(axisX, um(50));  
    //optimize(axisY, um(50));
    optimize(axisX, um(10));
    optimize(axisY, um(10));  
    //optimize(um(10));    
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


void optimize(int axis, long increment)
{ 
  // Get starting conditions before optimizing
  voltage = readAnalog(pinMPPT, iter8); 
  
  Serial.println(voltage);

  // Move one increment in + direction and get new voltage and position
  zMoveRel(axis, increment);  
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8); 
  
  Serial.print(axis);
  Serial.println(" + initial");
  Serial.println(voltage);
  
  // Start optimizing along axis
  if(voltage > previousVoltage)         
  {
     while(voltage > previousVoltage)
      {        
        previousVoltage = voltage;
        zMoveRel(axis, increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
        
        Serial.print(axis);
        Serial.println(" +");
        Serial.println(voltage);
      }
      zMoveRel(axis, (-1)*increment);
   }
   else if(voltage < previousVoltage)
   {
      previousVoltage = voltage;
      zMoveRel(axis, (-2)*increment);      
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 
      Serial.print(axis);
      Serial.println(" 2-");
      Serial.println(voltage);
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;
        zMoveRel(axis, (-1)*increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
        
        Serial.print(axis);
        Serial.println(" -");
        Serial.println(voltage);
      }
      zMoveRel(axis, increment);
   }     
}
