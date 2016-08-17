/*   Feedback-based tracking for prototype solar concentrator
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Controls the crossed Zaber X-LRM200A linear stages.  Makes small changes in X and Y while measuring the change in voltage between movements.
 *   Attempts to maximize the voltage tied to pinMPPT.
 *   
 */

#include <zaberx.h>

#include <Wire.h>

#include <SoftwareSerial.h>

int pinMPPT = 0;   //Analog pin used to read voltage across MPPT load resistor
int pinPyro = 1;
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

String serialComm;
String comm1;

// Period of feedback iterations
const int interval = 2500;

int dLay = 500;   //time between incremental movement and photodiode voltage read
int iter8 = 500;   //number of reads the photodiode voltage is averaged over

// On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 3;
int TXpin = 4;

// Reset pins for digital potentiometers
int resetCPV = 26;
int resetPV = 27;

// Pins for controlling latching relays
int cpvSMU = 24;
int cpvTIA = 25;
int pvSMU = 28;
int pvTIA = 29;

unsigned int dpData;

byte dpCommand[2];    // [ MSByte, LSByte ]

boolean enable = true;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

void setup()
{
  Serial.begin(9600);
  rs232.begin(115200);
  delay(200);
  rs232.println("/renumber");
  delay(2000);
  rs232.println("/set maxspeed 200000");
  delay(1000);
}

void loop()
{
  // Serial commands to start/stop optimization, measure pyranometer voltage, and get the current position of the stages 
  if(Serial.available() > 0)
  {
    serialComm = Serial.readStringUntil('\n');    
    if(serialComm == "stop")
    {
      enable = false;
    }
    else if(serialComm == "start")
    {
      enable = true;
    }
    else if(serialComm == "meas")
    {      
      Serial.println(readAnalog(pinPyro, iter8));
    }
    else if(serialComm == "getpos")
    {
      posX = sendCommand(axisX, getPos, 0);
      posY = sendCommand(axisY, getPos, 0);
      Serial.print(posX);
      Serial.print(',');
      Serial.println(posY);
    }
    else if(serialComm == "cpvsmu")
    {
      digitalWrite(cpvSMU, HIGH);
      delay(5);
      digitalWrite(cpvSMU, LOW);
    }
    else if(serialComm == "cpvtia")
    {
      digitalWrite(cpvTIA, HIGH);
      delay(5);
      digitalWrite(cpvTIA, LOW);
    }
    else if(serialComm == "pvsmu")
    {
      digitalWrite(pvSMU, HIGH);
      delay(5);
      digitalWrite(pvSMU, LOW);
    }
    else if(serialComm == "pvtia")
    {
      digitalWrite(pvTIA, HIGH);
      delay(5);
      digitalWrite(pvTIA, LOW);
    }
    
    if(serialComm.substring(0, 5) == "setpv")
    {
      comm1 = serialComm.substring(6);
      dpData = comm1.toInt();

      // Generating two bytes to be sent to the digipot shift register, MSByte first
      dpCommand[0] = (1024 + dpData) >> 8;
      dpCommand[1] = dpData & 255;
  
      Wire.beginTransmission(0x2C);
      Wire.write(dpCommand, 2);
      Wire.endTransmission();
    }
    if(serialComm.substring(0, 6) == "setcpv")
    {
      comm1 = serialComm.substring(7);
      dpData = comm1.toInt();

      // Generating two bytes to be sent to the digipot shift register, MSByte first
      dpCommand[0] = (1024 + dpData) >> 8;
      dpCommand[1] = dpData & 255;

      Wire.beginTransmission(0x2F);
      Wire.write(dpCommand, 2);
      Wire.endTransmission(); 
    }
  }

  // Running optimization function along X and Y
  currentMillis = millis();
  if((currentMillis - previousMillis >= interval) && (enable == true))
  {   
    previousMillis = currentMillis;
    optimize(axisX, um(10));
    optimize(axisY, um(10));        
  }
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
  
  //Serial.println(voltage);

  // Move one increment in + direction and get new voltage and position
  zMoveRel(axis, increment);  
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8); 

  /*
  Serial.print(axis);
  Serial.println(" + initial");
  Serial.println(voltage);
  */
  
  // Start optimizing along axis
  if(voltage > previousVoltage)         
  {
     while(voltage > previousVoltage)
      {        
        previousVoltage = voltage;
        zMoveRel(axis, increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 

        /*
        Serial.print(axis);
        Serial.println(" +");
        Serial.println(voltage);
        */
      }
      zMoveRel(axis, (-1)*increment);
   }
   else if(voltage < previousVoltage)
   {
      previousVoltage = voltage;
      zMoveRel(axis, (-2)*increment);      
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 

      /*
      Serial.print(axis);
      Serial.println(" 2-");
      Serial.println(voltage);
      */
      
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;
        zMoveRel(axis, (-1)*increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 

        /*
        Serial.print(axis);
        Serial.println(" -");
        Serial.println(voltage);
        */
      }
      zMoveRel(axis, increment);
   }     
}
