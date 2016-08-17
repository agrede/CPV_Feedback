/*   Feedback-based tracking for prototype solar concentrator
 *    Using Zaber Binary protocol
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

const unsigned long offsetX = 2148185;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 2104209;
unsigned long posX = 0;
unsigned long posY = 0;

// Variables for Zaber binary communication
byte command[6];
byte reply[6];
float outData;
long replyData;

// Stage IDs
int axisX = 1;
int axisY = 2;

// Define common command numbers
int homer = 1;      // home the stage
int renumber = 2;   // renumber all devices in the chain
int moveAbs = 20;   // move absolute
int moveRel = 21;   // move relative
int stopMove = 23;  // Stop
int speedSet = 42;    // Speed to target = 0.00219727(V) degrees/sec (assuming 64 microstep resolution)
int getPos = 60;      // Query the device for its position
int storePos = 16;    // Position can be stored in registers 0 to 15
int returnPos = 17;   // returns the value (in microsteps) of the position stored in the indicated register
int move2Pos = 18;    // move to the position stored in the indicated register
int reset = 0;        // akin to toggling device power

String serialComm;
String comm1;

// Period of feedback iterations
const int interval = 2500;

int dLay = 500;   //time between incremental movement and photodiode voltage read
int iter8 = 500;   //number of reads the photodiode voltage is averaged over

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

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

unsigned int dpData;      // 10-bit value to be sent to the desired digital potentiometer

byte dpCommand[2];    // [ MSByte , LSByte ]

boolean enable = true;    // Controls whether or not the closed-loop optimization routine is running

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  
  // Sets the stages to use binary protocol
  rs232.begin(115200);
  delay(1000);  
  rs232.println("/tools setcomm 9600 1");
  delay(500);
  Serial.println(rs232.readStringUntil('\n'));
  delay(100);
  rs232.end();
  delay(200);

  //Start software serial connection with Zaber stages
  rs232.begin(9600);
  delay(2000);
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

long sendCommand(int device, int com, long data)
{
   unsigned long data2;
   unsigned long temp;
   unsigned long repData;
   long replyNeg;
   float replyFloat;
   byte dumper[1];
   
   // Building the six command bytes
   command[0] = byte(device);
   command[1] = byte(com);
   if(data < 0)
   {
     data2 = data + quad;
   }
   else
   {
     data2 = data;
   }
   temp = data2 / cubed;
   command[5] = byte(temp);
   data2 -= (cubed * temp);
   temp = data2 / squared;
   command[4] = byte(temp);
   data2 -= (squared * temp);
   temp = data2 / 256;
   command[3] = byte(temp);
   data2 -= (256 * data2);
   command[2] = byte(data2);
   
   // Clearing serial buffer
   while(rs232.available() > 0)
   {
     rs232.readBytes(dumper, 1);
   }
   
   // Sending command to stage(s)
   rs232.write(command, 6);

   delay(20);
   
   // Reading device reply
   if(rs232.available() > 0)
   {
     rs232.readBytes(reply, 6);
   }
   
   replyFloat = (cubed * float(reply[5])) + (squared * float(reply[4])) + (256 * float(reply[3])) + float(reply[2]); 
   repData = long(replyFloat);
   
   if(reply[5] > 127)
   {
     replyNeg = repData - quad;
   }
   
   // Printing full reply bytes as well as reply data in decimal 
   Serial.print(reply[0]);
   Serial.print(' ');
   Serial.print(reply[1]);
   Serial.print(' ');
   Serial.print(reply[2]);
   Serial.print(' ');
   Serial.print(reply[3]);
   Serial.print(' ');
   Serial.print(reply[4]);
   Serial.print(' ');
   Serial.println(reply[5]);
   Serial.print("\tData:");
   if(reply[5] > 127)
   {
     Serial.println(replyNeg);
     return replyNeg;
   }
   else
   {
     Serial.println(repData);  
     return repData;
   }    
}

void optimize(int axis, long increment)
{ 
  // Get starting conditions before optimizing
  voltage = readAnalog(pinMPPT, iter8); 
  
  //Serial.println(voltage);

  // Move one increment in + direction and get new voltage and position
  replyData = sendCommand(axis, moveRel, increment);
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
        replyData = sendCommand(axis, moveRel, increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 

        /*
        Serial.print(axis);
        Serial.println(" +");
        Serial.println(voltage);
        */
      }
      replyData = sendCommand(axis, moveRel, (-1)*increment);
   }
   else if(voltage < previousVoltage)
   {
      previousVoltage = voltage;
      replyData = sendCommand(axis, moveRel, (-2)*increment);    
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
        replyData = sendCommand(axis, moveRel, (-1)*increment);        
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 

        /*
        Serial.print(axis);
        Serial.println(" -");
        Serial.println(voltage);
        */
      }
      replyData = sendCommand(axis, moveRel, increment);
   }     
}
