/*   Feedback-based tracking for prototype solar concentrator
 *    Using Zaber Binary protocol
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Barebones closed-loop tracking algorithm.  Controls the crossed Zaber X-LRM200A linear stages.  Makes small changes in 
 *   X and Y while measuring the change in voltage between movements.  Attempts to maximize the voltage tied to pinMPPT.   
 */
 
#include <zaberx.h>

#include <SoftwareSerial.h>

int voltage = 0;   //value read from MPPT
int previousVoltage = 0;  //MPPT value from previous iteration

const unsigned long offsetX = 2148185;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 2104209;

unsigned long posX = 0;    // Tracks the absolute position of the X-axis stage (in microsteps)
unsigned long posY = 0;    // Tracks the absolute position of the Y-axis stage (in microsteps)

// Variables for Zaber binary communication
byte command[6];
byte reply[6];
float outData;
long replyData;

// Port IDs
int portA = 1;
int portB = 2;

// Linear Stage IDs
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

int dLay = 100;   //time between incremental movement and photodiode voltage read
int iter8 = 100;   //number of reads the photodiode voltage is averaged over

// Period of feedback iterations
const int intervalCPV = 5000;

unsigned long millisCPV = 0;
unsigned long currentMillis = 0;

// PIN ASSIGNMENTS

// Transimpedance amplifier outputs
int pinPyro = 8;   // Bare pyranometer
int pinDNI = 9;    // DNI pyranometer
int pinPV = 10;    // Bare cell
int pinCPV = 11;   // Concentrator cell

// On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
// Linear Stages Serial comm.
int RXpin = 2;      
int TXpin = 3;

SoftwareSerial rs232a(RXpin, TXpin);   //RX, TX

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  
  // Sets the stages to use binary protocol
  rs232a.begin(115200);
  delay(1000);  
  rs232a.println("/tools setcomm 9600 1");
  delay(500);
  Serial.println(rs232a.readStringUntil('\n'));
  delay(100);
  rs232a.end();
  delay(200);

  //Start software serial connection with Zaber stages
  rs232a.begin(9600);
  delay(2000);
}

void loop()
{
  // Running optimization function along X and Y
  currentMillis = millis();
  if(currentMillis - millisCPV >= intervalCPV)
  {   
    millisCPV = currentMillis;
    optimize(axisX, um(10));
    optimize(axisY, um(10));        
  }
}

long sendCommand(int port, int device, int com, long data)
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
   if(port == 1)
   {
     while(rs232a.available() > 0)
     {
       rs232a.readBytes(dumper, 1);
     }
   
     // Sending command to stage(s)
     rs232a.write(command, 6);

     delay(20);
   
     // Reading device reply
     if(rs232a.available() > 0)
     {
       rs232a.readBytes(reply, 6);
     }   
   }
   else if(port == 2)
   {
     while(rs232b.available() > 0)
     {
       rs232b.readBytes(dumper, 1);
     }
   
     // Sending command to stage(s)
     rs232b.write(command, 6);

     delay(20);
   
     // Reading device reply
     if(rs232b.available() > 0)
     {
       rs232b.readBytes(reply, 6);
     }   
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
  voltage = readAnalog(pinCPV, iter8); 
  
  //Serial.println(voltage);

  // Move one increment in + direction and get new voltage and position
  replyData = sendCommand(portA, axis, moveRel, increment);
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinCPV, iter8); 

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
        replyData = sendCommand(portA, axis, moveRel, increment);        
        delay(dLay);
        voltage = readAnalog(pinCPV, iter8);  

        /*
        Serial.print(axis);
        Serial.println(" +");
        Serial.println(voltage);
        */
      }
      replyData = sendCommand(portA, axis, moveRel, (-1)*increment);
   }
   else if(voltage < previousVoltage)
   {
      previousVoltage = voltage;
      replyData = sendCommand(portA, axis, moveRel, (-2)*increment);    
      delay(dLay);
      voltage = readAnalog(pinCPV, iter8);  

      /*
      Serial.print(axis);
      Serial.println(" 2-");
      Serial.println(voltage);
      */
      
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;
        replyData = sendCommand(portA, axis, moveRel, (-1)*increment);        
        delay(dLay);
        voltage = readAnalog(pinCPV, iter8); 

        /*
        Serial.print(axis);
        Serial.println(" -");
        Serial.println(voltage);
        */
      }
      replyData = sendCommand(portA, axis, moveRel, increment);
   }     
}
