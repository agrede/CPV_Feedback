/*   Feedback-based tracking for prototype solar concentrator
 *    Test Sketch
 *   
 *   Michael Lipski
 *   AOPL
 *   Summer 2016
 *   
 *   Controls the crossed Zaber X-LRM200A linear stages.  Makes small changes in X and Y while measuring the change in voltage between movements.
 *   Attempts to maximize the voltage tied to pinMPPT.
 *   Contains the most recent changes.
 *   
 */

#include <zaberx.h>

#include <Wire.h>

#include <SoftwareSerial.h>


int voltage = 0;   //value read from MPPT
int previousVoltage = 0;  //MPPT value from previous iteration

const unsigned long offsetX = 2148185;    //tracking the starting and current absolute positions of the stages
const unsigned long offsetY = 2104209;

unsigned long posX = 0;    // Tracks the absolute position of the X-axis stage (in microsteps)
unsigned long posY = 0;    // Tracks the absolute position of the Y-axis stage (in microsteps)

unsigned long azimPos = 0;   // Variable which tracks the absolute position of the azimuth stage (in microsteps)
unsigned long zeniPos = 0;   // Variable which tracks the absolute position of the elevation stage (in microsteps)

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

// Rotational Stage IDs
int azimuth = 1;    // Device ID of azimuth stage
int zenith = 2;     // Device ID of elevation stage

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

int dLay = 500;   //time between incremental movement and photodiode voltage read
int iter8 = 100;   //number of reads the photodiode voltage is averaged over

// Period of feedback iterations
const int intervalCPV = 2500;
const int intervalDNI = 2500;

unsigned long millisCPV = 0;
unsigned long millisDNI = 0;
unsigned long currentMillis = 0;

// PIN ASSIGNMENTS

// Transimpedance amplifier outputs
int pinPyro = 8;   // Bare pyranometer
int pinDNI = 9;    // DNI pyranometer
int pinPV = 10;    // Bare cell
int pinCPV = 11;   // Concentrator cell

// Photoresistor analog pins
int topR = 0;       // top right photoresistor
int topL = 1;       // top left photoresistor
int bottomR = 2;    // bottom right photoresistor
int bottomL = 3;    // bottom left photoresistor

// On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
// Linear Stages Serial comm.
int RXpin = 2;      
int TXpin = 3;

// Rotational Stages Serial comm.
int RXpin2 = 4;
int TXpin2 = 5;

// Reset pins for digital potentiometers
int resetCPV = 26;
int resetPV = 27;

// Pins for controlling latching relays
int cpvSMU = 25;
int cpvTIA = 24;
int pvSMU = 29;
int pvTIA = 28;

unsigned int dpData;      // 10-bit value to be sent to the desired digital potentiometer

byte dpCommand[2];    // [ MSByte , LSByte ]

boolean enableCPV = true;    // Controls whether or not the CPV closed-loop optimization routine is running
boolean enableDNI = true;    // Controls whether or not the photoresistor-based DNI pyranometer tracking is running

SoftwareSerial rs232a(RXpin, TXpin);   //RX, TX

SoftwareSerial rs232b(RXpin2, TXpin2);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  Wire.begin();

    // Memory efficient version of pin initialization, for Mega2560 only
  DDRA |= B11111100;    // Sets Mega 2560 pins 24-29 as outputs
  PORTA = B00110000;    // Sets 26, 27 HIGH and 22-25, 28, 29 LOW
  
  /*
  // Initializes the proper pins as outputs and ensures that they are at logic low
  pinMode(resetCPV, OUTPUT);
  pinMode(resetPV, OUTPUT);
  pinMode(cpvSMU, OUTPUT);
  pinMode(cpvTIA, OUTPUT);
  pinMode(pvSMU, OUTPUT);
  pinMode(pvTIA, OUTPUT);
  digitalWrite(resetCPV, HIGH);
  digitalWrite(resetPV, HIGH);
  digitalWrite(cpvSMU, LOW);
  digitalWrite(cpvTIA, LOW);
  digitalWrite(pvSMU, LOW);
  digitalWrite(pvTIA, LOW);
  */
  
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
  rs232b.begin(9600);
  delay(2000);
}

void loop()
{
  // Serial commands to start/stop optimization, measure pyranometer voltage, and get the current position of the stages 
  if(Serial.available() > 0)
  {
    serialComm = Serial.readStringUntil('\n');    
    if(serialComm == "stopcpv")
    {
      enableCPV = false;
    }
    else if(serialComm == "startcpv")
    {
      enableCPV = true;
    }
    else if(serialComm == "stopdni")
    {
      enableDNI = false;
    }
    else if(serialComm == "startdni")
    {
      enableDNI = true; 
    }
    else if(serialComm == "measpyr")
    {      
      Serial.println(readAnalog(pinPyro, iter8));
    }
    else if(serialComm == "measdni")
    {
      Serial.println(readAnalog(pinDNI, iter8));
    }
    else if(serialComm == "measpv")
    {
      Serial.println(readAnalog(pinPV, iter8));
    }
    else if(serialComm == "meascpv")
    {
      Serial.println(readAnalog(pinCPV, iter8));
    }
    else if(serialComm == "getposx")
    {
      posX = sendCommand(portA, axisX, getPos, 0);
      posY = sendCommand(portA, axisY, getPos, 0);
      Serial.print(posX);
      Serial.print(',');
      Serial.println(posY);
    }
    else if(serialComm == "getpost")
    {
      azimPos = sendCommand(portB, azimuth, getPos, 0);
      zeniPos = sendCommand(portB, zenith, getPos, 0);
      Serial.print(azimPos);
      Serial.print(',');
      Serial.println(zeniPos);
    }
    else if(serialComm == "getldr")
    {
      Serial.print(readAnalog(topR, iter8));
      Serial.print(',');
      Serial.print(readAnalog(topL, iter8));
      Serial.print(',');
      Serial.print(readAnalog(bottomR, iter8));
      Serial.print(',');
      Serial.println(readAnalog(bottomL, iter8));      
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
  if((currentMillis - millisCPV >= intervalCPV) && (enableCPV == true))
  {   
    millisCPV = currentMillis;
    optimize(axisX, um(10));
    optimize(axisY, um(10));        
  }

  // Running tracking routine for DNI pyranometer
  if((currentMillis - millisDNI >= intervalDNI) && (enableDNI == true))
  {   
    millisDNI = currentMillis;
    quadrant(stepsD(0.2));       
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

void quadrant(long increment)
{
  // Find voltages from photoresistor voltage divider
  int vTR = readAnalog(topR, iter8);   // voltage from top right photoresistor
  int vTL = readAnalog(topL, iter8);    // voltage from top left photoresistor
  int vBR = readAnalog(bottomR, iter8);    // voltage from bottom right photoresistor
  int vBL = readAnalog(bottomL, iter8);    // voltage from bottom left photoresistor

  // Print 10-bit values read by the ADC from photoresistor voltage divider
  Serial.print("Top Right: ");
  Serial.print(vTR);
  Serial.print("\tTop Left: ");
  Serial.print(vTL);
  Serial.print("\tBottom Right: ");
  Serial.print(vBR);
  Serial.print("\tBottom Left: ");
  Serial.println(vBL);

  // Find average values
  int top = (vTR + vTL) / 2;      // average of top right and top left voltages
  int bottom = (vBR + vBL) / 2;   // average of bottom right and bottom left voltages
  int right = (vTR + vBR) / 2;    // average of top right and bottom right voltages
  int left = (vTL + vBL) / 2;     // average of top left and bottom left voltages

  if(top > bottom)
  {
    replyData = sendCommand(portB, zenith, moveRel, (-1)*increment);
  }
  else if(top < bottom)
  {
    replyData = sendCommand(portB, zenith, moveRel, increment);
  }

  if(right > left)
  {
    replyData = sendCommand(portB, azimuth, moveRel, increment);
  }
  else if(right < left)
  {
    replyData = sendCommand(portB, azimuth, moveRel, (-1)*increment);
  }  
}
