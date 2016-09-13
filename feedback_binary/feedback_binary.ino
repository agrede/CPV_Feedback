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
 *   Controls the Zaber T-RS60A rotational stages controlling the DNI pyranometer.  DNI pyranometer tracking uses external photoresistor voltage
 *   divider.
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
int intervalCPV = 5000;

// Amount the stage is moved between voltage samples
int increment = 10;

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
int RXpin = 10;      
int TXpin = 3;

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
byte dpEnable[2] = {7, 2};

boolean enableCPV = true;    // Controls whether or not the CPV closed-loop optimization routine is running
boolean logData = false;     // Controls whether or not data is constantly sent to serial monitor

SoftwareSerial rs232a(RXpin, TXpin);   //RX, TX

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  Wire.begin();

  // Enable digipot wiper to be controlled over I2C & sets wiper to lowest resistance setting
  Wire.beginTransmission(0x2C);
  Wire.write(dpEnable, 2);
  Wire.write(byte(4));
  Wire.write(byte(0));
  Wire.endTransmission();
  Wire.beginTransmission(0x2F);
  Wire.write(dpEnable, 2);
  Wire.write(byte(4));
  Wire.write(byte(0));
  Wire.endTransmission();

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
    else if(serialComm == "dataon")
    {
      logData = true;
    }
    else if(serialComm == "dataoff")
    {
      logData = false;
    }
    else if(serialComm == "measpyr")
    {      
      Serial.println(readAnalog(pinPyro, iter8));
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
    
    if(serialComm.substring(0, 5) == "setdt")
    {
      comm1 = serialComm.substring(6);
      intervalCPV = comm1.toInt();
    }
    if(serialComm.substring(0, 5) == "setdz")
    {
      comm1 = serialComm.substring(6);
      increment = comm1.toInt();
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

    if(logData == true)
    {
      posX = sendCommand(portA, axisX, getPos, 0);
      posY = sendCommand(portA, axisY, getPos, 0);
      Serial.println("start");
    }
    
    optimize(axisX, um(increment));
    optimize(axisY, um(increment));   

    if(logData == true)
    {
      int x1 = posX;
      int y1 = posY;      
      posX = sendCommand(portA, axisX, getPos, 0);
      posY = sendCommand(portA, axisY, getPos, 0);
      int dx = posX - x1;
      int dy = posY - y1;
      float d = sqrt(dx*dx - dy*dy);
      Serial.print(d * umResolution);
      Serial.println(" , um direct");
      Serial.println("end");
    }
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
   /*else if(port == 2)
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
   */

   replyFloat = (cubed * float(reply[5])) + (squared * float(reply[4])) + (256 * float(reply[3])) + float(reply[2]); 
   repData = long(replyFloat);
   
   if(reply[5] > 127)
   {
     replyNeg = repData - quad;
   }

   /*
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
   */
   if(reply[5] > 127)
   {
     //Serial.println(replyNeg);
     return replyNeg;
   }
   else
   {
     //Serial.println(repData);  
     return repData;
   }    
}

void optimize(int axis, long increment)
{ 
  int moves = 0; 
  // Get starting conditions before optimizing
  voltage = readAnalog(pinCPV, iter8); 
  
  if(logData == true)
    Serial.println(voltage);

  // Move one increment in + direction and get new voltage and position
  replyData = sendCommand(portA, axis, moveRel, increment);
  moves++;
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinCPV, iter8);

  if(logData == true)
    Serial.println(voltage); 
  
  // Start optimizing along axis
  if(voltage > previousVoltage)         
  {
     while(voltage > previousVoltage)
      {        
        previousVoltage = voltage;
        replyData = sendCommand(portA, axis, moveRel, increment);  
        moves++;      
        delay(dLay);
        voltage = readAnalog(pinCPV, iter8);  

        if(logData == true)
          Serial.println(voltage);
      }
      replyData = sendCommand(portA, axis, moveRel, (-1)*increment);
      moves++;
   }
   else if(voltage < previousVoltage)
   {
      previousVoltage = voltage;
      replyData = sendCommand(portA, axis, moveRel, (-2)*increment);  
      moves += 2;  
      delay(dLay);
      voltage = readAnalog(pinCPV, iter8);  

      if(logData == true)
          Serial.println(voltage);
      
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;
        replyData = sendCommand(portA, axis, moveRel, (-1)*increment);    
        moves++;    
        delay(dLay);
        voltage = readAnalog(pinCPV, iter8); 

        if(logData == true)
          Serial.println(voltage);
      }
      replyData = sendCommand(portA, axis, moveRel, increment);
      moves++;
   }  
   if(logData == true)
   {
     Serial.print(increment * moves);  
     if(axis == axisX)
     {
       Serial.println(" , um y"); 
     }
     else if(axis == axisY)
     {
       Serial.println(" , um x"); 
     }
      
   }
}


