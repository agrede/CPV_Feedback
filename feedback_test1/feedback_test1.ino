#include <SoftwareSerial.h>

struct reply {
  String fullResponse;
  bool isReply;
  int deviceAddress;
  int axisNumber;
  bool isRejected;
  bool isBusy;
  bool hasWarning;
  String warningFlag;
  String responseDataString;
  int responseData;
};

reply response;   //Reply string from Zaber stages in the form of a struct

int pinMPPT = 0;   //Analog pin used to read voltage across MPPT load resistor
int voltage = 0;   //value read from MPPT
int previousVoltage = 0;  //MPPT value from previous iteration
int posXstart = 0;    //tracking the starting and current absolute positions of the stages
int posYstart = 0;
int posX = 0;
int posY = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// Define common command numbers
String axisX = "1";
String axisY = "2";
String Home = "home";
String MoveAbs = "move abs";
String MoveRel = "move rel";
String Stop = "stop";
String SetMaxspeed = "set maxspeed";
String GetPos = "get pos";
String comm;

// Define resolution (0.000047625 for X-LRM200A-E03)
float mmResolution = 0.000047625; 
float umResolution = 0.047625; 

//Period of feedback iterations
const int interval = 5000;

int dLay = 1000;   //time between incremental movement and photodiode voltage read
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
  attachInterrupt(digitalPinToInterrupt(interruptPin), homer, FALLING);
  delay(10000);
  //rs232.println("/renumber");
  //delay(1000);
  //rs232.println("/home");
  //delay(1000);
  //rs232.println("/1 set maxspeed 200000");  // Maxspeed = (0.02906799 * <value>) μm/sec  
  //rs232.println("/2 set maxspeed 200000");  // Maxspeed = (0.02906799 * <value>) μm/sec
  //delay(1000); 
  //rs232.println("/1 move abs 3880000");  
  //rs232.println("/2 move abs 1000000");  
  //delay(10000); 
}

void loop() 
{  
  currentMillis = millis();
  if((currentMillis - previousMillis >= interval) && (analogRead(pinMPPT) < 1010))
  {   
    previousMillis = currentMillis;
    optimize(um(75));
    optimize(um(10));
  }
}

void homer()
{
  rs232.println("/home");
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

String mm(float mmValue)
{
  long dataValue;
  dataValue = mmValue / mmResolution;
  String dataR = String(dataValue);
  return dataR;
} 

String um(float umValue)
{
  long dataValue;
  dataValue = umValue / umResolution;
  String dataR = String(dataValue);
  return dataR;
} 

struct reply requestCommand(String device, String command, String data)
{
  rs232.println("/" + device + " " + command + " " + data);
  
  String fullResponse = rs232.readStringUntil('\n');
  
  String messageType = fullResponse.substring(0,1);
  String nn = fullResponse.substring(1,3);
  String a = fullResponse.substring(4,5);
  String fl = fullResponse.substring(6,8);
  String bbbb = fullResponse.substring(9,13);
  String ww = fullResponse.substring(14,16);
  String xxx = fullResponse.substring(17,fullResponse.length()-1);
    
  bool isReply = messageType.equals("@");
  int deviceAddress = nn.toInt();
  int axisNumber = a.toInt();
  bool isRejected = fl.equals("RJ");
  bool isBusy = bbbb.equals("BUSY");
  bool hasWarning = !(ww.equals("--"));
  int responseData = xxx.toInt();
  
  reply response = {fullResponse, isReply, deviceAddress, axisNumber, isRejected, isBusy, hasWarning, ww, xxx, responseData  };
  return response;
}

void pollUntilIdle(String device)
{
  while(requestCommand(device, "", "").isBusy);
  {
    delay(10);
  }
}

void optimize(String increment)
{ 
  //Get starting conditions before optimizing
  voltage = readAnalog(pinMPPT, iter8); 

  //Move one increment in +x and get new voltage and position
  pollUntilIdle(axisX);
  comm = String("/1 move rel " + increment);
  rs232.println(comm);
  previousVoltage = voltage;
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8); 
  
  //Start optimizing along X axis
  if(voltage > previousVoltage)         
  {
     while(voltage > previousVoltage)
      {
        //Serial.print("+ X   ");
        previousVoltage = voltage;
        pollUntilIdle(axisX);
        comm = String("/1 move rel " + increment);
        rs232.println(comm);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }
    else if(voltage <= previousVoltage)
    {
      previousVoltage = voltage;
      pollUntilIdle(axisX);
      comm = String('-' + increment);
      comm = String("/1 move rel " + comm);
      rs232.println(comm);
      pollUntilIdle(axisX);
      rs232.println(comm);
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 
      while(voltage >= previousVoltage)
      {
        previousVoltage = voltage;
        pollUntilIdle(axisX);
        comm = String('-' + increment);
        comm = String("/1 move rel " + comm);
        rs232.println(comm);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }

  //Start optimizing along Y axis
  voltage = readAnalog(pinMPPT, iter8); 
  previousVoltage = voltage;  
  comm = String("/2 move rel " + increment);
  rs232.println(comm);
  delay(dLay);
  voltage = readAnalog(pinMPPT, iter8);
  
  if(voltage > previousVoltage)           
  {
      while(voltage > previousVoltage)
      {
        previousVoltage = voltage;      
        comm = String("/2 move rel " + increment);
        rs232.println(comm);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }
    else if(voltage <= previousVoltage)
    {
      previousVoltage = voltage;
      pollUntilIdle(axisY);
      comm = String('-' + increment);
      comm = String("/1 move rel " + comm);
      rs232.println(comm);
      pollUntilIdle(axisY);
      rs232.println(comm);
      delay(dLay);
      voltage = readAnalog(pinMPPT, iter8); 
      while(voltage >= previousVoltage) 
      {
        previousVoltage = voltage;
        pollUntilIdle(axisY);
        comm = String('-' + increment);
        comm = String("/2 move rel " + comm);
        rs232.println(comm);
        delay(dLay);
        voltage = readAnalog(pinMPPT, iter8); 
      }
    }      
    
}
