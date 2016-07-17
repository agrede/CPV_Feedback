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

int pinMPPT = 1;   //Analog pin used to read voltage across MPPT load resistor
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

// Define resolution (0.000047625 for X-LRM200A-E03)
float mmResolution = 0.000047625; 
float umResolution = 0.047625; 

//Period of feedback iterations
const int interval = 10000;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 11;
int TXpin = 4;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX
 

void setup() 
{
  rs232.begin(115200);
  delay(2000);
  rs232.println("/ renumber");
  delay(1000);
  requestCommand(axisX, Home, " ");
  requestCommand(axisY, Home, " ");
  pollUntilIdle(axisX);
  requestCommand(axisX, SetMaxspeed, "200000");  // Maxspeed = (0.02906799 * <value>) μm/sec
  pollUntilIdle(axisY;
  requestCommand(axisY, SetMaxspeed, "200000");  // Maxspeed = (0.02906799 * <value>) μm/sec
}

void loop() 
{  
  currentMillis = millis();
  if(currentMillis - previousMillis >= interval)
  {   
    previousMillis = currentMillis;
    optimize(um(250), mm(10));
    optimize(um(5), um(250));
  }
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

void optimize(String increment, int travelMax)
{ 
  //Get starting conditions before optimizing  
  voltage = analogRead(pinMPPT);              //read voltage on MPPT load 
  /*
  pollUntilIdle(axisX);
  response = requestCommand(axisX, GetPos, " ");      //get initial x and y positions
  posXstart = response.responseData;
  pollUntilIdle(axisY);
  response = requestCommand(axisY, GetPos, " ");
  posYstart = response.responseData;
  */

  //Move one increment in +x and get new voltage and position
  pollUntilIdle(axisX);
  requestCommand(axisX, MoveRel, increment);  //move X axis stage +increment
  /*pollUntilIdle(axisX);
  response = requestCommand(axisX, GetPos, " ");
  posX = response.responseData; */
  previousVoltage = voltage;
  voltage = analogRead(pinMPPT);

  //Start optimizing along X axis
  if(voltage > previousVoltage)         
  {
    while((voltage >= previousVoltage) && (abs(posX - posXstart) <= travelMax))
    {
      previousVoltage = voltage;
      pollUntilIdle(axisX);
      requestCommand(axisX, MoveRel, increment);
      pollUntilIdle(axisX);
      response = requestCommand(axisX, GetPos, " ");
      posX = response.responseData;
      voltage = analogRead(pinMPPT);
    }
    pollUntilIdle(axisX);
    requestCommand(axisX, MoveRel, String('-' + increment));
  }
  else if(voltage < previousVoltage)
  {
    while ((voltage <= previousVoltage) && (abs(posX - posXstart) <= travelMax))
    {
      previousVoltage = voltage;
      pollUntilIdle(axisX);
      requestCommand(axisX, MoveRel, String('-' + increment));
      pollUntilIdle(axisX);
      response = requestCommand(axisX, GetPos, " ");
      posX = response.responseData;
      voltage = analogRead(pinMPPT);
    }
    pollUntilIdle(axisX);
    requestCommand(axisX, MoveRel, increment);
  }

  //Start optimizing along Y axis
  voltage = analogRead(pinMPPT);
  previousVoltage = voltage;
  pollUntilIdle(axisY);
  requestCommand(axisY, MoveRel, increment);
  pollUntilIdle(axisY);
  response = requestCommand(axisY, GetPos, " ");
  posY = response.responseData;
  voltage = analogRead(pinMPPT);
  
  if(voltage > previousVoltage)           
  {
    while((voltage >= previousVoltage) && (abs(posY - posYstart) <= travelMax))
    {
      previousVoltage = voltage;
      pollUntilIdle(axisY);
      requestCommand(axisY, MoveRel, increment);
      pollUntilIdle(axisY);
      response = requestCommand(axisY, GetPos, " ");
      posY = response.responseData;
      voltage = analogRead(pinMPPT);
    }
    pollUntilIdle(axisY);
    requestCommand(axisY, MoveRel, String('-' + increment));
  }
  else if(voltage < previousVoltage)
  {
    while ((voltage <= previousVoltage) && (abs(posY - posYstart) <= travelMax))
    {
      previousVoltage = voltage;
      pollUntilIdle(axisY);
      requestCommand(axisY, MoveRel, String('-' + increment));
      pollUntilIdle(axisY);
      response = requestCommand(axisY, GetPos, " ");
      posY = response.responseData;
      voltage = analogRead(pinMPPT);
    }
    pollUntilIdle(axisY);
    requestCommand(axisY, MoveRel, increment);
  }    
}
