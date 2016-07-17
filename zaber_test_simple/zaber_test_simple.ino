#include <SoftwareSerial.h>

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

long volTemp = 0;
int iter8 = 100;

//On Mega, RX must be one of the following: pin 10-15, 50-53, A8-A15
int RXpin = 11;
int TXpin = 4;

SoftwareSerial rs232(RXpin, TXpin);   //RX, TX

String dist = "200000";
String comm;

void setup() 
{
  Serial.begin(9600);
  rs232.begin(115200);
  delay(2000);
  /*rs232.println("/renumber");
  delay(500);
  Serial.println(rs232.readStringUntil('\n'));
  delay(100);
  rs232.println("/home");  
  delay(15000);
  rs232.println("/1 move abs 3880000");
  rs232.println("/2 move abs 1200000");
  delay(5000);  */
}

void loop() 
{    
  /*rs232.println("/1 get pos");
  Serial.println(rs232.readStringUntil('\n'));
  delay(1000);
  rs232.println("/2 get pos");
  Serial.println(rs232.readStringUntil('\n')); */
  /*
  
  volTemp = 0;
  for(int i = 0; i < iter8; i++)
  {
    volTemp += analogRead(pinMPPT);
  }
  voltage = volTemp/iter8;
  Serial.println(voltage); 
  */
  Serial.print(analogRead(pinMPPT));
  Serial.print(", ");
  Serial.println(millis());      
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



