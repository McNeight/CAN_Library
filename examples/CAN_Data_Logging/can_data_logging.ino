//Libraries
#include <SD.h>
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <Wire.h>

//Defines
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   A5
#define rpmPin 3

//Variables
File data;
String output;
int string[11];
tCAN message;
tCAN lastMessage;
bool toggle = false;
int i = 0;
int halfRPM;
int rpm;
int time = 0;
int timeold = 0;
int elapsed = 0;
int start = 0;

void setup()
{
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(LEFT,INPUT);
  pinMode(RIGHT,INPUT);
  pinMode(CLICK,INPUT);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  digitalWrite(CLICK, HIGH);

  pinMode(rpmPin,OUTPUT);

  Serial.begin(115200);

  if(Canbus.init(CANSPEED_250))
  {
    Serial.println("CAN Init ok");
  } 
  else
  {
    Serial.println("Can't init CAN");
  } 

  pinMode(10, OUTPUT);
  if (!SD.begin(9)) 
  {
    Serial.println("Card failed, or not present");
    return;
  }

  if(SD.exists("data.asc"))
  {
    SD.remove("data.asc");
  }
  data = SD.open("data.asc",FILE_WRITE);
  data.println("ID -> DATA");
  Serial.println("Click joystick to begin logging; right joystick to stop");

  message.id = 0;
  lastMessage.id = 0;
}

void loop()
{
  if(toggle)
  {
    if (mcp2515_get_message(&message))
    {
      if(message.id != lastMessage.id)
      {
        //timeold = millis();
        create_string();
        write_string();
        //time = millis();
        //elapsed = time-timeold;
        //Serial.println(elapsed);
        setPins();
        lastMessage = message;
      }
    }
  }
  if(!digitalRead(CLICK) && !toggle)
  {
    toggle = true;
    start = millis();
  }
  if(!digitalRead(RIGHT) && toggle)
  {
    toggle = false;
    data.close();
    Serial.println("File closed");
    while(1)
    {
    }
  }
}

void create_string()
{
  /*
  output = ""+String(message.id, HEX);
   output += "            Rx   ";
   output += " "+String(message.header.length);
   for(int i = 0; i<8; i++)
   {
   output += " "+String(message.data[i], HEX);
   }
   */

  string[0] = message.id;
  for(i=1;i<9;i++)
  {
    string[i] = message.data[i-1];
  }
}

void write_string()
{  
  for(i=0;i<10;i++)
  {
    data.print(string[i],HEX);
    data.print(" ");
  }
  data.print(millis()-start);
  data.println();  
}

void setPins()
{
  if(message.id == 0x181)
  {
    halfRPM = message.data[5];
    halfRPM = halfRPM << 8;
    rpm = halfRPM | message.data[4];
    rpm = (rpm)*(17/500);
    //Serial.println(rpm);
    analogWrite(rpmPin, rpm);
  }
}







