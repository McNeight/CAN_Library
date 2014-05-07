/* CAN communications example
 
  Topic: Receive Messages using read(CANopen *message) function
  Author: Pedro Cevallos
  Created: 05/07/14
 
  This example does not cover the actual CANopen network management, object dictionary or application. 
  it only shows how to receive CANopen messages using MCP2515 CAN controller.
  This example uses Serial Monitor to display received messages. 
  
 
  As per wiki information:
 "CANopen is a communication protocol and device profile specification for embedded systems used in automation. 
  In terms of the OSI model, CANopen implements the layers above and including the network layer. The CANopen 
  standard consists of an addressing scheme, several small communication protocols and an application layer 
  defined by a device profile. The communication protocols have support for network management, device monitoring 
  and communication between nodes, including a simple transport layer for message segmentation/desegmentation. 
  The lower level protocol implementing the data link and physical layers is usually Controller Area Network (CAN)"
  for more info http://en.wikipedia.org/wiki/CANopen
  
    
  CAN bit rate:  10 kpbs; 20 kpbs; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps
  CAN modes: NORMAL, LISTEN,SLEEP,CONFIG, LOOPBACK
 
 */

#include <can.h> 
#include <SPI.h> 


// First we define our CAN mode and rate. 

#define mode NORMAL // define CAN mode
#define bitrate 1000 // define CAN speed (bitrate)

/* 
  Second we create CAN1 object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors. 
  Needs to be CAN0, CAN1, CAN2, or whatever name you want to give that channel. This can also allow us to create more channels 
  using more MCP2515s as long as we use different SPI CS to control data.
 */

MCP CAN1(10); //Create CAN Channel
CANopen message; // Create message object to use CANopen message structure 


void setup(){

  Serial.begin(115200);                                          // Initialize Serial communications with computer to use serial monitor

  //Set CAN mode and speed. Note: Speed is now 1Mbits/s so adjust your CAN monitor

  CAN1.begin(mode, bitrate);
  
  delay(4000);                                                   // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...
  
 if ((CAN1.readMode () == mode) && (CAN1.readRate() == bitrate)) // Check to see if we set the Mode and speed correctly. For debugging purposes only.
  {
    Serial.println("CAN Initialization complete");
    Serial.print ("CAN speed set to:  ");
    Serial.print (bitrate);
    Serial.println ("kbit/s");
  }
  else
  {
    Serial.println("CAN Initialization failed");
    Serial.println("CAN speed failed");
  }
}


// Create a function to read message and display it through Serial Monitor

void readMessage(){
 
if (CAN1.msgAvailable() == true){                               // Check to see if a valid message has been received.

    CAN1.read(&message);                                        //read message, it will follow the CANopen structure of COB_ID,Function code (FC), Node, RTR, length, data.
    Serial.print("COB_ID");
    Serial.print(" | ");
    Serial.print(message.COB_ID,HEX);                           //display Communication object identifier 
    Serial.print(" | ");
    Serial.print("Function Code");
    Serial.print(" | ");
    Serial.print(message.FC,HEX);                               //display Function code
    Serial.print(" | ");
    Serial.print("Node");
    Serial.print(" | ");
    Serial.print(message.NODE,HEX);
    Serial.print(" | ");
    Serial.print("RTR");  
   if (message.rtr == 1){
     Serial.print(" REMOTE REQUEST MESSAGE ");                  //technically if its RTR frame/message it will not have data So display this
   }else {
    Serial.print(" | ");
    Serial.print("Length");
    Serial.print(" | ");
    Serial.print(message.length,HEX);                            //display message length
    Serial.print(" | ");
    Serial.print("Data");
    for (byte i=0;i<message.length;i++) {
      Serial.print(" | ");
      if(message.data[i] <0x10)
      {
        Serial.print("0");
      }
      Serial.print(message.data[i],HEX);                          //display data based on length
    }
   }
    Serial.println();
}   
}
    

// Finally arduino loop to execute above function with a 150ms delay

void loop(){

  readMessage();
  delay(150);

}

