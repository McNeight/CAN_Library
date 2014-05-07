/* CAN communications example
 
  Topic: Receive Messages using read(CAN *message) function
  Author: Pedro Cevallos
  Created: 05/07/14
 
  Example shows how to receive messages using MCP2515 CAN controller
  This example uses Serial Monitor to display received messages
 
  As per wiki information:
  "CAN bus is a message-based protocol, designed specifically for automotive applications 
  but now also used in other areas such as aerospace, maritime, industrial automation and medical equipment."
  For more info http://en.wikipedia.org/wiki/Controller_area_network
 
  CAN bit rate:  10 kpbs; 20 kpbs; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps
  CAN modes: NORMAL, LISTEN,SLEEP,CONFIG, LOOPBACK
 
 */

#include <can.h> 
#include <SPI.h> 


// First we define our CAN mode and rate. 

#define mode NORMAL // define CAN mode
#define bitrate 500 // define CAN speed (bitrate)

/* 
  Second we create CAN1 object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors. 
  Needs to be CAN0, CAN1, CAN2, or whatever name you want to give that channel. This can also allow us to create more channels 
  using more MCP2515s as long as we use different SPI CS to control data.
 */

MCP CAN1(10); //Create CAN Channel
CAN message; // Create message object to use CAN message structure 


void setup(){

  Serial.begin(115200);                                          // Initialize Serial communications with computer to use serial monitor

  //Set CAN mode and speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

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

    CAN1.read(&message);                                        //read message, it will follow the CAN structure of ID,RTR, legnth, data. Allows both Extended or Standard

    Serial.print("ID");
    Serial.print(" | ");
    Serial.print(message.ID,HEX);                               //display message ID
    Serial.print(" | ");
    Serial.print("RTR");
    Serial.print(" | ");
    Serial.print(message.rtr,HEX);                              //display message RTR
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

