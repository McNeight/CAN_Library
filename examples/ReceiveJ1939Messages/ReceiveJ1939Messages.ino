/* CAN communications example
 
  Topic: Receive Messages using read(J1939 *message) function
  Author: Pedro Cevallos
  Created: 05/07/14
 
  Example shows how to receive J1939 messages using MCP2515 CAN controller
  This example uses Serial Monitor to display received messages. 
  Note: Standard and remote frames are not supported in J1939 message structure. 
 
  As per wiki information:
  "CAN bus is a message-based protocol, designed specifically for automotive applications 
  but now also used in other areas such as aerospace, maritime, industrial automation and medical equipment."
  For more info http://en.wikipedia.org/wiki/Controller_area_network
  
  "SAE J1939 defines five layers in the seven-layer OSI network model, and this includes the Controller Area Network (CAN) 2.0b
  specification (using only the 29-bit/"extended" identifier) for the physical and data-link layers. Under J1939/11 and J1939/15, 
  the baud rate is specified as 250 kbit/s, with J1939/14 specifying 500 kbit/s." 
  For more info http://en.wikipedia.org/wiki/J1939
 
  Presentation regarding J1939 standard  http://www.simmasoftware.com/j1939-presentation.pdf
 
  CAN bit rate:  10 kpbs; 20 kpbs; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps
  CAN modes: NORMAL, LISTEN,SLEEP,CONFIG, LOOPBACK
 
 */

#include <can.h> 
#include <SPI.h> 


// First we define our CAN mode and rate. 

#define mode NORMAL // define CAN mode
#define bitrate 250 // define CAN speed (bitrate)

/* 
  Second we create CAN1 object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors. 
  Needs to be CAN0, CAN1, CAN2, or whatever name you want to give that channel. This can also allow us to create more channels 
  using more MCP2515s as long as we use different SPI CS to control data.
 */

MCP CAN1(10); //Create CAN Channel
J1939 message; // Create message object to use J1939 message structure 


void setup(){

  Serial.begin(115200); // Initialize Serial communications with computer to use serial monitor

  

  CAN1.begin(mode, bitrate);                                             //Set CAN mode and speed.
  
  delay(4000);                                                           // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...
  
 if ((CAN1.readMode () == mode) && (CAN1.readRate() == bitrate))         // Check to see if we set the Mode and speed correctly. For debugging purposes only.
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


// Create a function to read message and display it through Serial Monitor. 
// Note: standard frames are not supported in this message structure. 
// If a standard message is received it will display all zeros and no data.

void readMessage(){
 
 if (CAN1.msgAvailable() == true){           // Check to see if a valid message has been received. 

    CAN1.read(&message);                     //read message, it will follow the J1939 structure of ID,Priority, source address, destination address, DLC, PGN, 

    Serial.print("ID");
    Serial.print(" | ");
    Serial.print(message.ID,HEX);            //Display Identifier
    Serial.print(" | ");
    Serial.print("Priority");
    Serial.print(" | ");
    Serial.print(message.PRIO,HEX);          //Display Message priority
    Serial.print(" | ");
    Serial.print("Source Address");
    Serial.print(" | ");
    if(message.SA <0x10)                     //Adds a leading zero                   
      {
        Serial.print("0");
      }
    Serial.print(message.SA,HEX);            //Display Source Address
    Serial.print(" | ");
    Serial.print("Destination Address");
    Serial.print(" | ");
    if(message.DA <0x10)
      {
        Serial.print("0");
      }
    Serial.print(message.DA,HEX);            //Display Destination Address
     Serial.print(" | ");
    Serial.print("PGN");
    Serial.print(" | ");
    if(message.PGN <0x10)                     //Adds a leading zero    
      {
        Serial.print("0");
      }
    Serial.print(message.PGN,HEX);            //Display Parameter Group Number (PGN)
    Serial.print(" | ");
    Serial.print("DLC");
    Serial.print(" | ");
    Serial.print(message.DLC,HEX);            //Display data length code (DLC)
    Serial.print(" | ");
    Serial.print("Data");
    for (byte i=0;i<message.DLC;i++) {
      Serial.print(" | ");
      if(message.data[i] <0x10)
      {
        Serial.print("0");
      }
      Serial.print(message.data[i],HEX);      //Displays Data based on length
    }
    Serial.println();
  }
}
    

// Finally arduino loop to execute above function with a 100ms delay

void loop(){

  readMessage();
  delay(100);

}

