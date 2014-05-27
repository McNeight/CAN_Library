/* CAN communications example
 
  Topic: Send messages using write(ID, length, data) function
  Author: Pedro Cevallos
  Created: 05/07/14
  Updated: 05/27/14
 
  Example shows how to send messages using MCP2515 CAN controller
  This example uses Serial Monitor to display received messages.
 
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
#define bitrate 250 // define CAN speed (bitrate)

 // data counter just to show a dynamic change in data messages
unsigned int counter1 = 0;
unsigned int counter2 = 0;

/* 
  Second we create CAN1 object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors. 
  Needs to be CAN0, CAN1, CAN2, or whatever name you want to give that channel. This can also allow us to create more channels 
  using more MCP2515s as long as we use different SPI CS to control data.
 */

MCP CAN1(10); //Create CAN Channel. SPI CS is pin 10 on this example

void setup(){

  Serial.begin(115200); // Initialize Serial communications with computer to use serial monitor

  //Set CAN mode and speed 

  CAN1.begin(mode, bitrate);
  
  delay(4000); // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...
  
  if ((CAN1.readMode () == mode) && (CAN1.readRate() == bitrate)) // Check to see if we set the Mode and speed correctly. For debugging purposes only.
  {
    Serial.println("CAN Initialization complete");
    Serial.print ("CAN speed set to:  ");
    Serial.print (bitrate);
    Serial.print ("kbit/s");
  }
  else
  {
    Serial.println("CAN Initialization failed");
    Serial.println("CAN speed failed");
  }
}

// Create a fuction to load and send an extended frame message

void extendedMessage(){

  unsigned long ID = 0x02DACBF1;                                      // Random Extended Message ID
  byte length = 8;                                                    // Data length
  byte data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, counter1}; // data message with an added counter
  
  CAN1.write(ID, extID, length, data);                                // Load message and send
  counter1++;                                                         // increase count
}

// Create a fuction to load and send a stadard frame message

void standardMessage(){

  unsigned long ID = 0x077;                                            // Random Standard Message ID
  byte length = 5;                                                     // Data length in this case let's say 5
  byte data[] = {0x01, 0x02, 0x03, 0x04, counter2};
  
  CAN1.write(ID, stdID, length, data);                                 // Load message and send
  counter2++;                                                          // increase count
 }


// Finally arduino loop to execute above functions with a 500ms delay

void loop(){

  standardMessage(); 
  extendedMessage();
  delay(500);

}

