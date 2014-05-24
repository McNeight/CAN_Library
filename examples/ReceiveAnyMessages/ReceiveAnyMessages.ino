/* CAN communications example
 
  Topic: Receive Messages using read(ID, length, data) function
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

void setup(){

  Serial.begin(115200);                                            // Initialize Serial communications with computer to use serial monitor

  //Set CAN mode and speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CAN1.begin(mode, bitrate);
  
  delay(4000);                                                    // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...
  
 if ((CAN1.readMode () == mode) && (CAN1.readRate() == bitrate))  // Check to see if we set the Mode and speed correctly. For debugging purposes only.
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

  unsigned long ID;                                       // assign a variable for Message ID
  byte length;                                            //assign a variable for length
  byte data[8];                                           //assign an array for data
  
  if (CAN1.msgAvailable() == true){                       // Check to see if a valid message has been received.
    
    CAN1.read(&ID, &length, data);                        // read Message and assign data through reference operator &
    
    Serial.print("ID");
    Serial.print(" | ");
    Serial.print(ID,HEX);                                 // Displays received ID
    Serial.print(" | ");
    Serial.print("Data Length");
    Serial.print(" | ");
    Serial.print( length,HEX);                            // Displays message length
    Serial.print(" | ");
    Serial.print("Data");
    for (byte i=0;i<length;i++) {
      Serial.print(" | ");
      if(data[i] <0x10)                                   // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
      {
      Serial.print("0");
      }
      Serial.print(data[i],HEX);                          // Displays message data
      
    }
    Serial.println();                                     // adds a line
  }
}
    

// Finally arduino loop to execute above function with a 100ms delay

void loop(){

  readMessage();
  delay(100);

}

