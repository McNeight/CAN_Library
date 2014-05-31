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

#include <Arduino.h>
#include "CAN.h"
#if defined(ARDUINO_ARCH_AVR)
#include <SPI.h> 
#include "CAN_MCP2515.h"
#elif defined(ARDUINO_ARCH_SAM)
#include <variant.h>
#include "CAN_SAM3X8E.h"
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

// First we define our CAN mode and rate. 

#define mode MCP2515_MODE_NORMAL // define CAN mode
#define bitrate CAN_BPS_500K // define CAN speed (bitrate)

/* 
  Second we create CANbus object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors.
  Can't use CAN0 or CAN1 as variable names, as they are defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/sam3x8e.h
  Needs to be CANbus0, CANbus1, CANbus2, or whatever name you want to give that channel. This can also allow us to create more channels
  using more MCP2515s as long as we use different SPI CS to control data.
 */

#if defined(ARDUINO_ARCH_AVR)
// Can't use CAN0 or CAN1 as variable names, as they are defined in
CAN_MCP2515 CANbus(10); // Create CAN channel using pin 10 for SPI chip select
#elif defined(ARDUINO_ARCH_SAM)
// Can't use CAN0 or CAN1 as variable names, as they are defined in
CAN_SAM3X8E CANbus(0);  // Create CAN channel on CAN bus 0
//CAN1.init(SystemCoreClock, CAN_BPS_500K);
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

//CAN_FRAME testFrame;

void setup() {

  Serial.begin(115200);                                            // Initialize Serial communications with computer to use serial monitor

  //Set CAN mode and speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CANbus.begin(bitrate);
  
  delay(4000);                                                    // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...
  
  if ((CANbus.readMode () == MCP2515_MODE_NORMAL) && (CANbus.readRate() == bitrate))  // Check to see if we set the Mode and speed correctly. For debugging purposes only.
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

void readMessage() {

  unsigned long can_ID;                                       // assign a variable for Message ID
  byte can_length;                                            //assign a variable for length
  byte can_data[8];                                           //assign an array for data
  
  if (CANbus.available() == true) {                      // Check to see if a valid message has been received.
    
    CANbus.read(&can_ID, &can_length, can_data);                        // read Message and assign data through reference operator &
    
    if (can_ID == 0x7E8)
    {
      Serial.print("Time | ");
      Serial.print(millis());
      Serial.print(" | ID");
    Serial.print(" | ");
      Serial.print(can_ID, HEX);                                // Displays received ID
    Serial.print(" | ");
    Serial.print("Data Length");
    Serial.print(" | ");
      Serial.print(can_length, HEX);                           // Displays message length
    Serial.print(" | ");
    Serial.print("Data");
      for (byte i = 0; i < can_length; i++) {
      Serial.print(" | ");
        if (can_data[i] < 0x10)                                 // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
      {
      Serial.print("0");
      }
        Serial.print(can_data[i], HEX);                         // Displays message data
      
    }
    Serial.println();                                     // adds a line
  }
  }
}
    
void printFrame(CAN_FRAME &frame) {
   Serial.print("ID: 0x");
//   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
//       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

// Finally arduino loop to execute above function with a 100ms delay

void loop() {

  byte J1939_data[] = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};
  CANbus.write(0x7DF, CAN_BASE_FRAME, 8, J1939_data);
  readMessage();
  delay(50);

}

