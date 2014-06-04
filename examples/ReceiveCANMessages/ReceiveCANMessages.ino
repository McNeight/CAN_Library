/* CAN communications example

  Topic: Receive Messages using read() function
  Author: Pedro Cevallos
  Created: 05/07/14

  Example shows how to receive messages using CAN
  This example uses Serial Monitor to display received messages

  As per wiki information:
  "CAN bus is a message-based protocol, designed specifically for automotive applications
  but now also used in other areas such as aerospace, maritime, industrial automation and medical equipment."
  For more info http://en.wikipedia.org/wiki/Controller_area_network

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

// First we define our CAN bitrate.
#define bitrate CAN_BPS_500K // define CAN speed (bitrate)

/*
  Second we create CANbus object (CAN channel) and select SPI CS Pin. Do not use "CAN" by itself as it will cause compile errors.
  Can't use CAN0 or CAN1 as variable names, as they are defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/sam3x8e.h
  Needs to be CANbus0, CANbus1, CANbus2, or whatever name you want to give that channel. This can also allow us to create more channels
  using more hardware devices.
 */

#if defined(ARDUINO_ARCH_AVR)
CAN_MCP2515 CANbus(10); // Create CAN channel using pin 10 for SPI chip select
#elif defined(ARDUINO_ARCH_SAM)
CAN_SAM3X8E CANbus(0);  // Create CAN channel on CAN bus 0
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

CAN_FRAME message; // Create message object to use CAN message structure

void setup()
{
  Serial.begin(115200);  // Initialize Serial communications with computer to use serial monitor

  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CANbus.begin(bitrate);

  delay(4000);  // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...

  // Output will be formatted as a CSV file, for capture and analysis
  Serial.println(F("millis(),ID,RTR,EID,Length,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7"));
}

// Create a function to read message and display it through Serial Monitor

void readMessage()
{
  if (CANbus.available() == true) // Check to see if a valid message has been received.
  {
    message = CANbus.read(); //read message, it will follow the CAN structure of ID,RTR, legnth, data. Allows both Extended or Standard

    Serial.print(millis());
    Serial.print(F(",0x"));
    Serial.print(message.id, HEX); //display message ID
    Serial.print(',');
    Serial.print(message.rtr); //display message RTR
    Serial.print(',');
    Serial.print(message.extended); //display message EID
    Serial.print(',');
    if (message.rtr == 1)
    {
      Serial.print(F(" REMOTE REQUEST MESSAGE ")); //technically if its RTR frame/message it will not have data So display this
    }
    else
    {
      Serial.print(message.length, HEX); //display message length
      for (byte i = 0; i < message.length; i++)
      {
        Serial.print(',');
        if (message.data[i] < 0x10) // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
        {
          Serial.print('0');
        }
        Serial.print(message.data[i], HEX); //display data based on length
      }
    }
    Serial.println(); // adds a line
  }
}

// Finally arduino loop to execute above function with a 50ms delay

void loop()
{
  readMessage();
  delay(50);
}

