/* CAN communications example

  Topic: Send messages using write() function
  Author: Pedro Cevallos
  Created: 05/07/14
  Updated: 05/27/14

  Example shows how to send messages using CAN
  This example uses Serial Monitor to display received messages.

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

// data counter just to show a dynamic change in data messages
uint32_t extended_counter = 0;
uint16_t standard_counter = 0;

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

void setup()
{
  Serial.begin(115200); // Initialize Serial communications with computer to use serial monitor

  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CANbus.begin(bitrate);

  delay(4000); // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...

  // Output will be formatted as a CSV file, for capture and analysis
  Serial.println(F("millis(),standard_counter,extended_counter"));
}

// Create a function to load and send an extended frame message

void extendedMessage()
{
  CAN_FRAME extended_message; // Create message object to use CAN message structure

  // There are at least two ways to put data into the message; memcpy() and individual arrays
  uint8_t message_data[8] = { 0 }; // data message with an added counter

  extended_message.id = extended_counter + 0x800; // Random Extended Message ID
  extended_message.valid = true;
  extended_message.rtr = 0;
  extended_message.extended = CAN_EXTENDED_FRAME;
  // Serial.print("extended_message.extended:");
  // Serial.println(extended_message.extended);

  extended_message.length = 8; // Data length
  uint32_t timehack = millis();
  message_data[0] = 0x55;
  message_data[4] = (timehack >> 24);
  message_data[5] = (timehack >> 16);
  message_data[6] = (timehack >> 8);
  message_data[7] = (timehack & 0xF);
  memcpy(extended_message.data, message_data, sizeof(extended_message.data));

  CANbus.write(extended_message); // Load message and send
  extended_counter++; // increase count
}

// Create a function to load and send a standard frame message

void standardMessage()
{
  CAN_FRAME standard_message; // Create message object to use CAN message structure

  standard_message.id = standard_counter; // Random Standard Message ID
  standard_message.valid = true;
  standard_message.rtr = 0;
  standard_message.extended = CAN_BASE_FRAME;
  //Serial.print("standard_message.extended:");
  //Serial.println(standard_message.extended);

  standard_message.length = 5; // Data length in this case let's say 5
  uint32_t timehack = millis();
  // data message with an added counter
  standard_message.data[0] = 0x55;
  standard_message.data[1] = (timehack >> 24);
  standard_message.data[2] = (timehack >> 16);
  standard_message.data[3] = (timehack >> 8);
  standard_message.data[4] = (timehack & 0xF);

  CANbus.write(standard_message); // Load message and send
  standard_counter++; // increase count
}


// Finally arduino loop to execute above functions with a 250ms delay

void loop()
{
  standardMessage();
  extendedMessage();
  delay(250);
  Serial.print(millis());
  Serial.print(',');
  Serial.print(standard_counter);
  Serial.print(',');
  Serial.println(extended_counter); // adds a line
}
