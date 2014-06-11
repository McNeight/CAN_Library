/* CAN communications example

  Topic: Send messages using write(ID, length, data) function
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
#elif defined(__MK20DX256__) // Teensy 3.1
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

// First we define our CAN bitrate.
#define bitrate CAN_BPS_500K // define CAN speed (bitrate)

// data counter just to show a dynamic change in data messages
uint8_t extended_counter = 0;
uint8_t standard_counter = 0;

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
  unsigned long ID = 0x02DACBF1; // Random Extended Message ID
  byte length = 8; // Data length
  byte data[] = {0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, extended_counter}; // data message with an added counter

  CANbus.write(ID, CAN_EXTENDED_FRAME, length, data); // Load message and send
  extended_counter++; // increase count
}

// Create a function to load and send a standard frame message

void standardMessage()
{
  unsigned long ID = 0x555; // Random Standard Message ID
  byte length = 5; // Data length in this case let's say 5
  byte data[] = {0x55, 0xAA, 0x55, 0xAA, standard_counter}; // data message with an added counter

  CANbus.write(ID, CAN_BASE_FRAME, length, data); // Load message and send
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
