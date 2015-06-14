/* CAN communications example

  Topic: Send messages using write(ID, frameType, length, data) function
  Authors: Pedro Cevallos & Neil McNeight
  Created: 05/07/14
  Updated: 06/14/15

  Example shows how to send messages using CAN
  This example uses Serial Monitor to display received messages.

  As per wiki information:
  "CAN bus is a message-based protocol, designed specifically for automotive
  applications but now also used in other areas such as aerospace, maritime,
  industrial automation and medical equipment."

  For more info http://en.wikipedia.org/wiki/Controller_area_network

 */

#include <CAN.h>
#include <SPI.h> // required to resolve #define conflicts

// Define our CAN speed (bitrate).
#define bitrate CAN_BPS_500K

// data counter just to show a dynamic change in data messages
uint32_t extended_counter = 0;
uint32_t standard_counter = 0;

void setup()
{
  Serial.begin(115200); // Initialize Serial communications with computer to use serial monitor

  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CAN.begin(bitrate);

  delay(4000); // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...

  // Output will be formatted as a CSV file, for capture and analysis
  Serial.println(F("millis(),standard_counter,extended_counter"));
}


// Create a function to load and send an extended frame message
void extendedMessage()
{
  uint32_t timehack = millis();
  unsigned long ID = 0x02DACBF1; // Random Extended Message ID
  byte length = 8; // Data length
  // counter in the first 4 bytes, timing in the last 4 bytes
  byte data[] = { (extended_counter >> 24), (extended_counter >> 16), (extended_counter >> 8), (extended_counter & 0xF), \
                  (timehack >> 24), (timehack >> 16), (timehack >> 8), (timehack & 0xF)
                }; // data message with an added counter

  CAN.write(ID, CAN_EXTENDED_FRAME, length, data); // Load message and send
  extended_counter++; // increase count
}


// Create a function to load and send a standard frame message
void standardMessage()
{
  uint32_t timehack = millis();
  unsigned long ID = 0x555; // Random Standard Message ID
  byte length = 8; // Data length
  // counter in the first 4 bytes, timing in the last 4 bytes
  byte data[] = { (standard_counter >> 24), (standard_counter >> 16), (standard_counter >> 8), (standard_counter & 0xF), \
                  (timehack >> 24), (timehack >> 16), (timehack >> 8), (timehack & 0xF)
                }; // data message with an added counter

  CAN.write(ID, CAN_STANDARD_FRAME, length, data); // Load message and send
  standard_counter++; // increase count
}


// Finally arduino loop to execute above functions with a 250ms delay
void loop()
{
  standardMessage();
  delay(250);
  extendedMessage();
  delay(250);
  Serial.print(millis());
  Serial.print(',');
  Serial.print(standard_counter);
  Serial.print(',');
  Serial.println(extended_counter); // adds a line
}
