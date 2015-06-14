/* CAN communications example

  Topic: Send messages using write() function
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
  CAN_Frame extended_message; // Create message object to use CAN message structure

  // There are at least two ways to put data into the message; memcpy() and individual arrays
  uint8_t message_data[8] = { 0 };

  extended_message.id = 0x02DACBF1; // Random Extended Message ID
  extended_message.valid = true;
  extended_message.rtr = 0;
  extended_message.extended = CAN_EXTENDED_FRAME;
  extended_message.length = 8; // Data length

  // counter in the first 4 bytes, timing in the last 4 bytes
  uint32_t timehack = millis();

  // Load the data into a local array and then use memcpy() to transfer to CAN_Frame
  message_data[0] = (extended_counter >> 24);
  message_data[1] = (extended_counter >> 16);
  message_data[2] = (extended_counter >> 8);
  message_data[3] = (extended_counter & 0xF);
  message_data[4] = (timehack >> 24);
  message_data[5] = (timehack >> 16);
  message_data[6] = (timehack >> 8);
  message_data[7] = (timehack & 0xF);
  memcpy(extended_message.data, message_data, sizeof(extended_message.data));

  CAN.write(extended_message); // Load message and send
  extended_counter++; // increase count
}


// Create a function to load and send a standard frame message
void standardMessage()
{
  CAN_Frame standard_message; // Create message object to use CAN message structure

  standard_message.id = 0x555; // Random Standard Message ID
  standard_message.valid = true;
  standard_message.rtr = 0;
  standard_message.extended = CAN_STANDARD_FRAME;
  standard_message.length = 8; // Data length

  // counter in the first 4 bytes, timing in the last 4 bytes
  uint32_t timehack = millis();

  // Load the data directly into CAN_Frame
  standard_message.data[0] = (standard_counter >> 24);
  standard_message.data[1] = (standard_counter >> 16);
  standard_message.data[2] = (standard_counter >> 8);
  standard_message.data[3] = (standard_counter & 0xF);
  standard_message.data[4] = (timehack >> 24);
  standard_message.data[5] = (timehack >> 16);
  standard_message.data[6] = (timehack >> 8);
  standard_message.data[7] = (timehack & 0xF);

  CAN.write(standard_message); // Load message and send
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
