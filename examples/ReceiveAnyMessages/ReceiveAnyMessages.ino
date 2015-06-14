/* CAN communications example

  Topic: Receive Messages using read(ID, length, data) function
  Authors: Pedro Cevallos & Neil McNeight
  Created: 05/07/14
  Updated: 06/14/15

  Example shows how to receive messages using CAN
  This example uses Serial Monitor to display received messages

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

void setup()
{
  Serial.begin(115200);  // Initialize Serial communications with computer to use serial monitor

  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CAN.begin(bitrate);

  delay(4000);  // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...

  // Output will be formatted as a CSV file, for capture and analysis
  Serial.println(F("millis(),ID,Length,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7"));
}


// Create a function to read message and display it through Serial Monitor
void readMessage()
{
  unsigned long can_ID; // assign a variable for Message ID
  byte can_length; //assign a variable for length
  byte can_data[8] = {0}; //assign an array for data

  if (CAN.available() == true) // Check to see if a valid message has been received.
  {
    CAN.read(&can_ID, &can_length, can_data); // read Message and assign data through reference operator &

    Serial.print(millis());
    Serial.print(F(",0x"));
    Serial.print(can_ID, HEX); // Displays received ID
    Serial.print(',');
    Serial.print(can_length, HEX); // Displays message length
    for (byte i = 0; i < can_length; i++)
    {
      Serial.print(',');
      if (can_data[i] < 0x10) // If the data is less than 10 hex it will assign a zero to the front as leading zeros are ignored...
      {
        Serial.print('0');
      }
      Serial.print(can_data[i], HEX); // Displays message data
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
