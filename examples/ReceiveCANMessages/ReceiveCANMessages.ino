/* CAN communications example

  Topic: Receive Messages using read() function
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
  Serial.println(F("millis(),ID,RTR,EID,Length,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7"));
}


// Create a function to read message and display it through Serial Monitor
void readMessage()
{
  CAN_Frame message; // Create message object to use CAN message structure

  Serial.print(millis());

  if (CAN.available() == true) // Check to see if a valid message has been received.
  {
    message = CAN.read(); //read message, it will follow the CAN structure of ID,RTR, legnth, data. Allows both Extended or Standard

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
  }
  Serial.println(); // adds a line
}


// Finally arduino loop to execute above function with a 50ms delay
void loop()
{
  readMessage();
  delay(50);
}
