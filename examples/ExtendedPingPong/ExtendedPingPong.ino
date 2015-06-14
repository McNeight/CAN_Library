/* CAN extended Frames with Ping/Pong sending

  This example sets up a receive and transmit mailbox on both CAN devices.
  First NODE0 sends to NODE1. When NODE1 receives it sends to NODE0. PING/PONGs forever
  and as quickly as possible - This will saturate the bus so don't have anything important connected.

  Authors: Thibaut Viard, Wilfredo Molina, Collin Kidder, Pedro Cevallos & Neil McNeight
  Created: 06/01/14
  Updated: 06/14/15

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

//Comment out for one node, and leave uncommented for the other node
#define NODE1

#ifdef NODE1
#define NODETX_CAN_ID    0x15555555
#define NODERX_CAN_ID    0x0AAAAAAA
#else
#define NODETX_CAN_ID    0x0AAAAAAA
#define NODERX_CAN_ID    0x15555555
#endif

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

uint32_t sentFrames, receivedFrames;

CAN_Frame frameTX, frameRX, incoming;

void setup()
{
  Serial.begin(115200);  // Initialize Serial communications with computer to use serial monitor

  //Set CAN speed. Note: Speed is now 500kbit/s so adjust your CAN monitor

  CAN.begin(bitrate);

  delay(4000);  // Delay added just so we can have time to open up Serial Monitor and CAN bus monitor. It can be removed later...

  // Output will be formatted as a CSV file, for capture and analysis
  Serial.println(F("millis(),ID,RTR,EID,Length,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7"));

  // Initialize frame counters
  sentFrames = 0;
  receivedFrames = 0;

  //Initialize the definitions for the frames we'll be sending.
  //This can be done here because the frame never changes
  frameTX.id = NODETX_CAN_ID;
  frameTX.length = MAX_CAN_FRAME_DATA_LEN;
  //We are using extended frames so mark that here. Otherwise it will just use
  //the first 11 bits of the ID set
  frameTX.extended = 1;

  // Process
  processTXFrame();
  // Send out the first frame
  CAN.write(frameTX);
}

// Test rapid fire ping/pong of extended frames
void loop()
{
  if (CAN.available())
  {
    frameRX = CAN.read();
    receivedFrames++;
    // Process
    processRXFrame();

    // Only send frames after receiving one, otherwise bus is flooded
    processTXFrame();
    CAN.write(frameTX);
  }
}

void processTXFrame()
{
  sentFrames++;
  frameTX.data[3] = sentFrames;
  frameTX.data[2] = sentFrames >> 8;
  frameTX.data[1] = sentFrames >> 16;
  frameTX.data[0] = sentFrames >> 24;

  frameTX.data[7] = receivedFrames;
  frameTX.data[6] = receivedFrames >> 8;
  frameTX.data[5] = receivedFrames >> 16;
  frameTX.data[4] = receivedFrames >> 24;
}

void processRXFrame()
{
  if (frameRX.id == NODERX_CAN_ID)
  {
    uint32_t otherNodeSent = (uint32_t)frameRX.data[3];
    otherNodeSent |= ((uint32_t)frameRX.data[2] << 8);
    otherNodeSent |= ((uint32_t)frameRX.data[1] << 16);
    otherNodeSent |= ((uint32_t)frameRX.data[0] << 24);

    uint32_t otherNodeReceived = (uint32_t)frameRX.data[7];
    otherNodeReceived |= ((uint32_t)frameRX.data[6] << 8);
    otherNodeReceived |= ((uint32_t)frameRX.data[5] << 16);
    otherNodeReceived |= ((uint32_t)frameRX.data[4] << 24);

    //    Serial.print("I am node 0x");
    //    Serial.println(NODETX_CAN_ID, HEX);
    if ((sentFrames % 5000) == 0)
    {
      Serial.print("IS:");
      Serial.print(sentFrames);
      Serial.print(" OR:");
      Serial.println(otherNodeReceived);
      Serial.print("IR:");
      Serial.print(receivedFrames);
      Serial.print(" OS:");
      Serial.println(otherNodeSent);
    }

    int16_t txDropped = sentFrames - otherNodeReceived;
    if (txDropped < 0)
    {
      Serial.print("Node 0x");
      Serial.print(NODERX_CAN_ID, HEX);
      Serial.print(" received ");
      Serial.print(abs(txDropped));
      Serial.println(" more frames than I transmitted.");
    }
    else if (txDropped > 0)
    {
      Serial.print(txDropped);
      Serial.print(" frames dropped from here to node 0x");
      Serial.println(NODERX_CAN_ID, HEX);
    }

    int16_t rxDropped = receivedFrames - otherNodeSent;
    if (rxDropped < 0)
    {
      Serial.print("I received ");
      Serial.print(abs(rxDropped));
      Serial.print(" more frames than node 0x");
      Serial.print(NODERX_CAN_ID, HEX);
      Serial.println(" transmitted.");
    }
    else if (rxDropped > 0)
    {
      Serial.print(rxDropped);
      Serial.print(" frames dropped from node 0x");
      Serial.print(NODERX_CAN_ID, HEX);
      Serial.println(" to here.");
    }

    // Synchronize the counters
    sentFrames = otherNodeReceived;
    receivedFrames = otherNodeSent;
  }
}
