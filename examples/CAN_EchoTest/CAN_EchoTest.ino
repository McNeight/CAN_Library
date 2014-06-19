// Arduino Due - CAN Sample 1
// Brief CAN example for Arduino Due
// Test the transmission from CAN0 Mailbox 0 to CAN1 Mailbox 0
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013

// Required libraries
#include <Arduino.h>
#include "CAN.h"
#if defined(ARDUINO_ARCH_AVR)
#include <SPI.h>
#include "CAN_MCP2515.h"
#elif defined(ARDUINO_ARCH_SAM)
#include <variant.h>
#include "CAN_SAM3X.h"
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x07
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x55AAEE22

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;

#if defined(ARDUINO_ARCH_AVR)
// Can't use CAN0 or CAN1 as variable names, as they are defined in
CAN_MCP2515 CANbus0(10); // Create CAN channel using pin 10 for SPI chip select
CAN_MCP2515 CANbus1(9);  // Create CAN channel using pin 9 for SPI chip select
#elif defined(ARDUINO_ARCH_SAM)
// Can't use CAN0 or CAN1 as variable names, as they are defined in
CAN_SAM3X CANbus0(0);  // Create CAN channel on CAN bus 0
CAN_SAM3X CANbus1(1);  // Create CAN channel on CAN bus 1
#else
#error “This library only supports boards with an AVR or SAM processor.”
#endif

//Leave defined if you use native port, comment if using programming port
//#define Serial SerialUSB

void setup()
{
  CAN_FRAME output;

  // start serial port at 115200 bps:
  Serial.begin(115200);
  Serial.println("Type CAN message to send");
  while (Serial.available() == 0);
}
void loop() {

  CAN_FRAME output;
  while (Serial.available() > 0) {
    CAN_MSG_1 = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print("Sent value= ");
      Serial.println(CAN_MSG_1);
    }
  }

  // Initialize CAN0 and CAN1, baudrate is 250kb/s
  //CAN.init(CAN_BPS_250K);
  //CAN2.init(CAN_BPS_250K);
  CANbus0.begin(CAN_BPS_500K);
  CANbus1.begin(CAN_BPS_500K);
  Serial.println("Past begin()");

  //Set a filter for CANbus1 such that only frames that exactly match
  //TEST1_CAN_TRANSFER_ID can get through
  //CANbus1.setRXFilter(0, TEST1_CAN_TRANSFER_ID, 0x7FF, false);

  // Prepare transmit ID, data and data length in CAN0 mailbox 0
  output.id = TEST1_CAN_TRANSFER_ID;
  output.length = MAX_CAN_FRAME_DATA_LEN;
  //Set first four bytes (32 bits) all at once
  output.data.low = CAN_MSG_1;
  //Set last four bytes (32 bits) all at once
  output.data.high = CAN_MSG_DUMMY_DATA;
  //Send out the frame on whichever mailbox is free or queue it for
  //sending when there is an opening.
  Serial.println("Before write()");
  Serial.print("output.id = ");
  Serial.println(output.id);
  Serial.print("output.data.low = ");
  Serial.println(output.data.low);
  CANbus0.write(output);
  Serial.println("Past write()");

  // Wait for CAN1 mailbox 0 to receive the data
  while (!CANbus1.available()) {
  }
  Serial.println("Past available()");

  // Read the received data from CAN1 mailbox 0
  CAN_FRAME incoming;
  CANbus1.read(incoming);
  Serial.println("Past read()");
  
  Serial.print("CAN message received= ");
  Serial.print(incoming.data.low, HEX);
  Serial.print(incoming.data.high, HEX);
  
  // Disable CAN0 Controller
  CANbus0.end();

  // Disable CAN1 Controller
  CANbus1.end();

  Serial.print("\nEnd of test");

  while (1) {
  }
}

