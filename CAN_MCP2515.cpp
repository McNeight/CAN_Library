/*
    Copyright © 2007-2015 Fabian Greif, David Harding, Kyle Crockett,
    Nuno Alves, Stevenh, Collin Kidder, Daniel Kasamis, Cory Fowler, teachop,
    Pedro Cevallos, Neil McNeight

    This file is part of CAN_Library.

    CAN_Library is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 2.1 of the License, or
    (at your option) any later version.

    CAN_Library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Acknowledgements:
  Fabian Greif for the initial libraries for MCP2515, SJA1000 and AT90CAN
    http://www.kreatives-chaos.com/artikel/universelle-can-bibliothek
    as well as his updates at https://github.com/dergraaf/avr-can-lib
  David Harding for his version of the MCP2515 library
    http://forum.arduino.cc/index.php/topic,8730.0.html
  Kyle Crockett CANduino library with 16Mhz oscillator
    http://code.google.com/p/canduino/
  Nuno Alves for the help on Extended ID messaging
  Stevenh for his work on library and all of the MCP research/work
    http://modelrail.otenko.com/arduino/arduino-controller-area-network-can
  Collin Kidder (collin80) for his work on the Arduino Due CAN interface
    https://github.com/collin80/due_can
  Daniel Kasamis (togglebit) both for his code at
    https://github.com/togglebit/ArduinoDUE_OBD_FreeRunningCAN as well as his
    DUE CANshield http://togglebit.net/product/arduino-due-can-shield/
  Cory Fowler (coryjfowler) for 16 MHz bitrate timing information
    https://github.com/coryjfowler/MCP2515_lib
  teachop for the FlexCAN library for the Teensy 3.1
    https://github.com/teachop/FlexCAN_Library

-------------------------------------------------------------------------------
Change Log

DATE      VER   WHO   WHAT
07/07/13  0.1   PC    Modified and merge all MCP2515 libraries found. Stripped
                        away most unused functions and corrected MCP2515 defs
09/12/13  0.2   PC    Added selectable CS SPI for CAN controller to use 1 IC
                        to control several mcp2515
02/05/14  0.3   PC    Added filter and mask controls
05/01/14  0.4   PC    Cleaned up functions, variables and added message
                        structures for J1939, CANopen and CAN.
05/07/14  1.0   PC    Released Library to the public through GitHub
06/18/14  1.5   NEM   Preparing a unified CAN library across three different
                        CAN controllers
06/14/15  1.6.0 NEM   Code cleanup and compatibility with Arduino 1.6.*
-------------------------------------------------------------------------------

*/


#if defined(ARDUINO_ARCH_AVR) && !defined(__MK20DX256__)

#include <Arduino.h>
#include <SPI.h>
#include "CAN.h"
#include "CAN_MCP2515.h"

///////////////////////////////////////////////////////////////////
///							                                                ///
///	                 CAN library for MCP2515		           	    ///
///							                                                ///
///////////////////////////////////////////////////////////////////

//Initialize SPI communications and set MCP2515 into Config mode
CAN_MCP2515::CAN_MCP2515()
{
  //Use a default of pin 10 for SPI chip select
  CS = 10;
  _init();
}

CAN_MCP2515::CAN_MCP2515(uint8_t CS_Pin)
{
  CS = CS_Pin;
  _init();
}

void CAN_MCP2515::_init()
{
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
}

//Start MCP2515 communications
void CAN_MCP2515::begin(uint32_t bitrate, uint8_t mode)
{
  SPI.begin();//SPI communication begin
  reset();//Set MCP2515 into Config mode by soft reset. Note MCP2515 is in Config mode by default at power up.
  clearRxBuffers();
  clearTxBuffers();
  clearFilters();
  // enable Transmit Buffer Empty Interrupt Enable bits
  //enableInterrupts(MCP2515_TXnIE, MCP2515_TXnIE);
  setBitrate(bitrate); //Set CAN bit rate
  setMode(mode);    //Set CAN mode
}


void CAN_MCP2515::end()
{
  SPI.end();
}

// Check to see if message is available
uint8_t CAN_MCP2515::available()
{
  uint8_t msgStatus = readStatus();
  // (msgStatus & 0x01) means message in RX buffer 0
  // (msgStatus & 0x02) means message in RX buffer 1
  // Returns number of messages available
  return (msgStatus & MCP2515_STATUS_CANINTF_RXnIF);
}

// Receive and display CAN message.
// This allows use of the message structure for easier message handling.
CAN_Frame CAN_MCP2515::read()
{
  CAN_Frame message;
  uint8_t buffer, msgStatus;
  uint8_t RXBnSIDH, RXBnSIDL, RXBnEID8, RXBnEID0, RXBnDLC;

  msgStatus = readStatus();

  if (msgStatus & MCP2515_STATUS_CANINTF_RX0IF)
  {
    buffer = MCP2515_READ_RX_BUFFER_0_ID;
  }
  else if (msgStatus & MCP2515_STATUS_CANINTF_RX1IF)
  {
    buffer = MCP2515_READ_RX_BUFFER_1_ID;
  }
  else
  {
    // No message?
    message.valid = false;
    return message;
  }

  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  RXBnSIDH = SPI.transfer(0xFF); // SID<10:3>
  RXBnSIDL = SPI.transfer(0xFF); // SID<2:0>, SRR, IDE, EID<17:16>
  RXBnEID8 = SPI.transfer(0xFF); // EID<15:8>
  RXBnEID0 = SPI.transfer(0xFF); // EID<7:0>
  RXBnDLC  = SPI.transfer(0xFF); // RTR, RB<1:0>, DLC<3:0>
  message.length = (RXBnDLC & MCP2515_DLC);
  for (int i = 0; i < message.length; i++)
  {
    message.data[i] = SPI.transfer(0xFF);
  }
  digitalWrite(CS, HIGH);

  message.extended = bitRead(RXBnSIDL, MCP2515_IDE);
  // check to see if this is an Extended ID Msg.
  if (message.extended == CAN_EXTENDED_FRAME)
  {
    // Serial.println(message.id, HEX);
    // Serial.print(RXBnSIDH, HEX);
    // Serial.print(RXBnSIDL, HEX);
    // Serial.print(RXBnEID8, HEX);
    // Serial.println(RXBnEID0, HEX);
    // If you don't cast to a larger int _before_ assignment, then
    // sign extension _WILL_ bite you!!!
    // https://en.wikipedia.org/wiki/Sign_extension
    message.id  = ((uint32_t)RXBnSIDH << 21);            // ID<28:21> = SIDH<7:0>
    message.id |= ((uint32_t)(RXBnSIDL & MCP2515_SIDL_SID) << 13); // ID<20:18> = SIDL<7:5>
    message.id |= ((uint32_t)(RXBnSIDL & MCP2515_SIDL_EID) << 16); // ID<17:16> = SIDL<1:0>
    //Serial.println(message.id, HEX);
    message.id |= ((uint32_t)RXBnEID8 << 8);             // ID<15:8>  = EID8<7:0>
    //Serial.println(message.id, HEX);
    message.id |= ((uint32_t)RXBnEID0 << 0);             // ID<7:0>   = EID0<7:0>
    //Serial.println(message.id, HEX);
    message.rtr = bitRead(RXBnDLC, MCP2515_RTR);
    //Serial.println(message.id, HEX);
  }
  else if (message.extended == CAN_STANDARD_FRAME)
  {
    message.id  = (RXBnSIDH << 3);                       // ID<10:3> = SIDH<7:0>
    message.id |= ((RXBnSIDL & MCP2515_SIDL_SID) >> 5);  // ID<2:0>  = SIDL<7:5>
    message.rtr = bitRead(RXBnSIDL, MCP2515_SRR);
  }
  // everything checks out!
  message.valid = true;

  return message;
}

// Receive and display any message (J1939, CANopen, CAN).
// This functions provides an easy way to see the message if user doesn't care about the actual message protocol.
// No message struct is used here.
void CAN_MCP2515::read(uint32_t * ID, uint8_t * length_out, uint8_t * data_out)
{
  CAN_Frame message_to_be_parsed;
  message_to_be_parsed = read();
  (*ID) = message_to_be_parsed.id;
  (*length_out) = message_to_be_parsed.length;
  memcpy(data_out, message_to_be_parsed.data, sizeof(message_to_be_parsed.data));
}

void CAN_MCP2515::flush()
{
  clearRxBuffers();
  clearTxBuffers();
}

uint8_t CAN_MCP2515::write(const CAN_Frame & message)
{
  uint8_t TXBnSIDH, TXBnSIDL, TXBnEID8, TXBnEID0, TXBnDLC, msgStatus, loadBuffer, sendBuffer;
  msgStatus = readStatus();

  if (!(msgStatus & MCP2515_STATUS_TXB0CNTRL_TXREQ)) //transmit buffer 0 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUFFER_0_ID;
    sendBuffer = MCP2515_RTS_TXB0;
  }
  else if (!(msgStatus & MCP2515_STATUS_TXB1CNTRL_TXREQ)) //transmit buffer 1 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUFFER_1_ID;
    sendBuffer = MCP2515_RTS_TXB1;
  }
  else if (!(msgStatus & MCP2515_STATUS_TXB2CNTRL_TXREQ)) //transmit buffer 2 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUFFER_2_ID;
    sendBuffer = MCP2515_RTS_TXB2;
  }
  else
  {
    // No transmit buffers available; no message sent
    return 0;
  }

  TXBnDLC = (message.length & MCP2515_DLC);
  if (message.extended == CAN_EXTENDED_FRAME)
  {
    //generate id bytes before SPI write
    TXBnSIDH  = (message.id >> 21);                      // SIDH<7:0> = ID<28:21>
    TXBnSIDL  = ((message.id >> 13) & MCP2515_SIDL_SID); // SIDL<7:5> = ID<20:18>
    TXBnSIDL |= ((message.id >> 16) & MCP2515_SIDL_EID); // SIDL<1:0> = ID<17:16>
    bitSet(TXBnSIDL, MCP2515_IDE);
    TXBnEID8  = (message.id >> 8);                       // EID8<7:0> = ID<15:8>
    TXBnEID0  = (message.id >> 0);                       // EID0<7:0> = ID<7:0>
    if (message.rtr)
    {
      bitSet(TXBnDLC, MCP2515_RTR);
    }
  }
  else if (message.extended == CAN_STANDARD_FRAME)
  {
    TXBnSIDH = (message.id >> 3);                        // SIDH<7:0> = ID<10:3>
    TXBnSIDL = ((message.id << 5) & MCP2515_SIDL_SID);   // SIDL<7:5> = ID<2:0>
    TXBnEID8 = 0x00; // zero out extended ID registers
    TXBnEID0 = 0x00; // zero out extended ID registers
    if (message.rtr)
    {
      bitSet(TXBnSIDL, MCP2515_SRR);
    }
  }

  digitalWrite(CS, LOW);
  SPI.transfer(loadBuffer);
  SPI.transfer(TXBnSIDH); //ID high bits
  SPI.transfer(TXBnSIDL); //ID low bits
  SPI.transfer(TXBnEID8); //extended ID high bits
  SPI.transfer(TXBnEID0); //extended ID low bits
  SPI.transfer(TXBnDLC); //data length code
  for (int i = 0; i < message.length; i++)   //load data buffer
  {
    SPI.transfer(message.data[i]);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer(sendBuffer);
  digitalWrite(CS, HIGH);

  return message.length;
}


// Function to load and send any message. (J1939, CANopen, CAN). It assumes user knows what the ID is supposed to be
uint8_t CAN_MCP2515::write(uint32_t ID, uint8_t frameType, uint8_t length, uint8_t * data) // changed from send() to write()
{
  CAN_Frame message_to_be_sent;
  message_to_be_sent.id = ID;
  message_to_be_sent.length = length;
  message_to_be_sent.extended = frameType;
  memcpy(message_to_be_sent.data, data, sizeof(data));
  return (write(message_to_be_sent));
}

// MCP2515 SPI INTERFACE COMMANDS
// Reset command
void CAN_MCP2515::reset()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_RESET);
  digitalWrite(CS, HIGH);
}

//Reads a single MCP2515 register
uint8_t CAN_MCP2515::readAddress(uint8_t address)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_READ);
  SPI.transfer(address);
  uint8_t retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;
}

// Writes a single MCP2515 register
void CAN_MCP2515::writeAddress(uint8_t address, uint8_t value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

// Modifies a single MCP2515 register
void CAN_MCP2515::modifyAddress(uint8_t address, uint8_t mask, uint8_t value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_BIT_MODIFY);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

//Function that reads several status bits for transmit and receive functions.
uint8_t CAN_MCP2515::readStatus()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_READ_STATUS);
  uint8_t retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;
}

//Function that reads receive functions and filter hits
uint8_t CAN_MCP2515::readRXStatus()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_RX_STATUS);
  uint8_t retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;
  /*
  Values are as follows

    |bit|bit|bit|bit|bit|bit|bit|bit| Received Message
    | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | -----------------
    | 0 | 0 |---|---|---|---|---|---| No RX message
    | 0 | 1 |---|---|---|---|---|---| Message in RXB0
    | 1 | 0 |---|---|---|---|---|---| Message in RXB1
    | 1 | 1 |---|---|---|---|---|---| Message in both buffers* (Buffer 0 has higher priority,
  				    therefore, RXB0 status is reflected in bits 4:0)

    |bit|bit|bit|bit|bit|bit|bit|bit| Msg Type Received
    | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | -----------------
    |---|---|---| 0 | 0 |---|---|---| Standard data frame
    |---|---|---| 0 | 1 |---|---|---| Standard remote frame
    |---|---|---| 1 | 0 |---|---|---| Extended data frame
    |---|---|---| 1 | 1 |---|---|---| Extended remote frame

    |bit|bit|bit|bit|bit|bit|bit|bit| Filter Match
    | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | -----------------
    |---|---|---|---|---|	0 | 0 | 0 | RXF0
    |---|---|---|---|---| 0 | 0 | 1 | RXF1
    |---|---|---|---|---| 0 | 1 | 0 | RXF2
    |---|---|---|---|---| 0 | 1 | 1 | RXF3
    |---|---|---|---|---| 1 | 0 | 0 | RXF4
    |---|---|---|---|---| 1 | 0 | 1 | RXF5
    |---|---|---|---|---| 1 | 1 | 0 | RXF0 (rollover to RXB1)
    |---|---|---|---|---| 1 | 1 | 1 | RXF1 (rollover to RXB1)
    */
}

// MCP2515 INITIALIZATION COMMANDS.
//MCP2515 can be set into 5 different modes: CONFIG, NORMAL, SLEEP, LISTEN, LOOPBACK
void CAN_MCP2515::setMode(uint8_t mode)
{
  modifyAddress(MCP2515_CANCTRL, MCP2515_REQOPn, mode); //Writes config values to registers
}

// Function to read mode back
uint8_t CAN_MCP2515::getMode()
{
  return (readAddress(MCP2515_CANSTAT) & MCP2515_REQOPn);
}

//Sets MCP2515 controller bitrate.
// Configuration speeds are determined by Crystal Oscillator.
// See MCP2515 datasheet Pg39 for more info.
void CAN_MCP2515::setBitrate(uint32_t bitrate)
{
  uint8_t CNF1, CNF2, CNF3;
  if (bitrate == 10000)
  {
    CNF1 = 0x31; // Set BRP5, BRP4, and BRP0
    CNF2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    CNF3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 20000)
  {
    CNF1 = 0x18;
    CNF2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    CNF3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 50000)
  {
    CNF1 = 0x09;
    CNF2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    CNF3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 100000)
  {
    CNF1 = 0x04;
    CNF2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    CNF3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 125000)
  {
    CNF1 = 0x03;
    CNF2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    CNF3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 250000)
  {
    CNF1 = 0x01;
    CNF2 = 0xB8;
    CNF3 = 0x05;
  }
  else if (bitrate == 500000)
  {
    CNF1 = 0x00;
    CNF2 = 0xB8;
    CNF3 = 0x05;
  }
  else if (bitrate == 1000000)
  {
    CNF1 = 0x00;
    CNF2 = 0xD0;
    CNF3 = 0x82;
  }
  writeAddress(MCP2515_CNF1, CNF1);//Write config address 1
  writeAddress(MCP2515_CNF2, CNF2);//Write config address 2
  writeAddress(MCP2515_CNF3, CNF3);//Write config address 3
}

//Sets MCP2515 controller bitrate.
// Configuration speeds are determined by 16 MHz Crystal Oscillator.
// https://github.com/coryjfowler/MCP2515_lib/blob/master/mcp_can_dfs.h
// Baudrates 5k, 10k, 20k, 50k, 100k, 125k, 250k, 500k, & 1000k are confirmed
// to work using a Peak-System PCAN-USB dongle as a reference.
void CAN_MCP2515::setBitrate16MHz(uint32_t bitrate)
{
  uint8_t CNF1, CNF2, CNF3;

  if (bitrate == 5000)
  {
    CNF1 = 0x3F;
    CNF2 = 0xFF;
    CNF3 = 0x87;
  }
  else if (bitrate == 10000)
  {
    CNF1 = 0x1F;
    CNF2 = 0xFF;
    CNF3 = 0x87;
  }
  else if (bitrate == 20000)
  {
    CNF1 = 0x0F;
    CNF2 = 0xFF;
    CNF3 = 0x87;
  }
  else if (bitrate == 31025)
  {
    CNF1 = 0x0F;
    CNF2 = 0xF1;
    CNF3 = 0x85;
  }
  else if (bitrate == 40000)
  {
    CNF1 = 0x07;
    CNF2 = 0xFF;
    CNF3 = 0x87;
  }
  else if (bitrate == 50000)
  {
    CNF1 = 0x07;
    CNF2 = 0xFA;
    CNF3 = 0x87;
  }
  else if (bitrate == 80000)
  {
    CNF1 = 0x03;
    CNF2 = 0xFF;
    CNF3 = 0x87;
  }
  else if (bitrate == 100000)
  {
    CNF1 = 0x03;
    CNF2 = 0xFA;
    CNF3 = 0x87;
  }
  else if (bitrate == 125000)
  {
    CNF1 = 0x03;
    CNF2 = 0xF0;
    CNF3 = 0x86;
  }
  else if (bitrate == 200000)
  {
    CNF1 = 0x01;
    CNF2 = 0xFA;
    CNF3 = 0x87;
  }
  else if (bitrate == 250000)
  {
    CNF1 = 0x41;
    CNF2 = 0xF1;
    CNF3 = 0x85;
  }
  else if (bitrate == 500000)
  {
    CNF1 = 0x00;
    CNF2 = 0xF0;
    CNF3 = 0x86;
  }
  else if (bitrate == 1000000)
  {
    CNF1 = 0x00;
    CNF2 = 0xD0;
    CNF3 = 0x82;
  }
  writeAddress(MCP2515_CNF1, CNF1);//Write config address 1
  writeAddress(MCP2515_CNF2, CNF2);//Write config address 2
  writeAddress(MCP2515_CNF3, CNF3);//Write config address 3
}

uint32_t CAN_MCP2515::getBitrate()
{
  uint8_t CNF1, CNF2, CNF3;
  CNF1 = readAddress(MCP2515_CNF1);
  CNF2 = readAddress(MCP2515_CNF2);
  CNF3 = readAddress(MCP2515_CNF3);
  if ((CNF2 == 0xB8) && (CNF3 == 0x05))
  {
    if (CNF1 == 0x31)
    {
      return 10000;
    }
    else if (CNF1 == 0x18)
    {
      return 20000;
    }
    else if (CNF1 == 0x09)
    {
      return 50000;
    }
    else if (CNF1 == 0x04)
    {
      return 100000;
    }
    else if (CNF1 == 0x03)
    {
      return 125000;
    }
    else if (CNF1 == 0x01)
    {
      return 250000;
    }
    else if (CNF1 == 0x00)
    {
      return 500000;
    }
  }
  else if ((CNF1 == 0x00) && (CNF2 == 0xD0) && (CNF3 == 0x82))
  {
    return 1000000;
  }
  else
  {
    return 0;
  }
}

//Turns RX filters/masks off. Will receive any message.
void CAN_MCP2515::clearFilters()
{
  modifyAddress(MCP2515_RXB0CTRL, MCP2515_RXMn, MCP2515_RXMn);
  modifyAddress(MCP2515_RXB1CTRL, MCP2515_RXMn, MCP2515_RXMn);
}

//Set Masks for filters
void CAN_MCP2515::setMask(uint8_t mask, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  setMode(MCP2515_MODE_CONFIG);
  digitalWrite(CS, LOW);
  SPI.transfer(mask);
  SPI.transfer(b0);
  SPI.transfer(b1);
  SPI.transfer(b2);
  SPI.transfer(b3);
  digitalWrite(CS, HIGH);
  setMode(MCP2515_MODE_NORMAL);
}

// Set Receive Filters. Will think of a more user friendly way to set these in the future but for right now it works....
void CAN_MCP2515::setFilter(uint8_t filter, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  setMode(MCP2515_MODE_CONFIG);
  digitalWrite(CS, LOW);
  SPI.transfer(filter);
  SPI.transfer(b0);
  SPI.transfer(b1);
  SPI.transfer(b2);
  SPI.transfer(b3);
  digitalWrite(CS, HIGH);
  setMode(MCP2515_MODE_NORMAL);
}

//At power up, MCP2515 buffers are not truly empty. There is random data in the registers
//This loads buffers with zeros to prevent incorrect data to be sent.
void CAN_MCP2515::clearRxBuffers()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_WRITE);
  SPI.transfer(MCP2515_RXB0SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_RXB1SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
}

// This loads buffers with zeros to prevent incorrect data to be sent.
// Note: If RTS is sent to a buffer that has all zeros it will still send a message with all zeros.
void CAN_MCP2515::clearTxBuffers()
{
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB0SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB1SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB2SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
}

//Enable hardware Request to send pins. It allows messages to be send by driving RTS pins low.
//These are not to be confused with RTS commands. These are the actual MCP2515 hardware pins
void CAN_MCP2515::enableRTSPins()
{
  // According to section 10.1, TXRTSCTRL is only modifiable in Configuration Mode
  setMode(MCP2515_MODE_CONFIG);
  writeAddress(MCP2515_TXRTSCTRL, MCP2515_BnRTSM); // enable TXnRTS pins
  setMode(MCP2515_MODE_NORMAL);
}

// Enable interrupts. The CANINTF register contains the corresponding interrupt flag bit for
// each interrupt source. When an interrupt occurs, the INT pin is driven low by the MCP2515
// and will remain low until the interrupt is cleared by the MCU. An interrupt can not be
// cleared if the respective condition still prevails.
void CAN_MCP2515::setInterrupts(uint8_t mask, uint8_t writeVal)
{
  modifyAddress(MCP2515_CANINTE, mask, writeVal);
  /*
  Bit 7: MERRE: Message Error Interrupt Enable bit
  Bit 6: WAKIE: Wake-up Interrupt Enable bit
  Bit 5: ERRIE: Error Interrupt Enable bit (multiple sources in EFLG register)
  Bit 4: TX2IE: Transmit Buffer 2 Empty Interrupt Enable bit
  Bit 3: TX1IE: Transmit Buffer 1 Empty Interrupt Enable bit
  Bit 2: TX0IE: Transmit Buffer 0 Empty Interrupt Enable bit
  Bit 1: RX1IE: Receive Buffer 1 Full Interrupt Enable bit
  Bit 0: RX0IE: Receive Buffer 0 Full Interrupt Enable bit
  */
}

CAN_MCP2515 CAN(10); // Create CAN channel using pin 10 for SPI chip select

#endif // defined(ARDUINO_ARCH_AVR)

