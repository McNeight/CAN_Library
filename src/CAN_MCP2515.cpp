/* Copyright (C) 2014

   Contributor:  Pedro Cevallos

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Acknowledgements:
Fabian Greif for the initial MCP2515 library http://www.kreatives-chaos.com/artikel/universelle-can-bibliothek
David Harding for his version of the MCP2515 library http://forum.arduino.cc/index.php/topic,8730.0.html
Kyle Crockett CANduino library with 16Mhz oscillator. (http://code.google.com/p/canduino/)
Nuno Alves for the help on Extended ID messaging
Stevenh for his work on library and all of the MCP2515 research/work http://modelrail.otenko.com/arduino/arduino-controller-area-network-can

-------------------------------------------------------------------------------------------------------------
Change Log

DATE		VER		WHO			WHAT
07/07/13	0.1		PC		Modified and merge all MCP2515 libraries found. Stripped away most unused functions and corrected MCP2515 defs
09/12/13	0.2		PC		Added selectable CS SPI for CAN controller to use 1 IC to control several mcp2515
02/05/14	0.3		PC		Added filter and mask controls
05/01/14	0.4		PC		Cleaned up functions, variables and added message structures for J1939, CANopen and CAN.
05/07/14	1.0		PC		Released Library to the public through GitHub
-------------------------------------------------------------------------------------------------------------

Features:

CAN V2.0B
8 byte length in the data field
Standard and extended data frames
Two receive buffers
Three Transmit Buffers
SPI Interface

Supported bitrates:
10 kbps; 20 kbps; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

Designed to be used with ATMEL ATMega328P with Arduino bootloader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers.
*/

#if defined(ARDUINO_ARCH_AVR)

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
  bitRate(bitrate);//Set CAN bit rate
  setMode(mode);//Set CAN mode
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
  return ((msgStatus & 0x01) + (msgStatus & 0x02));
}


//Receive and display CAN message. This allows use of the message structure for easier message handling. Note: This fucntion does not allow extended frames as by default CAN is standard frames.
CAN_FRAME CAN_MCP2515::read()
{
  CAN_FRAME message;
  uint8_t len, i, buffer, msgStatus;
  uint16_t sid_h, sid_l, eid8, eid0, temp, rxbDlc;
  msgStatus = readStatus();

  digitalWrite(CS, LOW);
  // (msgStatus & 0x01) means message in RX buffer 0
  if (msgStatus & 0x01)
  {
    SPI.transfer(MCP2515_READ_RX_BUF_0_ID);
  }
  // (msgStatus & 0x02) means message in RX buffer 1
  else if (msgStatus & 0x02)
  {
    SPI.transfer(MCP2515_READ_RX_BUF_1_ID);
  }
  else
  {
    // No message?
    message.valid = false;
    digitalWrite(CS, HIGH);
    return message;
  }
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  rxbDlc = SPI.transfer(0xFF); // get bits and other data from MCP2515 RXB DLC buffer (it contains data and RTR)
  len = ((rxbDlc) & 0x0F);
  //len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++)
  {
    message.data[i] = SPI.transfer(0xFF);
  }
  digitalWrite(CS, HIGH);
  message.length = len;
  message.rtr = bitRead(rxbDlc, MCP2515_RTR);
  message.extended = bitRead (sid_l, MCP2515_IDE);
  if (message.extended)   // check to see if this is an Extended ID Msg.
  {
    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((uint16_t) sid_h >> 3); //shift SIDH
    message.id = (((uint32_t) sid_h << 24) | ((uint32_t) sid_l << 16 ) | ((uint32_t) eid8 << 8) | (eid0 << 0)); // repack message ID
  }
  else
  {
    message.id = ((((uint16_t) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame.
  }
  // everything checks out!
  message.valid = true;

  return message;
}


// Receive and display any message (J1939, CANopen, CAN).
// This functions provides an easy way to see the message if user doesn't care about the actual message protocol.
// No message struct is used here.
uint8_t CAN_MCP2515::read(uint32_t * ID, uint8_t * length_out, uint8_t * data_out)
{
  uint8_t len, i, buffer, msgStatus;
  uint16_t sid_h, sid_l, eid8, eid0, temp;
  msgStatus = readStatus();
  if (msgStatus & 0x01)
  {
    buffer = MCP2515_READ_RX_BUF_0_ID;
  }
  else if (msgStatus & 0x02)
  {
    buffer = MCP2515_READ_RX_BUF_1_ID;
  }
  else
  {
    return false;
  }
  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++)
  {
    data_out[i] = SPI.transfer(0xFF);
  }
  digitalWrite(CS, HIGH);
  (*length_out) = len;
  if (bitRead (sid_l, MCP2515_IDE))   // check to see if this is an Extended ID Msg.
  {
    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((uint16_t) sid_h >> 3); //shift SIDH
    (*ID) = (((uint32_t) sid_h << 24) | ((uint32_t) sid_l << 16 ) | ((uint32_t) eid8 << 8) | (eid0 << 0)); // repack message
  }
  else
  {
    (*ID) = ((((uint16_t) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame.
  }
}


void CAN_MCP2515::flush()
{
  clearRxBuffers();
  clearTxBuffers();
}

uint8_t CAN_MCP2515::write(CAN_FRAME & message)
{
  // Serial.print("id:");
  // Serial.println(message.id, HEX);
  uint8_t i, id_high, id_low, ex_high, ex_low, msgStatus, loadBuffer, sendBuffer;
  msgStatus = readStatus() & 0x54;
  if (msgStatus != 0x54)
  {
    if ((msgStatus & 0x04)  == 0) //transmit buffer 0 is open
    {
      loadBuffer = MCP2515_LOAD_TX_BUF_0_ID;
      sendBuffer = MCP2515_SEND_TX_BUF_0;
    }
    else if ((msgStatus & 0x10) == 0) //transmit buffer 1 is open
    {
      loadBuffer = MCP2515_LOAD_TX_BUF_1_ID;
      sendBuffer = MCP2515_SEND_TX_BUF_1;
    }
    else   // transmit buffer 2 is open
    {
      loadBuffer = MCP2515_LOAD_TX_BUF_2_ID;
      sendBuffer = MCP2515_SEND_TX_BUF_2;
    }
  }

  //Serial.print("message.extended:");
  //Serial.println(message.extended);

  //if(bitRead ((ID),27)==1){// might use this later to remove Frame Type. It works for right now....
  if (message.extended == CAN_EXTENDED_FRAME)
  {
    //generate id bytes before SPI write
    id_high = (message.id >> 21);
    id_low = (message.id >> 18);
    id_low = ((id_low) << 5 | 0x08);
    if (bitRead ((message.id), 17) == 1) bitSet(id_low, 1);
    if (bitRead ((message.id), 16) == 1) bitSet(id_low, 0);
    // Serial.print("id_high:");
    // Serial.print(id_high, HEX);
    // Serial.print(" id_low:");
    // Serial.println(id_low, HEX);
    ex_high = (message.id) >> 8;
    ex_low = (message.id) >> 0;
    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(ex_high); //extended ID high bits
    SPI.transfer(ex_low); //extended ID low bits
    SPI.transfer(message.length);
    for (i = 0; i < message.length; i++)   //load data buffer
    {
      SPI.transfer(message.data[i]);
    }
  }
  else if (message.extended == CAN_BASE_FRAME)
  {
    id_high = (uint8_t) (message.id >> 3);
    id_low = (uint8_t) ((message.id << 5) & 0x00E0);
    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(0x00); //extended ID registers
    SPI.transfer(0x00);
    SPI.transfer(message.length); //data length code
    for (i = 0; i < message.length; i++)   //load data buffer
    {
      SPI.transfer(message.data[i]);
    }
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer(sendBuffer);
  digitalWrite(CS, HIGH);
}


// Function to load and send any message. (J1939, CANopen, CAN). It assumes user knows what the ID is supposed to be
uint8_t CAN_MCP2515::write(uint32_t ID, uint8_t frameType, uint8_t length, uint8_t * data) // changed from send() to write()
{
  uint8_t i, id_high, id_low, ex_high, ex_low, msgStatus, loadBuffer, sendBuffer;
  msgStatus = readStatus();
  if (!(msgStatus & 0x04)) //transmit buffer 0 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUF_0_ID;
    sendBuffer = MCP2515_SEND_TX_BUF_0;
  }
  else if (!(msgStatus & 0x10)) //transmit buffer 1 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUF_1_ID;
    sendBuffer = MCP2515_SEND_TX_BUF_1;
  }
  else if (!(msgStatus & 0x40)) //transmit buffer 2 is open
  {
    loadBuffer = MCP2515_LOAD_TX_BUF_2_ID;
    sendBuffer = MCP2515_SEND_TX_BUF_2;
  }
  else
  {
    // all transmit buffers are full; please try again later
    return 0;
  }
  
  //if(bitRead ((ID),27)==1){// might use this later to remove Frame Type. It works for right now....
  if (frameType == CAN_EXTENDED_FRAME)
  {
    //generate id bytes before SPI write
    id_high = (ID >> 21);
    id_low = (ID >> 18);
    id_low = ((id_low) << 5 | 0x08);
    if (bitRead ((ID), 17) == 1) bitSet(id_low, 1);
    if (bitRead ((ID), 16) == 1) bitSet(id_low, 0);
    ex_high = (ID) >> 8;
    ex_low = (ID) >> 0;
    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(ex_high); //extended ID high bits
    SPI.transfer(ex_low); //extended ID low bits
    SPI.transfer(length);
    for (i = 0; i < length; i++)   //load data buffer
    {
      SPI.transfer(data[i]);
    }
  }
  else if (frameType == CAN_BASE_FRAME)
  {
    id_high = (uint8_t) (ID >> 3);
    id_low = (uint8_t) ((ID << 5) & 0x00E0);
    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(0x00); //extended ID registers
    SPI.transfer(0x00);
    SPI.transfer(length); //data length code
    for (i = 0; i < length; i++)   //load data buffer
    {
      SPI.transfer(data[i]);
    }
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer(sendBuffer);
  digitalWrite(CS, HIGH);
}

// MCP2515 SPI INTERFACE COMMANDS

// Reset command
void CAN_MCP2515::reset()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_RESET);
  digitalWrite(CS, HIGH);
}

//Reads MCP2515 registers
uint8_t CAN_MCP2515::readAddress(uint8_t address)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_READ);
  SPI.transfer(address);
  uint8_t retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;
}

// Writes MCP2515 registers
void CAN_MCP2515::writeAddress(uint8_t address, uint8_t value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

// Modify MCP2515 registers
void CAN_MCP2515::modifyAddress(uint8_t address, uint8_t mask, uint8_t value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_BIT_MODIFY);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

// MCP2515 INITIALIZATION COMMANDS.

//MCP2515 can be set into 5 different modes: CONFIG, NORMAL,SLEEP,LISTEN, LOOPBACK
void CAN_MCP2515::setMode(uint8_t mode)
{
  // (1 << MCP2515_REQOP0) | (1 << MCP2515_REQOP1) | (1 << MCP2515_REQOP2) = 0xE0
  modifyAddress(MCP2515_CANCTRL, 0xE0, mode); //Writes config values to registers
}

// Function to read mode back
uint8_t CAN_MCP2515::readMode()
{
  // (1 << MCP2515_REQOP0) | (1 << MCP2515_REQOP1) | (1 << MCP2515_REQOP2) = 0xE0
  return (readAddress(MCP2515_CANSTAT) & 0xE0);
}


//Sets MCP2515 controller bitrate.
// Configuration speeds are determined by Crystal Oscillator.
// See MCP2515 datasheet Pg39 for more info.
void CAN_MCP2515::bitRate(uint32_t bitrate)
{
  uint8_t config1, config2, config3;
  if (bitrate == 10000)
  {
    config1 = 0x31; // Set BRP5, BRP4, and BRP0
    config2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    config3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 20000)
  {
    config1 = 0x18;
    config2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    config3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 50000)
  {
    config1 = 0x09;
    config2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    config3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 100000)
  {
    config1 = 0x04;
    config2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    config3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 125000)
  {
    config1 = 0x03;
    config2 = 0xB8; // Set BTLMODE and PHSEG1<2:0>
    config3 = 0x05; // Set PHSEG22 and PHSEG20
  }
  else if (bitrate == 250000)
  {
    config1 = 0x01; //config1 = 0x41; these are other configs for 500kb/s. need to confirm with current crystal osc @ 16mhz
    config2 = 0xB8; //config2 = 0xF1;
    config3 = 0x05; //config3 = 0x85;
  }
  else if (bitrate == 500000)
  {
    config1 = 0x00;
    config2 = 0xB8; //config2 = 0xF0; these are other configs for 500kb/s. need to confirm with current crystal osc @ 16mhz
    config3 = 0x05; //config3 = 0x86;
  }
  else if (bitrate == 1000000)
  {
    config1 = 0x00; //config1 = 0x80; these are other configs for 1Mb/s. need to confirm with current crystal osc @ 16mhz
    config2 = 0xD0; //config2 = 0x90;
    config3 = 0x82; //config3 = 0x02;
  }
  writeAddress(MCP2515_CNF1, config1);//Write config address 1
  writeAddress(MCP2515_CNF2, config2);//Write config address 2
  writeAddress(MCP2515_CNF3, config3);//Write config address 3
}

uint32_t CAN_MCP2515::readRate()
{
  uint8_t config1, config2, config3;
  config1 = readAddress(MCP2515_CNF1);
  config2 = readAddress(MCP2515_CNF2);
  config3 = readAddress(MCP2515_CNF3);
  if ((config2 == 0xB8) && (config3 == 0x05))
  {
    if (config1 == 0x31)
    {
      return 10000;
    }
    else if (config1 == 0x18)
    {
      return 20000;
    }
    else if (config1 == 0x09)
    {
      return 50000;
    }
    else if (config1 == 0x04)
    {
      return 100000;
    }
    else if (config1 == 0x03)
    {
      return 125000;
    }
    else if (config1 == 0x01)
    {
      return 250000;
    }
    else if (config1 == 0x00)
    {
      return 500000;
    }
  }
  else if ((config1 == 0x00) && (config2 == 0xD0) && (config3 == 0x82))
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
  // (1 << MCP2515_RXM0) | (1 << MCP2515_RXM1) = 0x60
  modifyAddress(MCP2515_RXB0CTRL, 0x60, 0x60);
  modifyAddress(MCP2515_RXB1CTRL, 0x60, 0x60);
}

//Set Masks for filters
void CAN_MCP2515::setMask(uint8_t mask, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
  setMode(MCP2515_MODE_CONFIG);
  writeAddress(mask, data0);
  writeAddress(mask + 1, data1);
  writeAddress(mask + 2, data2);
  writeAddress(mask + 3, data3);
  setMode(MCP2515_MODE_NORMAL);
}

// Set Receive Filters. Will think of a more user friendly way to set these in the future but for right now it works....
void CAN_MCP2515::setFilter(uint8_t filter, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
  setMode(MCP2515_MODE_CONFIG);
  writeAddress(filter, data0);
  writeAddress(filter + 1, data1);
  writeAddress(filter + 2, data2);
  writeAddress(filter + 3, data3);
  setMode(MCP2515_MODE_NORMAL);
}

//At power up, MCP2515 buffers are not truly empty. There is random data in the registers
//This loads buffers with zeros to prevent incorrect data to be sent.
void CAN_MCP2515::clearRxBuffers()
{
  uint8_t writeVal = 0x00;
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_RXB0SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_RXB1SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
}

// This loads buffers with zeros to prevent incorrect data to be sent. Note: It RTS is sent to a buffer that has all zeros it will still send a message with all zeros.
void CAN_MCP2515::clearTxBuffers()
{
  uint8_t writeVal = 0x00;
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB0SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB1SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (MCP2515_TXB2SIDH);
  for (uint8_t i = 0; i < 13; i++)
  {
    SPI.transfer(writeVal);
  }
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
  /*
  FIGURE 12-8: READ STATUS INSTRUCTION
  bit 0: CANINTF.RX0IF
  bit 1: CANINTF.RX1IF
  bit 2: TXB0CNTRL.TXREQ
  bit 3: CANINTF.TX0IF
  bit 4: TXB1CNTRL.TXREQ
  bit 5: CANINTF.TX1IF
  bit 6: TXB2CNTRL.TXREQ
  bit 7: CANINTF.TX2IF

  (readStatus() & 0x01) == 0x01 means message in RX buffer 0
  (readStatus() & 0x02) == 0x02 means message in RX buffer 1
  (readStatus() & 0x04) == 0x04 means message in TX buffer 0
  (readStatus() & 0x08) == 0x08 means Transmit Buffer 0 Empty Interrupt Flag set
  (readStatus() & 0x10) == 0x10 means message in TX buffer 1
  (readStatus() & 0x20) == 0x20 means Transmit Buffer 1 Empty Interrupt Flag set
  (readStatus() & 0x40) == 0x40 means message in TX buffer 2
  (readStatus() & 0x80) == 0x80 means Transmit Buffer 2 Empty Interrupt Flag set
  */
}

//Function that reads receive functions and filhits
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

//Enable hardware Request to send pins. It allows messages to be send by driving RTS pins low.
//These are not to be confused with RTS commands. These are the actual MCP2515 hardware pins
void CAN_MCP2515::enableRTSPins()
{
  // (1 << MCP2515_B0RTSM) | (1 << MCP2515_B1RTSM) | (1 << MCP2515_B2RTSM) = 0x07
  writeAddress(MCP2515_TXRTSCTRL, 0x07);
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

#endif // defined(ARDUINO_ARCH_AVR)

