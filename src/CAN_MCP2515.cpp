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

Supported Baud rates
10 kbps; 20 kbps; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

Designed to be used with ATMEL ATMega328P with Arduino bootloader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers.
*/

#if defined(ARDUINO_ARCH_AVR)

#include <Arduino.h>
#include <SPI.h>
#include "CAN.h"
#include "CAN_MCP2515.h"

///////////////////////////////////////////////////////////////////
///							        ///
///	           CAN library for MCP2515		 	///
///								///
///////////////////////////////////////////////////////////////////



//Initialize SPI communications and set MCP2515 into Config mode

CAN_MCP2515::CAN_MCP2515(byte CS_Pin)
{
  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin, HIGH);

  CS = CS_Pin;

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
byte CAN_MCP2515::readAddress(byte address)
{

  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_READ);
  SPI.transfer(address);
  byte retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;
}

// Writes MCP2515 registers
void CAN_MCP2515::writeAddress(byte address, byte value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_WRITE);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);

}

// Modify MCP2515 registers
void CAN_MCP2515::modifyAddress(byte address, byte mask, byte value)
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
void CAN_MCP2515::setMode(byte mode)
{
  byte mask = 0xE0; //Mask prevents writing the wrong bits

  modifyAddress(CANCTRL, mask, mode); //Writes config values to registers
}

// Function to read mode back
byte CAN_MCP2515::readMode()
{
  byte mask = 0xE0; //Mask prevents reading the wrong bits

  return (readAddress(CANSTAT) & 0xE0);
}


//Sets MCP2515 controller bitrate. Configuration speeds are determined by Crystal Oscillator. See MCP2515 datasheet Pg39 for more info.
void CAN_MCP2515::bitRate(int CANspeed)
{
  byte config1, config2, config3;

  if (CANspeed == 10) {
    config1 = 0x31;
    config2 = 0xB8;
    config3 = 0x05;
  } else if (CANspeed == 20) {
    config1 = 0x18;
    config2 = 0xB8;
    config3 = 0x05;
  } else if (CANspeed == 50) {
    config1 = 0x09;
    config2 = 0xB8;
    config3 = 0x05;
  } else if (CANspeed == 100) {
    config1 = 0x04;
    config2 = 0xB8;
    config3 = 0x05;
  } else if (CANspeed == 125) {
    config1 = 0x03;
    config2 = 0xB8;
    config3 = 0x05;
  } else if (CANspeed == 250) {
    config1 = 0x01; //config1 = 0x41; these are other configs for 500kb/s. need to confirm with current crystal osc @ 16mhz
    config2 = 0xB8; //config2 = 0xF1;
    config3 = 0x05; //config3 = 0x85;
  } else if (CANspeed == 500) {
    config1 = 0x00;
    config2 = 0xB8; //config2 = 0xF0; these are other configs for 500kb/s. need to confirm with current crystal osc @ 16mhz
    config3 = 0x05; //config3 = 0x86;
  } else if (CANspeed == 1000) {
    config1 = 0x00; //config1 = 0x80; these are other configs for 1Mb/s. need to confirm with current crystal osc @ 16mhz
    config2 = 0xD0; //config2 = 0x90;
    config3 = 0x82; //config3 = 0x02;
  }

  writeAddress(CNF1, config1);//Write config address 1
  writeAddress(CNF2, config2);//Write config address 2
  writeAddress(CNF3, config3);//Write config address 3


}

unsigned short CAN_MCP2515::readRate()
{
  byte config1, config2, config3;

  config1 = readAddress(CNF1);
  config2 = readAddress(CNF2);
  config3 = readAddress(CNF3);

  if ((config1 == 0x31) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 10;
  } else if ((config1 == 0x18) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 20;
  } else if ((config1 == 0x09) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 50;
  } else if ((config1 == 0x04) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 100;
  } else if ((config1 == 0x03) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 125;
  } else if ((config1 == 0x01) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 250;
  } else if ((config1 == 0x00) && (config2 == 0xB8) && (config3 == 0x05)) {
    return 500;
  } else if ((config1 == 0x00) && (config2 == 0xD0) && (config3 == 0x82)) {
    return 1000;
  }

  else {
    return 0;
  }
}


//Start MCP2515 communications
void CAN_MCP2515::begin(byte mode, int CANspeed)
{
  SPI.begin();//SPI communication begin

  reset();//Set MCP2515 into Config mode by soft reset. Note MCP2515 is in Config mode by default at power up.

  bitRate(CANspeed);//Set CAN bit rate

  setMode(mode);//Set CAN mode

}

//Start MCP2515 communications; default to Normal mode
void CAN_MCP2515::begin(int CANspeed)
{
  SPI.begin(); //SPI communication begin

  reset(); //Set MCP2515 into Config mode by soft reset. Note MCP2515 is in Config mode by default at power up.

  bitRate(CANspeed); //Set CAN bit rate

  setMode(MCP2515_MODE_NORMAL); //Set CAN mode
}

//Turns RX filters/masks off. Will receive any message.
void CAN_MCP2515::clearFilters ()
{

  byte writeVal, mask;

  writeVal = 0x60;
  mask = 0x60;

  modifyAddress(RXB0CTRL, mask, writeVal);
  modifyAddress(RXB1CTRL, mask, writeVal);
}

//Set Masks for filters
void CAN_MCP2515::setMask(byte mask, byte data0, byte data1, byte data2, byte data3)
{
  setMode(MCP2515_MODE_CONFIG);
  writeAddress(mask, data0);
  writeAddress(mask + 1, data1);
  writeAddress(mask + 2, data2);
  writeAddress(mask + 3, data3);
  setMode(MCP2515_MODE_NORMAL);
}

// Set Receive Filters. Will think of a more user friendly way to set these in the future but for right now it works....
void CAN_MCP2515::setFilter(byte filter, byte data0, byte data1, byte data2, byte data3)
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

  byte writeVal = 0x00;

  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (RXB0SIDH);
  for (byte i = 0; i < 13; i++) {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);

  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (RXB1SIDH);
  for (byte i = 0; i < 13; i++) {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
}

// This loads buffers with zeros to prevent incorrect data to be sent. Note: It RTS is sent to a buffer that has all zeros it will still send a message with all zeros.
void CAN_MCP2515::clearTxBuffers()
{

  byte writeVal = 0x00;

  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (TXB0SIDH);
  for (byte i = 0; i < 13; i++) {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);


  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (TXB1SIDH);
  for (byte i = 0; i < 13; i++) {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);

  digitalWrite(CS, LOW);
  SPI.transfer (MCP2515_SPI_WRITE);
  SPI.transfer (TXB2SIDH);
  for (byte i = 0; i < 13; i++) {
    SPI.transfer(writeVal);
  }
  digitalWrite(CS, HIGH);
}


//(RTS) Request to send TX buffer X.
//MCP2515 buffers are zero indexed. To make it more user friendly it has been changed to buffer 1,2,3 on user input side.
//A value of 0 or SEND_ALL will send all buffers, not buffer 0.

void CAN_MCP2515::sendTx(byte buffer)//transmits buffer
{
  byte txBuffer;

  if (buffer == buffer1) {
    txBuffer = SEND_TX_BUF_0;
  }
  else if (buffer == buffer2) {
    txBuffer = SEND_TX_BUF_1;
  }
  else if (buffer == buffer3) {
    txBuffer = SEND_TX_BUF_2;
  }
  else if (buffer == allBuffers) {
    txBuffer = SEND_ALL;
  }

  digitalWrite(CS, LOW);
  SPI.transfer(txBuffer);
  digitalWrite(CS, HIGH);
}

//Function that reads several status bits for transmit and receive functions.

byte CAN_MCP2515::readStatus()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_READ_STATUS);
  byte retVal = SPI.transfer(0xFF);
  digitalWrite(CS, HIGH);
  return retVal;

  /*
  bit 7: CANINTF.RX0IF
  bit 6: CANINTF.RX1IF
  bit 5: TXB0CNTRL.TXREQ
  bit 4: CANINTF.TX0IF
  bit 3: TXB1CNTRL.TXREQ
  bit 2: CANINTF.TX1IF
  bit 1: TXB2CNTRL.TXREQ
  bit 0: CANINTF.TX2IF

  (readStatus() & 0x01) == 0x01 means message in RX buffer 0
  (readStatus() & 0x02) == 0x02 means message in RX buffer 1
  (readStatus() & 0x03) == 0x03 means message in TX buffer 0
  (readStatus() & 0x04) == 0x04 means Transmit Buffer 0 Empty Interrupt Flag set
  (readStatus() & 0x05) == 0x05 means message in TX buffer 1
  (readStatus() & 0x06) == 0x06 means Transmit Buffer 1 Empty Interrupt Flag set
  (readStatus() & 0x07) == 0x07 means message in TX buffer 2
  (readStatus() & 0x08) == 0x08 means Transmit Buffer 2 Empty Interrupt Flag set
  */
}

//Function that reads receive functions and filhits
byte CAN_MCP2515::readRXStatus()
{
  digitalWrite(CS, LOW);
  SPI.transfer(MCP2515_SPI_RX_STATUS);
  byte retVal = SPI.transfer(0xFF);
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

// Allows you to load message to specific Buffers. RTS is needed for message to be sent.
void CAN_MCP2515::loadMsg(byte buffer, unsigned long ID, byte frameType, byte length, byte *data)
{
  byte i, id_high, id_low, ex_high, ex_low, loadBuffer;

  //Determine what buffer has been selected

  if (buffer == 1) {
    loadBuffer = LOAD_TX_BUF_0_ID;
  }
  else if (buffer == 2) {
    loadBuffer = LOAD_TX_BUF_1_ID;
  }
  else if (buffer == 3) {
    loadBuffer = LOAD_TX_BUF_2_ID;
  }

  // Determine if this is an extended msg
  if (frameType == extID) {

    //generate id bytes before SPI write
    id_high = (ID >> 21);
    id_low = (ID >> 18);
    id_low = ((id_low) << 5 | 0x08);
    if (bitRead ((ID), 17) == 1) bitSet(id_low, 1);
    if (bitRead ((ID), 16) == 1) bitSet(id_low, 0);
    ex_high = (ID) >> 8;
    ex_low = (ID) >> 0;

    //Loads buffer with data
    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(ex_high); //extended ID high bits
    SPI.transfer(ex_low); //extended ID low bits
    SPI.transfer(length);
    for (i = 0; i < length; i++) { //load data buffer
      SPI.transfer(data[i]);
    }
    digitalWrite(CS, HIGH);
  }

  // If this is a standard message then...
  else if (frameType == stdID) {

    id_high = (byte) (ID >> 3);
    id_low = (byte) ((ID << 5) & 0x00E0);

    //generate id bytes before SPI write
    id_high = (byte) (ID >> 3);
    id_low = (byte) ((ID << 5) & 0x00E0);

    digitalWrite(CS, LOW);
    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(0x00); //extended ID registers, not used in standard frames
    SPI.transfer(0x00); //extended ID registers, not used in standard frames
    SPI.transfer(length);
    for (i = 0; i < length; i++) { //load data buffer
      SPI.transfer(data[i]);
    }

    digitalWrite(CS, HIGH);
  }
}

//Enable hardware Request to send pins. It allows messages to be send by driving RTS pins low.
//These are not to be confused with RTS commands. These are the actual MCP2515 hardware pins
void CAN_MCP2515::enableRTSPins ()
{

  byte writeVal = 0x07;
  writeAddress(TXRTSCTRL, writeVal);
}

// Enable interrupts. The CANINTF register contains the corresponding interrupt flag bit for
// each interrupt source. When an interrupt occurs, the INT pin is driven low by the MCP2515
// and will remain low until the interrupt is cleared by the MCU. An interrupt can not be
// cleared if the respective condition still prevails.

void CAN_MCP2515::setInterrupts(byte mask, byte writeVal)
{

  modifyAddress(CANINTE, mask, writeVal);

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


// Function to load and send any message. (J1939, CANopen, CAN). It assumes user knows what the ID is supposed to be
void CAN_MCP2515::send(unsigned long ID, byte frameType, byte length, byte *data)
{

  byte i, id_high, id_low, ex_high, ex_low, status, loadBuffer, sendBuffer;

  status = readStatus() & 0x54;

  if (status != 0x54) {
    if ((status & 0x04) == 0) { //transmit buffer 0 is open
      loadBuffer = LOAD_TX_BUF_0_ID;
      sendBuffer = SEND_TX_BUF_0;
    }
    else if ((status & 0x10) == 0) { //transmit buffer 1 is open
      loadBuffer = LOAD_TX_BUF_1_ID;
      sendBuffer = SEND_TX_BUF_1;
    }
    else { // transmit buffer 2 is open
      loadBuffer = LOAD_TX_BUF_2_ID;
      sendBuffer = SEND_TX_BUF_2;
    }
  }


  //if(bitRead ((ID),27)==1){// might use this later to remove Frame Type. It works for right now....
  if (frameType == extID) {

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
    for (i = 0; i < length; i++) { //load data buffer
      SPI.transfer(data[i]);
    }
  }

  else if (frameType == stdID) {

    id_high = (byte) (ID >> 3);
    id_low = (byte) ((ID << 5) & 0x00E0);

    digitalWrite(CS, LOW);

    SPI.transfer(loadBuffer);
    SPI.transfer(id_high); //ID high bits
    SPI.transfer(id_low); //ID low bits
    SPI.transfer(0x00); //extended ID registers
    SPI.transfer(0x00);
    SPI.transfer(length); //data length code
    for (i = 0; i < length; i++) { //load data buffer
      SPI.transfer(data[i]);
    }
  }

  digitalWrite(CS, HIGH);

  digitalWrite(CS, LOW);
  SPI.transfer(sendBuffer);
  digitalWrite(CS, HIGH);

}

// Receive and display any message (J1939, CANopen, CAN). This functions provides an easy way to see the message if user doesn't care about the actual message protocol. No message struct is used here.
void CAN_MCP2515::read(unsigned long *ID, byte *length_out, byte *data_out)
{

  byte len, i, buffer, status;

  unsigned short sid_h, sid_l, eid8, eid0, temp;

  status = readStatus();

  if ((status & 0x01) == 0x01) {
    buffer = 0x90;
  }
  else if ((status & 0x02) == 0x02) {

    buffer = 0x94;

  }

  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++) {
    data_out[i] = SPI.transfer(0xFF);
  }

  digitalWrite(CS, HIGH);

  (*length_out) = len;

  if (bitRead ((sid_l), 3) == 1) { // check to see if this is an Extended ID Msg.

    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((unsigned short) sid_h >> 3); //shift SIDH

    (*ID) = (((unsigned long) sid_h << 24) | ((unsigned long) sid_l << 16 ) | ((unsigned long) eid8 << 8) | (eid0 << 0)); // repack message
  }

  else {
    (*ID) = ((((unsigned short) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame.
  }
}

//Receive and display J1939 messages. This allows use of the message structure for easier message handling if PGN, SA and DA are needed.
void CAN_MCP2515::read(CAN_DATA_FRAME_J1939 *message)
{

  byte len, i, buffer, status;

  unsigned short sid_h, sid_l, eid8, eid0, temp, rxbDlc;

  status = readStatus();

  if ((status & 0x01) == 0x01) {
    buffer = 0x90;
  }
  else if ((status & 0x02) == 0x02) {

    buffer = 0x94;

  }

  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  rxbDlc = SPI.transfer(0xFF); // get bits and other data from MCP2515 RXB DLC buffer (it contains data and RTR)
  len = ((rxbDlc) & 0x0F);
  //len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++) {
    message -> data[i] = SPI.transfer(0xFF);
  }

  digitalWrite(CS, HIGH);

  message -> DLC = len;

  if (bitRead ((sid_l), 3) == 1) { // check to see if this is an Extended ID Msg.

    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((unsigned short) sid_h >> 3); //shift SIDH

    message->ID = (((unsigned long) sid_h << 24) | ((unsigned long) sid_l << 16 ) | ((unsigned long) eid8 << 8) | (eid0 << 0)); // repack message ID
    message->PRIO = ((unsigned long) sid_h >> 2); // send this to display priority
    message->PGN = (((unsigned short) sid_l << 8 ) | ((unsigned short) eid8 << 0));// send this to display PGN
    message->SA = eid0;// send this to display Source Address
    message->DA = eid8;// send this to display Destination Address

  }

  //Standard frames not supported in J1939
  else {

    message->ID = 0; // not supported in J1939
    message->PRIO = 0;// not supported in J1939
    message->PGN = 0;// not supported in J1939
    message->SA = 0;// not supported in J1939
    message->DA = 0;// not supported in J1939
    message->DLC = 0;// not supported in J1939
  }
}

//Receive and display CANopen message. This allows use of the message structure for easier message handling if
//communication object identifier (COB_ID), Function Code and Nodes are needed. See link for more info http://en.wikipedia.org/wiki/CANopen

void CAN_MCP2515::read(CAN_DATA_FRAME_CANopen *message)
{

  byte len, i, buffer, status;

  unsigned short sid_h, sid_l, eid8, eid0, rxbDlc;

  status = readStatus();

  if ((status & 0x01) == 0x01) {
    buffer = 0x90;
  }
  else if ((status & 0x02) == 0x02) {

    buffer = 0x94;

  }

  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  rxbDlc = SPI.transfer(0xFF); // get bits and other data from MCP2515 RXB DLC buffer (it contains data and RTR)
  len = ((rxbDlc) & 0x0F);
  //len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++) {
    message->data[i] = SPI.transfer(0xFF);
  }

  digitalWrite(CS, HIGH);

  message->rtr = (bitRead(rxbDlc, 6));
  message->length = len;
  message->COB_ID = ((((unsigned short) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame. COB_ID
  message->FC = (((unsigned short) sid_h) << 3);// Contains CAN open Function Code
  message->NODE = ((sid_l & 0xE0) >> 5); // Contains CANopen Node

}

//Receive and display CAN message. This allows use of the message structure for easier message handling. Note: This fucntion does not allow extended frames as by default CAN is standard frames.
void CAN_MCP2515::read(CAN_DATA_FRAME *message)
{

  byte len, i, buffer, status;

  unsigned short sid_h, sid_l, eid8, eid0, temp, rxbDlc;

  status = readStatus();

  if ((status & 0x01) == 0x01) {
    buffer = 0x90;
  }
  else if ((status & 0x02) == 0x02) {

    buffer = 0x94;

  }

  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  rxbDlc = SPI.transfer(0xFF); // get bits and other data from MCP2515 RXB DLC buffer (it contains data and RTR)
  len = ((rxbDlc) & 0x0F);
  //len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++) {
    message -> data[i] = SPI.transfer(0xFF);
  }

  digitalWrite(CS, HIGH);

  message -> length = len;
  message -> rtr = (bitRead(rxbDlc, 6));

  if (bitRead ((sid_l), 3) == 1) { // check to see if this is an Extended ID Msg.

    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((unsigned short) sid_h >> 3); //shift SIDH

    message->ID = (((unsigned long) sid_h << 24) | ((unsigned long) sid_l << 16 ) | ((unsigned long) eid8 << 8) | (eid0 << 0)); // repack message ID
  }

  else {
    message->ID = ((((unsigned short) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame.
  }
}


// Check to see if message is available
bool CAN_MCP2515::msgAvailable()
{

  byte msgStatus = readStatus();

  if (((msgStatus & 0x01) == 0x01) || ((msgStatus & 0x02) == 0x02))
  {
    return true;
  }
  else
  {
    return false;
  }
}

#endif // defined(ARDUINO_ARCH_AVR)
