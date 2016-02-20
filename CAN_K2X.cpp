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


#if defined(__MK20DX256__) // Teensy 3.1

#include "CAN_K2X.h"

static const int txb = 8; // with default settings, all buffers before this are consumed by the FIFO
static const int txBuffers = 8;
static const int rxb = 0;

// -------------------------------------------------------------
CAN_K2X::CAN_K2X()
{
  // set up the pins, 3=PTA12=CAN0_TX, 4=PTA13=CAN0_RX
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);// | PORT_PCR_PE | PORT_PCR_PS;
  // select clock source external 16MHz oscillator
  OSC0_CR |= OSC_ERCLKEN;
  SIM_SCGC6 |=  SIM_SCGC6_FLEXCAN0;
  FLEXCAN0_CTRL1 &= ~FLEXCAN_CTRL_CLK_SRC;

  // enable CAN
  FLEXCAN0_MCR |=  FLEXCAN_MCR_FRZ;
  FLEXCAN0_MCR &= ~FLEXCAN_MCR_MDIS;
  while (FLEXCAN0_MCR & FLEXCAN_MCR_LPM_ACK)
    ;
  // soft reset
  FLEXCAN0_MCR ^=  FLEXCAN_MCR_SOFT_RST;
  while (FLEXCAN0_MCR & FLEXCAN_MCR_SOFT_RST)
    ;
  // wait for freeze ack
  while (!(FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK))
    ;
  // disable self-reception
  FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;

  //enable RX FIFO
  FLEXCAN0_MCR |= FLEXCAN_MCR_FEN;

  // Default mask is allow everything
  defaultMask.rtr = 0;
  defaultMask.extended = 0;
  defaultMask.id = 0;
}


// -------------------------------------------------------------
void CAN_K2X::end(void)
{
  // enter freeze mode
  FLEXCAN0_MCR |= (FLEXCAN_MCR_HALT);
  while (!(FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK))
    ;
}


//Can find more timings at https://code.google.com/p/peninsula/source/browse/trunk/Kinetis512/kinetis-sc/src/projects/sci2can/can.c?r=16
//search for "switch (baudrateKHz)" on that page
// -------------------------------------------------------------
void CAN_K2X::begin(uint32_t bitrate)
{
   if ( 33333 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(29));
  }
  else if ( 50000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(19));
  }
  else if ( 83333 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(11));
  }
  else if ( 100000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(9));
  }
  else if ( 250000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(3));
  }
  else if ( 500000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(1));
  }
  else if ( 1000000 == bitrate )
  {
   FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(0)
                      | FLEXCAN_CTRL_PSEG1(1) | FLEXCAN_CTRL_PSEG2(1) | FLEXCAN_CTRL_PRESDIV(1));
  }
  else     // default to 125_000
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(7) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(7));
  }

  FLEXCAN0_RXMGMASK = 0;

  //enable reception of all messages that fit the mask
  // if (mask.extended)
  // {
  // FLEXCAN0_RXFGMASK = ((mask.rtr ? 1 : 0) << 31) | ((mask.extended ? 1 : 0) << 30) | ((mask.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
  // }
  // else
  // {
  // FLEXCAN0_RXFGMASK = ((mask.rtr ? 1 : 0) << 31) | ((mask.extended ? 1 : 0) << 30) | (FLEXCAN_MB_ID_IDSTD(mask.id) << 1);
  // }
  FLEXCAN0_RXFGMASK = 0;

  // start the CAN
  FLEXCAN0_MCR &= ~(FLEXCAN_MCR_HALT);
  // wait till exit of freeze mode
  while (FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);

  // wait till ready
  while (FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY);

  //set tx buffers to inactive
  for (int i = txb; i < txb + txBuffers; i++)
  {
    FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}


// -------------------------------------------------------------
void CAN_K2X::setFilter(const CAN_Filter &filter, uint8_t n)
{
  if ( 8 > n )
  {
    if (filter.extended)
    {
      FLEXCAN0_IDFLT_TAB(n) = ((filter.rtr ? 1 : 0) << 31) | ((filter.extended ? 1 : 0) << 30) | ((filter.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
    }
    else
    {
      FLEXCAN0_IDFLT_TAB(n) = ((filter.rtr ? 1 : 0) << 31) | ((filter.extended ? 1 : 0) << 30) | (FLEXCAN_MB_ID_IDSTD(filter.id) << 1);
    }
  }
}


// -------------------------------------------------------------
uint8_t CAN_K2X::available(void)
{
  //In FIFO mode, the following interrupt flag signals availability of a frame
  return (FLEXCAN0_IFLAG1 & FLEXCAN_IMASK1_BUF5M) ? 1 : 0;
}


// -------------------------------------------------------------
CAN_Frame CAN_K2X::read()
{
  unsigned long int startMillis;
  CAN_Frame msg;
  msg.timeout = CAN_TIMEOUT_T3;

  startMillis = msg.timeout ? millis() : 0;

  while ( !available() )
  {
    if ( !msg.timeout || (msg.timeout <= (millis() - startMillis)) )
    {
      // early EXIT nothing here
      msg.valid = false;
      return msg;
    }
    yield();
  }

  // get identifier and dlc
  msg.length = FLEXCAN_get_length(FLEXCAN0_MBn_CS(rxb));
  msg.extended = (FLEXCAN0_MBn_CS(rxb) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
  msg.id  = (FLEXCAN0_MBn_ID(rxb) & FLEXCAN_MB_ID_EXT_MASK);
  if (!msg.extended)
  {
    msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }

  // copy out message
  uint32_t dataIn = FLEXCAN0_MBn_WORD0(rxb);
  msg.data[3] = dataIn;
  dataIn >>= 8;
  msg.data[2] = dataIn;
  dataIn >>= 8;
  msg.data[1] = dataIn;
  dataIn >>= 8;
  msg.data[0] = dataIn;
  if ( 4 < msg.length )
  {
    dataIn = FLEXCAN0_MBn_WORD1(rxb);
    msg.data[7] = dataIn;
    dataIn >>= 8;
    msg.data[6] = dataIn;
    dataIn >>= 8;
    msg.data[5] = dataIn;
    dataIn >>= 8;
    msg.data[4] = dataIn;
  }
  for ( int loop = msg.length; loop < 8; ++loop )
  {
    msg.data[loop] = 0;
  }

  //notify FIFO that message has been read
  FLEXCAN0_IFLAG1 = FLEXCAN_IMASK1_BUF5M;

  msg.valid = true;
  return msg;
}

void CAN_K2X::flush()
{
}

// -------------------------------------------------------------
uint8_t CAN_K2X::write(const CAN_Frame &msg)
{
  unsigned long int startMillis;

  // Might as well check right off the bat
  if (msg.valid == false)
  {
    return 0;
  }
  startMillis = msg.timeout ? millis() : 0;

  // find an available buffer
  int buffer = -1;
  for ( int index = txb; ; )
  {
    if ((FLEXCAN0_MBn_CS(index) & FLEXCAN_MB_CS_CODE_MASK) == FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
    {
      buffer = index;
      break;// found one
    }
    if ( !msg.timeout )
    {
      if ( ++index >= (txb + txBuffers) )
      {
        return 0;// early EXIT no buffers available
      }
    }
    else
    {
      // blocking mode, only 1 txb used to guarantee frames in order
      if ( msg.timeout <= (millis() - startMillis) )
      {
        return 0; // timed out
      }
      yield();
    }
  }

  // transmit the frame
  FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  if (msg.extended)
  {
    FLEXCAN0_MBn_ID(buffer) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
  }
  else
  {
    FLEXCAN0_MBn_ID(buffer) = FLEXCAN_MB_ID_IDSTD(msg.id);
  }
  FLEXCAN0_MBn_WORD0(buffer) = (msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
  FLEXCAN0_MBn_WORD1(buffer) = (msg.data[4] << 24) | (msg.data[5] << 16) | (msg.data[6] << 8) | msg.data[7];
  if (msg.extended)
  {
    FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                              | FLEXCAN_MB_CS_LENGTH(msg.length) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  else
  {
    FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                              | FLEXCAN_MB_CS_LENGTH(msg.length);
  }

  return 1;
}

CAN_K2X CAN; // Create CAN channel

#endif // defined(__MK20DX256__) // Teensy 3.1
