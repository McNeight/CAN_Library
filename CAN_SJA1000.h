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


#if defined(CAN_CONTROLLER_SJA1000)

#ifndef _CAN_SJA1000_H_
#define _CAN_SJA1000_H_

#include <Arduino.h>
#include <SPI.h>
#include "CAN.h"

/**
 * \name	Adressen der Register des SJA1000 im Intel Mode
 *
 * PeliCAN Modus
 */
/*@{*/

#define MOD			0x00 // mode
#define CR			0x00 // control

#define CMR			0x01 // command
#define SR			0x02 // status
#define IR			0x03 // interrupt

#define BTR0		0x06 // bit timing 0
#define BTR1		0x07 // bit timing 1
#define OCR			0x08 // output control
#define CDR			0x1F // clock divider

#define IER			0x04 // interrupt enable

#define ALC			0x0B // arbitration lost capture
#define ECC			0x0C // error code capture
#define EWL			0x0D // error_warning_limit
#define RXERR		0x0E // RX error counter
#define TXERR		0x0F // TX error counter

#define RMC			0x1D // RX message counter
#define RBSA		0x1E // RX_buffer_start_adress

#define ACR0		0x10 // acceptance code0
#define ACR1		0x11 // acceptance code1
#define ACR2		0x12 // acceptance code2
#define ACR3		0x13 // acceptance code3
#define AMR0		0x14 // acceptance mask0
#define AMR1		0x15 // acceptance mask1
#define AMR2		0x16 // acceptance mask2
#define AMR3		0x17 // acceptance mask3

#define RX_INFO		0x10
#define RX_ID1		0x11
#define RX_ID0		0x12
#define RX_DATA0	0x13
#define RX_DATA1	0x14
#define RX_DATA2	0x15
#define RX_DATA3	0x16
#define RX_DATA4	0x17
#define RX_DATA5	0x18
#define RX_DATA6	0x19
#define RX_DATA7	0x1A

#define TX_INFO		0x10
#define TX_ID1		0x11
#define TX_ID0		0x12
#define TX_DATA0	0x13
#define TX_DATA1	0x14
#define TX_DATA2	0x15
#define TX_DATA3	0x16
#define TX_DATA4	0x17
#define TX_DATA5	0x18
#define TX_DATA6	0x19
#define TX_DATA7	0x1A

/*@}*/

/**
 * \name	Bitdefinition der verschiedenen Register
 */
/*@{*/

/**
 * \brief	Bitdefinition von MOD
 */
#define SM			4
#define AFM			3
#define STM			2
#define LOM			1
#define RM			0

/**
 * \brief	Bitdefinition von CMR
 */
#define SRR			4
#define CDO			3
#define RRB			2
#define AT			1
#define TR			0

/**
 * \brief	Bitdefinition von SR
 */
#define BS			7
#define ES			6
#define TS			5
#define RS			4
#define TCS			3
#define TBS			2
#define DOS			1
#define RBS			0

/**
 * \brief	Bitdefinition von IR
 */
#define BEI			7
#define ALI			6
#define EPI			5
#define WUI			4
#define DOI			3
#define EI			2
#define TI			1
#define RI			0

/**
* \brief	Bitdefinition von IER / CR
 */
#define BEIE		7
#define ALIE		6
#define EPIE		5
#define WUIE		4
#define DOIE		3
#define EIE			2
#define TIE			1
#define RIE			0

/**
 * \brief	Bitdefinition von BTR0
 */
#define _SJW1		7
#define _SJW0		6
#define _BRP5		5
#define _BRP4		4
#define _BRP3		3
#define _BRP2		2
#define _BRP1		1
#define _BRP0		0

/**
 * \brief	Bitdefinition von BTR1
 */
#define SAM			7
#define TSEG22		6
#define TSEG21		5
#define TSEG20		4
#define TSEG13		3
#define TSEG12		2
#define TSEG11		1
#define TSEG10		0

/**
 * \brief	Bitdefinition von OCR
 */
#define OCTP1		7
#define OCTN1		6
#define OCPOL1		5
#define OCTP0		4
#define OCTN0		3
#define OCPOL0		2
#define OCMODE1		1
#define OCMODE0		0

/**
 * \brief	Bitdefinition von ALC
 */
#define BITNO4		4
#define BITNO3		3
#define BITNO2		2
#define BITNO1		1
#define BITNO0		0

/**
 * \brief	Bitdefinition von ECC
 */
#define ERRC1		7
#define ERRC0		6
#define DIR			5
#define SEG4		4
#define SEG3		3
#define SEG2		2
#define SEG1		1
#define SEG0		0

/**
 * \brief	Bitdefinition von EWL
 */
#define ERRC1		7
#define ERRC0		6
#define DIR			5
#define SEG4		4
#define SEG3		3
#define SEG2		2
#define SEG1		1
#define SEG0		0

/**
 * \brief	Bitdefinition von CDR
 */
#define CANMODE		7
#define CBP			6
#define RXINTEN		5
#define CLKOFF		3
#define CD2			2
#define CD1			1
#define CD0			0

/**
 * \brief   Bitdefinition von RX_INFO und TX_INFO
 */
#define	FF			7
#define	RTR			6

/*@}*/

// check if the settings are correct
#if SUPPORT_EXTENDED_CANID == 0
#error	Extended CANIDs need to be supported!
#endif

#ifndef	SJA1000_INT
#error	SJA1000_INT	is not defined!
#endif

#ifndef SJA1000_CLKOUT_PRESCALER
#define	SJA1000_CLOCK_REGISTER		(1<<CLKOFF)
#else
#if SJA1000_CLKOUT_PRESCALER == 1
#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD1)|(1<<CD0))
#elif SJA1000_CLKOUT_PRESCALER == 2
#define	SJA1000_CLOCK_REGISTER		0
#elif SJA1000_CLKOUT_PRESCALER == 4
#define	SJA1000_CLOCK_REGISTER		(1<<CD0)
#elif SJA1000_CLKOUT_PRESCALER == 6
#define	SJA1000_CLOCK_REGISTER		(1<<CD1)
#elif SJA1000_CLKOUT_PRESCALER == 8
#define	SJA1000_CLOCK_REGISTER		((1<<CD1)|(1<<CD0))
#elif SJA1000_CLKOUT_PRESCALER == 10
#define	SJA1000_CLOCK_REGISTER		(1<<CD2)
#elif SJA1000_CLKOUT_PRESCALER == 12
#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD0))
#elif SJA1000_CLKOUT_PRESCALER == 14
#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD1))
#endif
#endif

#if SJA1000_MEMORY_MAPPED
#ifndef	SJA1000_BASE_ADDR
#error	SJA1000_BASE_ADDR is not defined!
#endif

#define	SUPPORT_FOR_SJA1000__		1

// write to a register
static inline void sja1000_write(uint8_t address, uint8_t data) {
  (*((uint8_t *) (SJA1000_BASE_ADDR + address))) = data;
}

// read a register
static inline uint8_t sja1000_read(uint8_t address) {
  return (*((uint8_t *) (SJA1000_BASE_ADDR + address)));
}

#else
#warning    not tested yet!

#if !defined(SJA1000_WR) || !defined(SJA1000_RD) || \
			!defined(SJA1000_CS) || !defined(SJA1000_DATA) || !defined(SJA1000_ALE)
#error in definition of SJA1000-pins (check SJA1000_WR, SJA1000_RD, SJA1000_CS, SJA1000_DATA and SJA1000_ALE)!
#endif

#define	SUPPORT_FOR_SJA1000__		1
extern void sja1000_write(uint8_t address, uint8_t data);
extern uint8_t sja1000_read(uint8_t address);
#endif	// SJA1000_MEMORY_MAPPED


// MCP class
class CAN_SJA1000 : public CANClass
{
  public:


    CAN_SJA1000(byte CS_Pin);// SPI CS is selectable through sketch. Allows multiple CAN channels.
    void begin (byte mode, int CANspeed);// Initializes CAN communications. Note it also starts SPI communications
    void begin (int CANspeed);// Initializes CAN communications into Normal mode. Note it also starts SPI communications
    void setBitrate(int CANspeed);//sets up CAN bit rate
    void setMode(byte mode) ;//puts CAN controller in one of five modes
    void reset(); //CAN software reset. Also puts MCP2515 into config mode
    byte getMode(); // reads CAN mode
    unsigned short getBitrate(); // reads CANspeed

    void writeAddress(byte address, byte value);// writes MCP2515 register addresses
    byte readAddress(byte address); //reads MCP2515 registers
    void modifyAddress(byte address, byte mask, byte value); // MCP2515 SPI bit modification commands

    void clearRxBuffers(); // clears all receive buffers
    void clearTxBuffers(); // clears all receive buffers
    void clearFilters();   // clears all filters and masks

    byte readStatus(); //reads several status bits for transmit and receive functions.
    byte readRXStatus(); //reads receive functions and filhits

    void loadMsg(byte buffer, unsigned long ID, byte frameType, byte length, byte *data); //Load Standard Data Frame Message into TX buffer X. Note this only load message to buffer. RTS is needed to send message

    void sendTx(byte buffer); //(RTS) Request to send individual TX buffers or all

    void send(unsigned long ID, byte frameType, byte length, byte *data); // Load and send message. No RTS needed.

    void read(unsigned long *ID, byte *length_out, byte *data_out); // Receive and display any message (J1939, CANopen, CAN)
    void read(CAN_DATA_FRAME *message); //Receive and display CAN message and allows use of the message structure for easier message handling
    void read(CAN_DATA_FRAME_J1939 *message); //Receive and display J1939 message and allows use of the message structure for easier message handling
    void read(CAN_DATA_FRAME_CANopen *message); //Receive and display CANopen message and allows use of the message structure for easier message handling

    bool msgAvailable(); // check if message has been received on any of the buffers


    // OTHER USEFUL FUNCTIONS

    void enableRTSPins (); // Enable hardware request to send (RTS) pins if they are available. It allows messages to be send by driving MCP2515 RTS pins low.
    void setInterrupts(byte mask, byte writeVal); //Enable/disable interrupts
    void setMask(byte mask, byte b0, byte b1, byte b2, byte b3); // Set Masks for filters
    void setFilter(byte filter, byte b0, byte b1, byte b2, byte b3); //Set Receive filters

  private:
    byte CS; //SPI CS is selectable through sketch

};

#endif // _CAN_SJA1000_H_

#endif // defined(ARDUINO_ARCH_AVR)
