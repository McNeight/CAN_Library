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
Stevenh for his work on library and all of the MCP research/work http://modelrail.otenko.com/arduino/arduino-controller-area-network-can

-------------------------------------------------------------------------------------------------------------
Change Log

DATE		VER		WHO			WHAT
07/07/13	0.1		PC		Modified and merge all MCP2515 libraries found. Stripped away most unused functions and corrected MCP2515 defs
09/12/13	0.2		PC		Added selectable CS SPI for CAN controller to use 1 IC to control several mcp2515
02/05/14	0.3		PC		Added filter and mask controls
05/01/14	0.4		PC		Cleaned up functions, variables and added message structures for J1939, CANopen and CAN.
05/07/14	1.0		PC		Released library to public through GitHub
-------------------------------------------------------------------------------------------------------------

*/


#ifndef _CAN_H_
#define _CAN_H_

#include <inttypes.h>
#include <Arduino.h>

// From the spec
#define CAN_DOMINANT    0
#define CAN_RECESSIVE   1

// Base and Extended ID defines
#define CAN_BASE_FRAME                11
#define CAN_EXTENDED_FRAME            29

// Define the typical bitrate for CAN communication in kbps.
#define CAN_BPS_1M                    1000000
#define CAN_BPS_1000K                 1000000
#define CAN_BPS_800K                  800000
#define CAN_BPS_500K                  500000
#define CAN_BPS_250K                  250000
#define CAN_BPS_125K                  125000
#define CAN_BPS_100K                  100000
#define CAN_BPS_50K                   50000
#define CAN_BPS_33333				  33333
#define CAN_BPS_25K                   25000
#define CAN_BPS_20K                   20000
#define CAN_BPS_10K                   10000
#define CAN_BPS_5K                    5000

// CAN Message Structures
// typedef struct
// {
// byte start_of_frame : 1;
// unsigned int identifier : 11;
// byte rtr : 1;
// byte dlc : 4;
// unsigned int crc : 15;
// byte ack : 2;
// byte eof : 7;
// } CAN_DATA_FRAME_COMPLETE;

#if defined(ARDUINO_ARCH_SAM)
//This is architecture specific. DO NOT USE THIS UNION ON ANYTHING OTHER THAN THE CORTEX M3 / Arduino Due
//UNLESS YOU DOUBLE CHECK THINGS!
typedef union {
  uint64_t value;
  struct {
    uint32_t low;
    uint32_t high;
  };
  struct {
    uint16_t s0;
    uint16_t s1;
    uint16_t s2;
    uint16_t s3;
  };
  uint8_t bytes[8];
} BytesUnion;

typedef struct
{
  uint32_t id : 29;		// EID if ide set, SID otherwise
  uint32_t fid;		// family ID
  uint8_t rtr : 1;		// Remote Transmission Request
  uint8_t priority;	// Priority but only important for TX frames and then only for special uses.
  uint8_t extended : 1;	// Extended ID flag
  uint8_t length : 4;		// Number of data bytes
  BytesUnion data;	// 64 bits - lots of ways to access it.
} CAN_FRAME;

#elif defined(ARDUINO_ARCH_AVR)

typedef struct
{
  uint32_t id : 29;		// EID if ide set, SID otherwise
  uint32_t fid;		// family ID
  uint8_t rtr : 1; 			// Remote Transmission Request
  uint8_t priority;	// Priority but only important for TX frames and then only for special uses.
  uint8_t extended : 1;	// Extended ID flag
  uint8_t length : 4; 		// Data Length
  uint8_t data[8]; 			// Message data
} CAN_FRAME;

#endif // CAN_FRAME

// SAE J1939 Message Structures
// Also:
//   ISO 11783 (ISOBUS or ISO Bus)
//   NMEA 2000
typedef struct
{
  uint32_t ID : 29; 			// Identifier
  uint8_t PRIO : 3; 		// Message priority
  uint16_t PGN; 	// Parameter Group Number
  uint8_t SA; 		// Source Address
  uint8_t DA; 		// Destination Address
  uint8_t DLC : 4; 		// Data length code
  uint8_t data[8];		// Message data
} CAN_DATA_FRAME_J1939;


// CANopen Message Structures
typedef struct
{
  uint16_t COB_ID		: 11; 	// Communication object identifier
  uint8_t FC			: 4;	// Function Code
  uint8_t NODE		: 7;	// Node
  uint8_t rtr			: 1;	// Remote Transmission Request
  uint8_t length		: 4;	// Data Length
  uint8_t data[8]; 			// Message data
} CAN_DATA_FRAME_CANopen;


// CANaerospace Message Structures
typedef struct
{
  uint16_t ID : 11;
  uint8_t NODE;
  uint8_t TYPE;
  uint8_t SERVICE;
  uint8_t MESSAGE;
  uint8_t data[4];
} CAN_DATA_FRAME_CANaerospace;

class CANClass // Can't inherit from Stream
{
  public:
    virtual void begin(uint32_t baud);
    virtual void end();
    virtual uint8_t available();
    virtual CAN_FRAME read();
    virtual void flush();
    virtual uint8_t write(CAN_FRAME&);

    //CAN_FRAME& operator=(const CAN_FRAME&);
};

//extern CANClass CAN;

#endif // _CAN_H_
