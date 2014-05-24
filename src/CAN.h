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

Features:

CAN V2.0B
8 byte length in the data field
Standard and extended data frames
Two receive buffers
Three Transmit Buffers
SPI Interface

Supported Baud rates
10 kbps; 20 kbps; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

Intended to be used with ATMEL ATMega328P with Arduino bootloader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers.
*/



#ifndef _CAN_H_
#define _CAN_H_

#include "Arduino.h"

// From the spec
#define DOMINANT    0
#define RECESSIVE   1

// Standard and Extended ID defines
#define extID 		1
#define stdID 		0

// Define the typical bitrate for CAN communication in kbps.
typedef enum
{
	CAN_BITRATE_5K                      = (0),
	CAN_BITRATE_10K                     = (1),
	CAN_BITRATE_20K                     = (2),
	CAN_BITRATE_25K                     = (3),
	CAN_BITRATE_50K                     = (4),
	CAN_BITRATE_100K                    = (5),
	CAN_BITRATE_125K                    = (6),
	CAN_BITRATE_250K                    = (7),
	CAN_BITRATE_500K                    = (8),
	CAN_BITRATE_800K                    = (9),
	CAN_BITRATE_1000K                   = (10),
	CAN_BITRATE_1M                      = (10)
} can_bitrate_t;

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

typedef struct
{
	uint32_t ID : 29; 			// Identifier
	uint8_t rtr : 1; 			// Remote Transmission Request
	uint8_t length : 4; 		// Data Length
	uint8_t data[8]; 			// Message data
} CAN_DATA_FRAME;

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

class CanClass
{
//    private:

    public:
// Constructor(s)
        void constructor();

// These must be defined by the subclass
        // virtual void enableAutoRange(bool enabled) {};
        // virtual void getEvent(sensors_event_t*);
        // virtual void getSensor(sensor_t*);

        //virtual bool begin();
        //virtual void baudConfig(int bitRate);      //sets up baud

};

//extern CanClass CAN;
#endif // _CAN_H_
