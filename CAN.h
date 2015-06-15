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


#ifndef _CAN_H_
#define _CAN_H_

#include <inttypes.h>
#include <Arduino.h>

// From the spec
#define CAN_DOMINANT                   0
#define CAN_RECESSIVE                  1

// Standard and Extended ID defines
#define CAN_STANDARD_FRAME             0
#define CAN_EXTENDED_FRAME             1

// For an 11 bit standard frame ID within a 16 bit variable, the first 5
// bits [15:11] are ignored
#define CAN_STANDARD_ID_MASK           0x07FF

// For a 29 bit extended frame ID within a 32 bit variable, the first 3
// bits [31:29] are ignored
#define CAN_EXTENDED_ID_MASK           0x1FFFFFFF

//

// Define the typical bitrate for CAN communication in kbps.
#define CAN_BPS_1M                     1000000
#define CAN_BPS_1000K                  1000000
#define CAN_BPS_800K                   800000
#define CAN_BPS_500K                   500000
#define CAN_BPS_250K                   250000
#define CAN_BPS_200K                   200000
#define CAN_BPS_125K                   125000
#define CAN_BPS_100K                   100000
#define CAN_BPS_83K                    83333  // According to ARINC 825, this is a thing
#define CAN_BPS_80K                    80000
#define CAN_BPS_50K                    50000
#define CAN_BPS_40K                    40000
#define CAN_BPS_33333                  33333
#define CAN_BPS_31K25                  31250
#define CAN_BPS_25K                    25000
#define CAN_BPS_20K                    20000
#define CAN_BPS_10K                    10000
#define CAN_BPS_5K                     5000

// ISO 11783-3:2014
// Section 5.13.3
// Controller response time and timeout defaults
#define CAN_TIMEOUT_TR 200
#define CAN_TIMEOUT_TH 500
#define CAN_TIMEOUT_T1 750
#define CAN_TIMEOUT_T2 1250
#define CAN_TIMEOUT_T3 1250
#define CAN_TIMEOUT_T4 1050

//
//
typedef struct __attribute__((__packed__))
{
  uint32_t id      : 29;  // if (extended == CAN_RECESSIVE) { extended ID } else { standard ID }
  uint8_t valid    : 1;   // To avoid passing garbage frames around
  uint8_t rtr      : 1;   // Remote Transmission Request Bit (RTR)
  uint8_t extended : 1;   // Identifier Extension Bit (IDE)
  uint32_t fid;           // family ID
  uint8_t priority : 4;   // Priority but only important for TX frames and then only for special uses.
  uint8_t length   : 4;   // Data Length
  uint16_t timeout;       // milliseconds, zero will disable waiting
  uint8_t data[8];                        // Message data
} CAN_Frame;              // suffix of '_t' is reserved by POSIX for future use


// From http://www.cse.dmu.ac.uk/~eg/tele/CanbusIDandMask.html
//
// CANBUS is a two-wire, half-duplex, bus based LAN system that is ‘collision
// free’. Data is BROADCAST onto the bus -THERE IS NO SUCH THNG AS A POINT TO
// POINT CONNECTION as with data LANs. All nodes receive all broadcast data
// and decide whether or not that data is relevant.
// A receiving node would examine the identifier to decide if it was relevant
// (e.g. waiting for a frame with ID 00001567 which contains data to switch on
// or off a motor). It could do this via software (using a C if or case
// statement); in practice the Canbus interface contains firmware to carry out
// this task using the acceptance filter and mask value to filter out unwanted
// messages.
//
// The filter mask is used to determine which bits in the identifier of the
// received frame are compared with the filter
//
//    If a mask bit is set to a zero, the corresponding ID bit will
//    automatically be accepted, regardless of the value of the filter bit.
//
//    If a mask bit is set to a one, the corresponding ID bit will be compared
//    with the value of the filter bit; if they match it is accepted otherwise
//    the frame is rejected.
//
// Example 1. we wish to accept only frames with ID of 00001567 (hexadecimal values)
//
//    set filter to 00001567
//
//    set mask to 1FFFFFFF
//
// when a frame arrives its ID is compared with the filter and all bits must
// match; any frame that does not match ID 00001567 is rejected
//
// Example 4. we wish to accept any frame
//
//    set filter to 0
//
//    set mask to 0
//
// all frames are accepted
//
//
// In practice Canbus interfaces tends to have a number of filters and masks
// so combinations of IDs can be accepted, e.g. a module that carries out a
// number of different tasks.
typedef struct __attribute__((__packed__))
{
  uint32_t id      : 29;  // if (extended == CAN_RECESSIVE) { extended ID } else { standard ID }
  uint8_t rtr      : 1;   // Remote Transmission Request Bit (RTR)
  uint8_t extended : 1;   // Identifier Extension Bit (IDE)
  uint8_t data[2];        // Filter / Mask for message data
} CAN_Filter;             // suffix of '_t' is reserved by POSIX for future use


class CANClass // Can't inherit from Stream
{
  public:
    // Initializes CAN communications.
    virtual void begin(uint32_t bitrate);
    // Finishes CAN communications
    virtual void end();
    // Check if message has been received on any of the buffers
    virtual uint8_t available();
    // Receive CAN message and allows use of the message structure for easier message handling
    virtual CAN_Frame read();

    virtual void flush();
    // Load and send CAN message.
    virtual uint8_t write(const CAN_Frame&);

    //CAN_Frame& operator=(const CAN_Frame&);
};

// Too many other libraries already define CAN.
//extern CANClass CAN;
//extern CANClass CANbus;
// Unable to use extern on a base class

// It's time for #ifdef bingo!
/**********************************************************/
/*     8 bit AVR-based boards                             */
/**********************************************************/
#if defined(__AVR__)

#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
#define CAN_CONTROLLER_AT90CAN
#include "CAN_AT90CAN.h"
// Not sure if code will be different for these CPUs
#elif defined(__AVR_ATmega32C1__) || \
     defined(__AVR_ATmega64C1__) || \
     defined(__AVR_ATmega16M1__) || \
     defined(__AVR_ATmega32M1__) || \
     defined(__AVR_ATmega64M1__)
#error Are you sure these are supported?
#else // Assume it's an AVR with SPI interface
//#if defined(MCP2515)
#define CAN_CONTROLLER_MCP2515 // SPI interface to MCP2515 chip
#include <SPI.h>
#include "CAN_MCP2515.h"
//#elif defined(SJA1000) // 8-bit parallel interface to SJA1000 chip
//#define CAN_CONTROLLER_SJA1000
//#include "CAN_SJA1000.h"
//#else
//#error "Your SPI CAN controller is currently unsupported."
//#endif // MCP2515 / SJA1000
#endif // AT90 / M1 / C1 / SPI

/**********************************************************/
/*     32 bit Arduino Due                                 */
/**********************************************************/
#elif defined(__arm__) && defined(__SAM3X8E__)
#define CAN_CONTROLLER_SAM3X
#include "CAN_SAM3X.h"
/**********************************************************/
/*     32 bit Teensy 3.0 and 3.1                          */
/**********************************************************/
#elif defined(__arm__) && defined(TEENSYDUINO) && defined(KINETISK)
#define CAN_CONTROLLER_K2X
#include "CAN_K2X.h"
/**********************************************************/
/*     32 bit Teensy-LC                                   */
/**********************************************************/
#elif defined(__arm__) && defined(TEENSYDUINO) && defined(KINETISL)
// Assume an SPI interface to an MCP2515
#define CAN_CONTROLLER_MCP2515 // SPI interface to MCP2515 chip
#include <SPI.h>
#include "CAN_MCP2515.h"
#else
#error "Your CPU & CAN controller are currently unsupported."
#endif // ARDUINO_ARCH_*

#endif // _CAN_H_
