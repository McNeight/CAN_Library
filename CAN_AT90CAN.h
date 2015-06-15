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


#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)

#ifndef _CAN_AT90CAN_H_
#define _CAN_AT90CAN_H_

#include <Arduino.h>
#include "CAN.h"

#if F_CPU != 16000000UL
#error	only 16 MHz for F_CPU supported!
#endif

#define	SUPPORT_FOR_AT90CAN__		1

// ----------------------------------------------------------------------------

#if CAN_RX_BUFFER_SIZE > 0
extern can_buffer_t can_rx_buffer;
#else
extern volatile uint8_t _messages_waiting;
#endif

#if CAN_TX_BUFFER_SIZE > 0
extern can_buffer_t can_tx_buffer;
#else
extern volatile uint8_t _free_buffer;
#endif

#if CAN_FORCE_TX_ORDER
extern volatile uint8_t _transmission_in_progress ;
#endif

// ----------------------------------------------------------------------------
extern uint8_t _find_free_mob(void);

// ----------------------------------------------------------------------------
extern void _disable_mob_interrupt(uint8_t mob);

// ----------------------------------------------------------------------------
extern void _enable_mob_interrupt(uint8_t mob);


// ----------------------------------------------------------------------------
extern uint8_t at90can_send_message(const can_t *msg);

// ----------------------------------------------------------------------------
extern uint8_t at90can_get_message(can_t *msg);

// ----------------------------------------------------------------------------
/**
 * \brief	Copy data form a message in RAM to the actual registers
 * \warning this function assumes CANPAGE to be set properly
 */
extern void at90can_copy_message_to_mob(const can_t *msg);

// ----------------------------------------------------------------------------
/**
 * \brief	Copy data form a message the registers to RAM
 * \warning this function assumes CANPAGE to be set properly
 */
extern void at90can_copy_mob_to_message(can_t *msg);

// ----------------------------------------------------------------------------
// enter standby mode => messages are not transmitted nor received

extern __attribute__ ((gnu_inline)) inline void _enter_standby_mode(void)
{
  // request abort
  CANGCON = (1 << ABRQ);

  // wait until receiver is not busy
  while (CANGSTA & (1 << RXBSY))
    ;

  // request standby mode
  CANGCON = 0;

  // wait until the CAN Controller has entered standby mode
  while (CANGSTA & (1 << ENFG))
    ;
}

// ----------------------------------------------------------------------------
// leave standby mode => CAN Controller is connected to CAN Bus

extern __attribute__ ((gnu_inline)) inline void _leave_standby_mode(void)
{
  // save CANPAGE register
  uint8_t canpage = CANPAGE;

  // reenable all MObs
  for (uint8_t i = 0; i < 15; i++) {
    CANPAGE = i << 4;
    CANCDMOB = CANCDMOB;
  }

  // restore CANPAGE
  CANPAGE = canpage;

  // request normal mode
  CANGCON = (1 << ENASTB);

  // wait until the CAN Controller has left standby mode
  while ((CANGSTA & (1 << ENFG)) == 0)
    ;
}


// MCP class
class CAN_AT90CAN : public CANClass
{
  public:


    CAN_AT90CAN();
    void begin (uint32_t bitrate)
    {
      begin(bitrate, AT90CAN_MODE_NORMAL);
    };
    // Initializes CAN communications
    void begin (uint32_t bitrate, uint8_t mode);

    void end();
    // check if message has been received on any of the buffers
    uint8_t available();
    //Receive and display CAN message and allows use of the message structure for easier message handling
    CAN_Frame read();
    // Receive and display any message (J1939, CANopen, CAN)
    void read(uint32_t *ID, uint8_t *length_out, uint8_t *data_out);

    void flush();

    uint8_t write(CAN_Frame&);
    // Load and send message. No RTS needed.
    uint8_t write(uint32_t ID, uint8_t frameType, uint8_t length, uint8_t *data);

};

#endif // _CAN_AT90CAN_H_

#endif // defined(__AVR_AT90CAN32__) || \
//        defined(__AVR_AT90CAN64__) || \
//        defined(__AVR_AT90CAN128__)
