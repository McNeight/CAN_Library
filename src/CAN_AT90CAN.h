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
Fabian Greif for the initial AT90CAN library from https://github.com/dergraaf/avr-can-lib

-------------------------------------------------------------------------------------------------------------
Change Log

DATE		VER		WHO			WHAT
-------------------------------------------------------------------------------------------------------------

Features:

CAN V2.0B
8 byte length in the data field
Standard and extended data frames
Two receive buffers
Three Transmit Buffers

Supported bitrates:
10 kbps; 20 kbps; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

*/

#if defined(ARDUINO_ARCH_AVR)

#ifndef _CAN_AT90CAN_H_
#define _CAN_AT90CAN_H_

#include <Arduino.h>
#include "CAN.h"

#if (defined (__AVR_AT90CAN32__) || \
	 defined (__AVR_AT90CAN64__) || \
	 defined (__AVR_AT90CAN128__)) && \
	 BUILD_FOR_AT90CAN == 1

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

#endif // defined(ARDUINO_ARCH_AVR)
