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


/* Reference for how this struct is defined:
This is from can.h from the libsam files
typedef struct {
	uint32_t ul_mb_idx;
	uint8_t uc_obj_type;  //! Mailbox object type, one of the six different objects.
	uint8_t uc_id_ver;    //! 0 stands for standard frame, 1 stands for extended frame.
	uint8_t uc_length;    //! Received data length or transmitted data length.
	uint8_t uc_tx_prio;   //! Mailbox priority, no effect in receive mode.
	uint32_t ul_status;   //! Mailbox status register value.
	uint32_t ul_id_msk;   //! No effect in transmit mode.
	uint32_t ul_id;       //! Received frame ID or the frame ID to be transmitted.
	uint32_t ul_fid;      //! Family ID.
	uint32_t ul_datal;
	uint32_t ul_datah;
} can_mb_conf_t;
These are from component_can.h in the CMSIS files
typedef struct {
  RwReg  CAN_MMR;       // (CanMb Offset: 0x0) Mailbox Mode Register
  RwReg  CAN_MAM;       // (CanMb Offset: 0x4) Mailbox Acceptance Mask Register
  RwReg  CAN_MID;       /**< \brief (CanMb Offset: 0x8) Mailbox ID Register
  RwReg  CAN_MFID;      /**< \brief (CanMb Offset: 0xC) Mailbox Family ID Register
  RwReg  CAN_MSR;       /**< \brief (CanMb Offset: 0x10) Mailbox Status Register
  RwReg  CAN_MDL;       /**< \brief (CanMb Offset: 0x14) Mailbox Data Low Register
  RwReg  CAN_MDH;       /**< \brief (CanMb Offset: 0x18) Mailbox Data High Register
  RwReg  CAN_MCR;       /**< \brief (CanMb Offset: 0x1C) Mailbox Control Register
} CanMb;
/** \brief Can hardware registers
#define CANMB_NUMBER 8
typedef struct {
  RwReg  CAN_MR;        /**< \brief (Can Offset: 0x0000) Mode Register
  WoReg  CAN_IER;       /**< \brief (Can Offset: 0x0004) Interrupt Enable Register
  WoReg  CAN_IDR;       /**< \brief (Can Offset: 0x0008) Interrupt Disable Register
  RoReg  CAN_IMR;       /**< \brief (Can Offset: 0x000C) Interrupt Mask Register
  RoReg  CAN_SR;        /**< \brief (Can Offset: 0x0010) Status Register
  RwReg  CAN_BR;        /**< \brief (Can Offset: 0x0014) Bitrate Register
  RoReg  CAN_TIM;       /**< \brief (Can Offset: 0x0018) Timer Register
  RoReg  CAN_TIMESTP;   /**< \brief (Can Offset: 0x001C) Timestamp Register
  RoReg  CAN_ECR;       /**< \brief (Can Offset: 0x0020) Error Counter Register
  WoReg  CAN_TCR;       /**< \brief (Can Offset: 0x0024) Transfer Command Register
  WoReg  CAN_ACR;       /**< \brief (Can Offset: 0x0028) Abort Command Register
  RoReg  Reserved1[46];
  RwReg  CAN_WPMR;      /**< \brief (Can Offset: 0x00E4) Write Protect Mode Register
  RoReg  CAN_WPSR;      /**< \brief (Can Offset: 0x00E8) Write Protect Status Register
  RoReg  Reserved2[69];
  CanMb  CAN_MB[CANMB_NUMBER]; /**< \brief (Can Offset: 0x200) MB = 0 .. 7
} Can;
*/

#if defined(__SAM3X8E__) // Arduino Due

#ifndef _CAN_SAM3X_H_
#define _CAN_SAM3X_H_

#include <Arduino.h>
#include <variant.h>

#include "CAN.h"
#include "sn65hvd234.h"

#define SAM3X_CAN0_RS  61
#define SAM3X_CAN0_EN  62
#define SAM3X_CAN1_RS  63
#define SAM3X_CAN1_EN  65

#define SAM3X_MODE_NORMAL    0
#define SAM3X_MODE_SLEEP     1 // Low-power mode
//#define SAM3X_MODE_LOOPBACK  2
#define SAM3X_MODE_LISTEN    3 // Auto-bitrate mode
#define SAM3X_MODE_CONFIG    4 // disabled?

/** Define the Mailbox mask for eight mailboxes. */
#define SAM3X_GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define SAM3X_DISABLE_ALL_INTERRUPT_MASK 0xffffffff

/** Define the mailbox mode. */
#define SAM3X_MB_DISABLE_MODE           0
#define SAM3X_MB_RX_MODE                1
#define SAM3X_MB_RX_OVER_WR_MODE        2
#define SAM3X_MB_TX_MODE                3
#define SAM3X_MB_CONSUMER_MODE          4
#define SAM3X_MB_PRODUCER_MODE          5

/** Define CAN mailbox transfer status code. */
#define SAM3X_MAILBOX_TRANSFER_OK       0     //! Read from or write into mailbox successfully.
#define SAM3X_MAILBOX_NOT_READY         0x01  //! Receiver is empty or transmitter is busy.
#define SAM3X_MAILBOX_RX_OVER           0x02  //! Message overwriting happens or there're messages lost in different receive modes.
#define SAM3X_MAILBOX_RX_NEED_RD_AGAIN  0x04  //! Application needs to re-read the data register in Receive with Overwrite mode.

#define SAM3X_SIZE_RX_BUFFER	32 //RX incoming ring buffer is this big
#define SAM3X_SIZE_TX_BUFFER	16 //TX ring buffer is this big

class CAN_SAM3X : public CANClass
{
  public:
    // Constructor
    CAN_SAM3X();
    CAN_SAM3X(uint8_t bus);

    // Initializes CAN communications into Normal mode.
    inline void begin (uint32_t bitrate)
    {
      begin(bitrate, SAM3X_MODE_NORMAL);
    };
    // Initializes CAN communications
    void begin (uint32_t bitrate, uint8_t mode);
    // Finishes CAN communications
    void end();
    // Check if message has been received on any of the buffers
    uint8_t available();
    // Receive CAN message and allows use of the message structure for easier message handling
    CAN_Frame read();

    void flush();
    // Load and send CAN message.
    uint8_t write(const CAN_Frame&);

    void interruptHandler();

  protected:
    /* CAN peripheral, set by constructor */
    Can* m_pCan ;

    /* CAN Transceiver */
    SSN65HVD234* m_Transceiver;

    int numTXBoxes; //There are 8 mailboxes, anything not TX will be set RX

    volatile CAN_Frame rx_frame_buff[SAM3X_SIZE_RX_BUFFER];
    volatile CAN_Frame tx_frame_buff[SAM3X_SIZE_TX_BUFFER];

    volatile uint8_t rx_buffer_head, rx_buffer_tail;
    volatile uint8_t tx_buffer_head, tx_buffer_tail;
    void mailbox_int_handler(uint8_t mb, uint32_t ul_status);

  private:
    bool _init( uint8_t Rs, uint8_t En);

    /**
    * \defgroup sam_driver_can_group Controller Area Network (CAN) Driver
    *
    * See \ref sam_can_quickstart.
    *
    * \par Purpose
    *
    * The CAN controller provides all the features required to implement
    * the serial communication protocol CAN defined by Robert Bosch GmbH,
    * the CAN specification. This is a driver for configuration, enabling,
    * disabling and use of the CAN peripheral.
    *
    * @{
    */

    void reset(); //CAN software reset.

    int setRXFilter(uint32_t id, uint32_t mask, bool extended);
    int setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    void setNumTXBoxes(int txboxes);
    int findFreeRXMailbox();
    uint8_t mailbox_get_mode(uint8_t uc_index);
    uint32_t mailbox_get_id(uint8_t uc_index);
    uint32_t getMailboxIer(int8_t mailbox);
    uint32_t set_bitrate(uint32_t ul_bitrate);
    uint32_t init(uint32_t ul_bitrate);
    void enable();
    void disable();
    void disable_low_power_mode();
    void enable_low_power_mode();
    void disable_autobitrate_listen_mode();
    void enable_autobitrate_listen_mode();
    void disable_overload_frame();
    void enable_overload_frame();
    void set_timestamp_capture_point(uint32_t ul_flag);
    void disable_time_triggered_mode();
    void enable_time_triggered_mode();
    void disable_timer_freeze();
    void enable_timer_freeze();
    void disable_tx_repeat();
    void enable_tx_repeat();
    void set_rx_sync_stage(uint32_t ul_stage);
    void enable_interrupt(uint32_t dw_mask);
    void disable_interrupt(uint32_t dw_mask);
    uint32_t get_interrupt_mask();
    uint32_t get_status();
    uint32_t get_internal_timer_value();
    uint32_t get_timestamp_value();
    uint8_t get_tx_error_cnt();
    uint8_t get_rx_error_cnt();
    void reset_internal_timer();
    void global_send_transfer_cmd(uint8_t uc_mask);
    void global_send_abort_cmd(uint8_t uc_mask);
    void mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt);
    uint32_t mailbox_get_status(uint8_t uc_index);
    void mailbox_send_transfer_cmd(uint8_t uc_index);
    void mailbox_send_abort_cmd(uint8_t uc_index);
    void mailbox_init(uint8_t uc_index);
    uint32_t mailbox_read(uint8_t uc_index, volatile CAN_Frame *rxframe);
    uint32_t mailbox_tx_frame(uint8_t uc_index);
    void mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended);
    void mailbox_set_priority(uint8_t uc_index, uint8_t pri);
    void mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext);
    void mailbox_set_mode(uint8_t uc_index, uint8_t mode);
    void mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val);
    void mailbox_set_datalen(uint8_t uc_index, uint8_t dlen);
    void mailbox_set_datal(uint8_t uc_index, uint32_t val);
    void mailbox_set_datah(uint8_t uc_index, uint32_t val);
    void reset_all_mailbox();
};

extern CAN_SAM3X CAN;
// Worry about multiple buses later
//extern CAN_SAM3X CANbus1;

#endif // _CAN_SAM3X_H_

#endif // defined(__SAM3X8E__) // Arduino Due
