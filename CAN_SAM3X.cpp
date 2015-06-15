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


#if defined(__SAM3X8E__) // Arduino Due

#include <Arduino.h>
#include <variant.h>

#include "CAN.h"
#include "CAN_SAM3X.h"
#include "sn65hvd234.h"

// Taken from libsam/source/can.c
/** Define the timemark mask. */
#define TIMEMARK_MASK              0x0000ffff

/* CAN timeout for synchronization. */
#define CAN_TIMEOUT                100000

/** The max value for CAN bitrate prescale. */
#define CAN_BITRATE_MAX_DIV       128

/** Define the scope for TQ. */
#define CAN_MIN_TQ_NUM             8
#define CAN_MAX_TQ_NUM             25

/** Define the fixed bit time value. */
#define CAN_BIT_SYNC               1
#define CAN_BIT_IPT                2

typedef struct
{
  uint8_t uc_tq;      //! CAN_BIT_SYNC + uc_prog + uc_phase1 + uc_phase2 = uc_tq, 8 <= uc_tq <= 25.
  uint8_t uc_prog;    //! Propagation segment, (3-bits + 1), 1~8;
  uint8_t uc_phase1;  //! Phase segment 1, (3-bits + 1), 1~8;
  uint8_t uc_phase2;  //! Phase segment 2, (3-bits + 1), 1~8, CAN_BIT_IPT <= uc_phase2;
  uint8_t uc_sjw;     //! Resynchronization jump width, (2-bits + 1), min(uc_phase1, 4);
  uint8_t uc_sp;      //! Sample point value, 0~100 in percent.
} can_bit_timing_t;


/** Values of bit time register for different bitrates, Sample point = ((1 + uc_prog + uc_phase1) / uc_tq) * 100%. */
const can_bit_timing_t can_bit_time[] =
{
  {8,   (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
  {9,   (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
  {10,  (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
  {11,  (3 + 1), (2 + 1), (2 + 1), (3 + 1), 72},
  {12,  (2 + 1), (3 + 1), (3 + 1), (3 + 1), 67},
  {13,  (3 + 1), (3 + 1), (3 + 1), (3 + 1), 77},
  {14,  (3 + 1), (3 + 1), (4 + 1), (3 + 1), 64},
  {15,  (3 + 1), (4 + 1), (4 + 1), (3 + 1), 67},
  {16,  (4 + 1), (4 + 1), (4 + 1), (3 + 1), 69},
  {17,  (5 + 1), (4 + 1), (4 + 1), (3 + 1), 71},
  {18,  (4 + 1), (5 + 1), (5 + 1), (3 + 1), 67},
  {19,  (5 + 1), (5 + 1), (5 + 1), (3 + 1), 68},
  {20,  (6 + 1), (5 + 1), (5 + 1), (3 + 1), 70},
  {21,  (7 + 1), (5 + 1), (5 + 1), (3 + 1), 71},
  {22,  (6 + 1), (6 + 1), (6 + 1), (3 + 1), 68},
  {23,  (7 + 1), (7 + 1), (6 + 1), (3 + 1), 70},
  {24,  (6 + 1), (7 + 1), (7 + 1), (3 + 1), 67},
  {25,  (7 + 1), (7 + 1), (7 + 1), (3 + 1), 68}
};

/**
* \brief constructor for the class
*
* \param Rs pin to use for transceiver Rs control
* \param En pin to use for transceiver enable
*/
CAN_SAM3X::CAN_SAM3X()
{
  // Default to CAN bus 0
  m_pCan = CAN0;
  _init(SAM3X_CAN0_RS, SAM3X_CAN0_EN);
}

CAN_SAM3X::CAN_SAM3X(uint8_t bus)
{
  if (bus == 0)
  {
    m_pCan = CAN0;
    _init(SAM3X_CAN0_RS, SAM3X_CAN0_EN);
  }
  else if (bus == 1)
  {
    m_pCan = CAN1;
    _init(SAM3X_CAN1_RS, SAM3X_CAN1_EN);
  }
  else
  {
    // Don't know what to tell you here...
  }
}

/**
 * \brief Initialize CAN controller.
 *
 * \param ul_mck CAN module input clock.
 * \param ul_bitrate CAN communication bitrates in kbps.
 *
 * \retval 0 If failed to initialize the CAN module; otherwise successful.
 *
 * \note PMC clock for CAN peripheral should be enabled before calling this function.
 */
bool CAN_SAM3X::_init(uint8_t Rs, uint8_t En)
{

  m_Transceiver = new SSN65HVD234(Rs, En);

  //arduino 1.5.2 doesn't init canbus so make sure to do it here.
#ifdef ARDUINO156
  PIO_Configure(PIOA, PIO_PERIPH_A, PIO_PA1A_CANRX0 | PIO_PA0A_CANTX0, PIO_DEFAULT);
  PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB15A_CANRX1 | PIO_PB14A_CANTX1, PIO_DEFAULT);
#endif

  if (m_pCan == CAN0) pmc_enable_periph_clk(ID_CAN0);
  if (m_pCan == CAN1) pmc_enable_periph_clk(ID_CAN1);

  m_Transceiver->DisableLowPower();
  m_Transceiver->Enable();
}

void CAN_SAM3X::begin(uint32_t bitrate, uint8_t mode)
{
  uint32_t ul_flag;
  uint32_t ul_tick;

  /* Initialize the bitrate for CAN module. */
  ul_flag = set_bitrate(bitrate);
  if (ul_flag == 0)
  {
    //return 0;
  }

  /* Reset the CAN eight message mailbox. */
  reset_all_mailbox();

  //Also disable all interrupts by default
  disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);

  //By default use one mailbox for TX
  setNumTXBoxes(1);

  /* Enable the CAN controller. */
  enable();

  /* Wait until the CAN is synchronized with the bus activity. */
  ul_flag = 0;
  ul_tick = 0;
  while (!(ul_flag & CAN_SR_WAKEUP) && (ul_tick < CAN_TIMEOUT))
  {
    ul_flag = m_pCan->CAN_SR;
    ul_tick++;
  }

  NVIC_EnableIRQ(m_pCan == CAN0 ? CAN0_IRQn : CAN1_IRQn); //tell the nested interrupt controller to turn on our interrupt

  /* Timeout or the CAN module has been synchronized with the bus. */
  // if (CAN_TIMEOUT == ul_tick) {
  // return 0;
  // } else {
  // return 1;
  // }

  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++)
  {
    CAN.setRXFilter(filter, 0, 0, true);
    // Worry about multiple buses later
    //CANbus1.setRXFilter(filter, 0, 0, true);
  }
  //standard
  for (int filter = 3; filter < 7; filter++)
  {
    CAN.setRXFilter(filter, 0, 0, false);
    // Worry about multiple buses later
    //CANbus1.setRXFilter(filter, 0, 0, false);
  }

}


void CAN_SAM3X::end()
{
  /* Disable the CAN controller. */
  disable();
}


/**
* \brief Check whether there are received canbus frames in the buffer
*/
uint8_t CAN_SAM3X::available()
{
  return (rx_buffer_head != rx_buffer_tail) ? true : false;
}


CAN_Frame CAN_SAM3X::read()
{
  CAN_Frame buffer;
  if (rx_buffer_head == rx_buffer_tail)
  {
    buffer.valid = 0;
  }
  else
  {
    buffer.valid = 1;
    buffer.id = rx_frame_buff[rx_buffer_tail].id;
    buffer.extended = rx_frame_buff[rx_buffer_tail].extended;
    buffer.length = rx_frame_buff[rx_buffer_tail].length;
    for (int c = 0; c < 8; c++)
    {
      buffer.data[c] = rx_frame_buff[rx_buffer_tail].data[c];
    }
    rx_buffer_tail = (rx_buffer_tail + 1) % SAM3X_SIZE_RX_BUFFER;
  }
  return buffer;
}


void CAN_SAM3X::flush()
{
  reset_all_mailbox();
}


/*
Does one of two things, either sends the given frame out on the first
TX mailbox that's open or queues the frame for sending later via interrupts.
This vastly simplifies the sending of frames - It does, however, assume
that you're going to use interrupt driven transmission - It forces it really.
*/
uint8_t CAN_SAM3X::write(const CAN_Frame& txFrame)
{
  for (int i = 0; i < 8; i++)
  {
    if (((m_pCan->CAN_MB[i].CAN_MMR >> 24) & 7) == CAN_MB_TX_MODE)
    {
      //is this mailbox set up as a TX box?
      if (m_pCan->CAN_MB[i].CAN_MSR & CAN_MSR_MRDY)
      {
        //is it also available (not sending anything?)
        mailbox_set_id(i, txFrame.id, txFrame.extended);
        mailbox_set_datalen(i, txFrame.length);
        mailbox_set_priority(i, txFrame.priority);
        for (uint8_t cnt = 0; cnt < 8; cnt++)
        {
          mailbox_set_databyte(i, cnt, txFrame.data[cnt]);
        }
        enable_interrupt(0x01u << i); //enable the TX interrupt for this box
        global_send_transfer_cmd((0x1u << i));
        return 1; //we've sent it. mission accomplished.
      }
    }
  }

  //if execution got to this point then no free mailbox was found above
  //so, queue the frame.
  tx_frame_buff[tx_buffer_tail].id = txFrame.id;
  tx_frame_buff[tx_buffer_tail].extended = txFrame.extended;
  tx_frame_buff[tx_buffer_tail].length = txFrame.length;
  tx_frame_buff[tx_buffer_tail].data[0] = txFrame.data[0];
  tx_frame_buff[tx_buffer_tail].data[1] = txFrame.data[1];
  tx_frame_buff[tx_buffer_tail].data[2] = txFrame.data[2];
  tx_frame_buff[tx_buffer_tail].data[3] = txFrame.data[3];
  tx_frame_buff[tx_buffer_tail].data[4] = txFrame.data[4];
  tx_frame_buff[tx_buffer_tail].data[5] = txFrame.data[5];
  tx_frame_buff[tx_buffer_tail].data[6] = txFrame.data[6];
  tx_frame_buff[tx_buffer_tail].data[7] = txFrame.data[7];
  tx_buffer_tail = (tx_buffer_tail + 1) % SAM3X_SIZE_TX_BUFFER;
}


/**
* \brief Handle all interrupt reasons
*/
void CAN_SAM3X::interruptHandler()
{

  uint32_t ul_status = m_pCan->CAN_SR; //get status of interrupts

  if (ul_status & CAN_SR_MB0)   //mailbox 0 event
  {
    mailbox_int_handler(0, ul_status);
  }
  if (ul_status & CAN_SR_MB1)   //mailbox 1 event
  {
    mailbox_int_handler(1, ul_status);
  }
  if (ul_status & CAN_SR_MB2)   //mailbox 2 event
  {
    mailbox_int_handler(2, ul_status);
  }
  if (ul_status & CAN_SR_MB3)   //mailbox 3 event
  {
    mailbox_int_handler(3, ul_status);
  }
  if (ul_status & CAN_SR_MB4)   //mailbox 4 event
  {
    mailbox_int_handler(4, ul_status);
  }
  if (ul_status & CAN_SR_MB5)   //mailbox 5 event
  {
    mailbox_int_handler(5, ul_status);
  }
  if (ul_status & CAN_SR_MB6)   //mailbox 6 event
  {
    mailbox_int_handler(6, ul_status);
  }
  if (ul_status & CAN_SR_MB7)   //mailbox 7 event
  {
    mailbox_int_handler(7, ul_status);
  }
  if (ul_status & CAN_SR_ERRA)   //error active
  {
  }
  if (ul_status & CAN_SR_WARN)   //warning limit
  {
  }
  if (ul_status & CAN_SR_ERRP)   //error passive
  {
  }
  if (ul_status & CAN_SR_BOFF)   //bus off
  {
  }
  if (ul_status & CAN_SR_SLEEP)   //controller in sleep mode
  {
  }
  if (ul_status & CAN_SR_WAKEUP)   //controller woke up
  {
  }
  if (ul_status & CAN_SR_TOVF)   //timer overflow
  {
  }
  if (ul_status & CAN_SR_TSTP)   //timestamp - start or end of frame
  {
  }
  if (ul_status & CAN_SR_CERR)   //CRC error in mailbox
  {
  }
  if (ul_status & CAN_SR_SERR)   //stuffing error in mailbox
  {
  }
  if (ul_status & CAN_SR_AERR)   //ack error
  {
  }
  if (ul_status & CAN_SR_FERR)   //form error
  {
  }
  if (ul_status & CAN_SR_BERR)   //bit error
  {
  }
}


/**
 * \brief Configure CAN bitrate.
 *
 * \param ul_bitrate bitrate value (kB/s), allowed values:
 *                    1000, 800, 500, 250, 125, 50, 25, 10, 5.
 *
 * \retval Set the bitrate successfully or not.
 */
uint32_t CAN_SAM3X::set_bitrate(uint32_t ul_bitrate)
{
  uint8_t uc_tq;
  uint8_t uc_prescale;
  uint32_t ul_mod;
  uint32_t ul_cur_mod;
  can_bit_timing_t *p_bit_time;

  static uint32_t ul_mck = SystemCoreClock;

  /* Check whether the bitrate prescale will be greater than the max divide value. */
  if (((ul_mck + (ul_bitrate * CAN_MAX_TQ_NUM - 1)) /
       (ul_bitrate * CAN_MAX_TQ_NUM)) > CAN_BITRATE_MAX_DIV)
  {
    return 0;
  }

  /* Check whether the input MCK is too small. */
  if (ul_mck  < ul_bitrate * CAN_MIN_TQ_NUM)
  {
    return 0;
  }

  /* Initialize it as the minimum Time Quantum. */
  uc_tq = CAN_MIN_TQ_NUM;

  /* Initialize the remainder as the max value. When the remainder is 0, get the right TQ number. */
  ul_mod = 0xffffffff;
  /* Find out the approximate Time Quantum according to the bitrate. */
  for (uint8_t i = CAN_MIN_TQ_NUM; i <= CAN_MAX_TQ_NUM; i++)
  {
    if ((ul_mck / (ul_bitrate * i)) <= CAN_BITRATE_MAX_DIV)
    {
      ul_cur_mod = ul_mck % (ul_bitrate * i);
      if (ul_cur_mod < ul_mod)
      {
        ul_mod = ul_cur_mod;
        uc_tq = i;
        if (!ul_mod)
        {
          break;
        }
      }
    }
  }

  /* Calculate the bitrate prescale value. */
  uc_prescale = ul_mck / (ul_bitrate * uc_tq);

  /* Get the right CAN BIT Timing group. */
  p_bit_time = (can_bit_timing_t *)&can_bit_time[uc_tq - CAN_MIN_TQ_NUM];

  /* Before modifying the CANBR register, disable the CAN controller. */
  //can_disable(m_pCan);
  m_pCan->CAN_MR &= ~CAN_MR_CANEN;

  /* Write into the CAN bitrate register. */
  m_pCan->CAN_BR = CAN_BR_PHASE2(p_bit_time->uc_phase2 - 1) |
                   CAN_BR_PHASE1(p_bit_time->uc_phase1 - 1) |
                   CAN_BR_PROPAG(p_bit_time->uc_prog - 1) |
                   CAN_BR_SJW(p_bit_time->uc_sjw - 1) |
                   CAN_BR_BRP(uc_prescale - 1);
  return 1;
}

void CAN_SAM3X::setNumTXBoxes(int txboxes)
{
  int c;

  if (txboxes > 8) txboxes = 8;
  if (txboxes < 0) txboxes = 0;
  numTXBoxes = txboxes;

  //Initialize RX boxen
  for (c = 0; c < 8 - numTXBoxes; c++)
  {
    mailbox_set_mode(c, CAN_MB_RX_MODE);
    mailbox_set_id(c, 0x0, false);
    mailbox_set_accept_mask(c, 0x7FF, false);
  }

  //Initialize TX boxen
  for (c = 8 - numTXBoxes; c < 8; c++)
  {
    mailbox_set_mode(c, CAN_MB_TX_MODE);
    mailbox_set_priority(c, 10);
    mailbox_set_accept_mask(c, 0x7FF, false);
  }
}

/**
 * \brief Enable CAN Controller.
 *
 */
void CAN_SAM3X::enable()
{
  m_pCan->CAN_MR |= CAN_MR_CANEN;
  m_Transceiver->Enable();
}

/**
 * \brief Disable CAN Controller.
 *
 */
void CAN_SAM3X::disable()
{
  m_pCan->CAN_MR &= ~CAN_MR_CANEN;

  m_Transceiver->EnableLowPower();
  m_Transceiver->Disable();

}

/**
 * \brief Disable CAN Controller low power mode.
 *
 */
void CAN_SAM3X::disable_low_power_mode()
{
  m_pCan->CAN_MR &= ~CAN_MR_LPM;
}

/**
 * \brief Enable CAN Controller low power mode.
 *
 */
void CAN_SAM3X::enable_low_power_mode()
{
  m_pCan->CAN_MR |= CAN_MR_LPM;
}

/**
 * \brief Disable CAN Controller autobitrate/listen mode.
 *
 */
void CAN_SAM3X::disable_autobitrate_listen_mode()
{
  m_pCan->CAN_MR &= ~CAN_MR_ABM;
}

/**
 * \brief Enable CAN Controller autobitrate/listen mode.
 *
 */
void CAN_SAM3X::enable_autobitrate_listen_mode()
{
  m_pCan->CAN_MR |= CAN_MR_ABM;
}

/**
 * \brief CAN Controller won't generate overload frame.
 *
 */
void CAN_SAM3X::disable_overload_frame()
{
  m_pCan->CAN_MR &= ~CAN_MR_OVL;
}

/**
 * \brief CAN Controller will generate an overload frame after each successful
 * reception for mailboxes configured in Receive mode, Producer and Consumer.
 *
 */
void CAN_SAM3X::enable_overload_frame()
{
  m_pCan->CAN_MR |= CAN_MR_OVL;
}

/**
 * \brief Configure the timestamp capture point, at the start or the end of frame.
 *
 * \param m_pCan   Pointer to a CAN peripheral instance.
 * \param ul_flag 0: Timestamp is captured at each start of frame;
 *                1: Timestamp is captured at each end of frame.
 */
void CAN_SAM3X::set_timestamp_capture_point(uint32_t ul_flag)
{
  if (ul_flag)
  {
    m_pCan->CAN_MR |= CAN_MR_TEOF;
  }
  else
  {
    m_pCan->CAN_MR &= ~CAN_MR_TEOF;
  }
}

/**
 * \brief Disable CAN Controller time triggered mode.
 *
 */
void CAN_SAM3X::disable_time_triggered_mode()
{
  m_pCan->CAN_MR &= ~CAN_MR_TTM;
}

/**
 * \brief Enable CAN Controller time triggered mode.
 *
 */
void CAN_SAM3X::enable_time_triggered_mode()
{
  m_pCan->CAN_MR |= CAN_MR_TTM;
}

/**
 * \brief Disable CAN Controller timer freeze.
 *
 */
void CAN_SAM3X::disable_timer_freeze()
{
  m_pCan->CAN_MR &= ~CAN_MR_TIMFRZ;
}

/**
 * \brief Enable CAN Controller timer freeze.
 *
 */
void CAN_SAM3X::enable_timer_freeze()
{
  m_pCan->CAN_MR |= CAN_MR_TIMFRZ;
}

/**
 * \brief Disable CAN Controller transmit repeat function.
 *
 */
void CAN_SAM3X::disable_tx_repeat()
{
  m_pCan->CAN_MR |= CAN_MR_DRPT;
}

/**
 * \brief Enable CAN Controller transmit repeat function.
 *
 */
void CAN_SAM3X::enable_tx_repeat()
{
  m_pCan->CAN_MR &= ~CAN_MR_DRPT;
}

/**
 * \brief Configure CAN Controller reception synchronization stage.
 *
 * \param ul_stage The reception stage to be configured.
 *
 * \note This is just for debug purpose only.
 */
void CAN_SAM3X::set_rx_sync_stage(uint32_t ul_stage)
{
  m_pCan->CAN_MR = (m_pCan->CAN_MR & ~CAN_MR_RXSYNC_Msk) | ul_stage;
}

/**
 * \brief Enable CAN interrupt.
 *
 * \param dw_mask Interrupt to be enabled.
 */
void CAN_SAM3X::enable_interrupt(uint32_t dw_mask)
{
  m_pCan->CAN_IER = dw_mask;
}

/**
 * \brief Disable CAN interrupt.
 *
 * \param dw_mask Interrupt to be disabled.
 */
void CAN_SAM3X::disable_interrupt(uint32_t dw_mask)
{
  m_pCan->CAN_IDR = dw_mask;
}

/**
 * \brief Get CAN Interrupt Mask.
 *
 *
 * \retval CAN interrupt mask.
 */
uint32_t CAN_SAM3X::get_interrupt_mask()
{
  return (m_pCan->CAN_IMR);
}

/**
 * \brief Get CAN status.
 *
 *
 * \retval CAN status.
 */
uint32_t CAN_SAM3X::get_status()
{
  return (m_pCan->CAN_SR);
}

/**
 * \brief Get the 16-bit free-running internal timer count.
 *
 *
 * \retval The internal CAN free-running timer counter.
 */
uint32_t CAN_SAM3X::get_internal_timer_value()
{
  return (m_pCan->CAN_TIM);
}

/**
 * \brief Get CAN timestamp register value.
 *
 *
 * \retval The timestamp value.
 */
uint32_t CAN_SAM3X::get_timestamp_value()
{
  return (m_pCan->CAN_TIMESTP);
}

/**
 * \brief Get CAN transmit error counter.
 *
 *
 * \retval Transmit error counter.
 */
uint8_t CAN_SAM3X::get_tx_error_cnt()
{
  return (uint8_t) (m_pCan->CAN_ECR >> CAN_ECR_TEC_Pos);
}

/**
 * \brief Get CAN receive error counter.
 *
 *
 * \retval Receive error counter.
 */
uint8_t CAN_SAM3X::get_rx_error_cnt()
{
  return (uint8_t) (m_pCan->CAN_ECR >> CAN_ECR_REC_Pos);
}

/**
 * \brief Reset the internal free-running 16-bit timer.
 *
 *
 * \note If the internal timer counter is frozen, this function automatically
 * re-enables it.
 */
void CAN_SAM3X::reset_internal_timer()
{
  m_pCan->CAN_TCR |= CAN_TCR_TIMRST;
}

/**
 * \brief Send global transfer request.
 *
 * \param uc_mask Mask for mailboxes that are requested to transfer.
 */
void CAN_SAM3X::global_send_transfer_cmd(uint8_t uc_mask)
{
  uint32_t ul_reg;

  ul_reg = m_pCan->CAN_TCR & ((uint32_t)~GLOBAL_MAILBOX_MASK);
  m_pCan->CAN_TCR = ul_reg | uc_mask;
}

/**
 * \brief Send global abort request.
 *
 * \param uc_mask Mask for mailboxes that are requested to abort.
 */
void CAN_SAM3X::global_send_abort_cmd(uint8_t uc_mask)
{
  uint32_t ul_reg;

  ul_reg = m_pCan->CAN_ACR & ((uint32_t)~GLOBAL_MAILBOX_MASK);
  m_pCan->CAN_ACR = ul_reg | uc_mask;
}

/**
 * \brief Configure the timemark for the mailbox.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 * \param us_cnt   The timemark to be set.
 *
 * \note The timemark is active in Time Triggered mode only.
 */
void CAN_SAM3X::mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt)
{
  uint32_t ul_reg;
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  ul_reg = m_pCan->CAN_MB[uc_index].CAN_MMR & ((uint32_t)~TIMEMARK_MASK);
  m_pCan->CAN_MB[uc_index].CAN_MMR = ul_reg | us_cnt;
}

/**
 * \brief Get status of the mailbox.
 *
 * \param uc_index Indicate which mailbox is to be read.
 *
 * \retval The mailbox status.
 */
uint32_t CAN_SAM3X::mailbox_get_status(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  return (m_pCan->CAN_MB[uc_index].CAN_MSR);
}

/**
 * \brief Send single mailbox transfer request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CAN_SAM3X::mailbox_send_transfer_cmd(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MTCR;
}

/**
 * \brief Send single mailbox abort request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CAN_SAM3X::mailbox_send_abort_cmd(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MACR;
}

/**
 * \brief Initialize the mailbox to a default, known state.
 *
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void CAN_SAM3X::mailbox_init(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MMR = 0;
  m_pCan->CAN_MB[uc_index].CAN_MAM = 0;
  m_pCan->CAN_MB[uc_index].CAN_MID = 0;
  m_pCan->CAN_MB[uc_index].CAN_MDL = 0;
  m_pCan->CAN_MB[uc_index].CAN_MDH = 0;
  m_pCan->CAN_MB[uc_index].CAN_MCR = 0;
}

/**
 * \brief Reset the eight mailboxes.
 *
 * \param m_pCan Pointer to a CAN peripheral instance.
 */
void CAN_SAM3X::reset_all_mailbox()
{
  for (uint8_t i = 0; i < CANMB_NUMBER; i++)
  {
    mailbox_init(i);
  }
}



/**
 * \brief Read a frame from out of the mailbox and into a software buffer
 *
 * \param uc_index which mailbox to read
 * \param rxframe Pointer to a receive frame structure which we'll fill out
 *
 * \retval Different CAN mailbox transfer status.
 *
 */
uint32_t CAN_SAM3X::mailbox_read(uint8_t uc_index, volatile CAN_Frame *rxframe)
{
  uint32_t ul_status;
  uint32_t ul_retval;
  uint32_t ul_id;
  uint32_t ul_datal, ul_datah;

  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;

  ul_retval = 0;
  ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;

  /* Check whether there is overwriting happening in Receive with Overwrite mode,
     or there're messages lost in Receive mode. */
  if ((ul_status & CAN_MSR_MRDY) && (ul_status & CAN_MSR_MMI))
  {
    ul_retval = CAN_MAILBOX_RX_OVER;
  }

  ul_id = m_pCan->CAN_MB[uc_index].CAN_MID;
  if ((ul_id & CAN_MID_MIDE) == CAN_MID_MIDE)   //extended id
  {
    rxframe->id = ul_id & CAN_EXTENDED_ID_MASK;
    rxframe->extended = true;
  }
  else   //standard ID
  {
    rxframe->id = (ul_id  >> CAN_MID_MIDvA_Pos) & CAN_STANDARD_ID_MASK;
    rxframe->extended = false;
  }
  rxframe->fid = m_pCan->CAN_MB[uc_index].CAN_MFID;
  rxframe->length = (ul_status & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
  ul_datal = m_pCan->CAN_MB[uc_index].CAN_MDL;
  ul_datah = m_pCan->CAN_MB[uc_index].CAN_MDH;
  rxframe->data[0] = (uint8_t)(ul_datal & 0xFF);
  rxframe->data[1] = (uint8_t)((ul_datal >> 8) & 0xFF);
  rxframe->data[2] = (uint8_t)((ul_datal >> 16) & 0xFF);
  rxframe->data[3] = (uint8_t)((ul_datal >> 24) & 0xFF);
  rxframe->data[4] = (uint8_t)(ul_datah & 0xFF);
  rxframe->data[5] = (uint8_t)((ul_datah >> 8) & 0xFF);
  rxframe->data[6] = (uint8_t)((ul_datah >> 16) & 0xFF);
  rxframe->data[7] = (uint8_t)((ul_datah >> 24) & 0xFF);

  /* Read the mailbox status again to check whether the software needs to re-read mailbox data register. */
  ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;
  if (ul_status & CAN_MSR_MMI)
  {
    ul_retval |= CAN_MAILBOX_RX_NEED_RD_AGAIN;
  }
  else
  {
    ul_retval |= CAN_MAILBOX_TRANSFER_OK;
  }

  /* Enable next receive process. */
  mailbox_send_transfer_cmd(uc_index);

  return ul_retval;
}


void CAN_SAM3X::mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (extended)
  {
    m_pCan->CAN_MB[uc_index].CAN_MID = id | CAN_MID_MIDE;
  }
  else
  {
    m_pCan->CAN_MB[uc_index].CAN_MID = CAN_MID_MIDvA(id);
  }
}

uint32_t CAN_SAM3X::mailbox_get_id(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (m_pCan->CAN_MB[uc_index].CAN_MID & CAN_MID_MIDE)
  {
    return m_pCan->CAN_MB[uc_index].CAN_MID;
  }
  else
  {
    return (m_pCan->CAN_MB[uc_index].CAN_MID >> CAN_MID_MIDvA_Pos) & 0x7ffu;
  }
}

void CAN_SAM3X::mailbox_set_priority(uint8_t uc_index, uint8_t pri)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (pri << CAN_MMR_PRIOR_Pos);
}

void CAN_SAM3X::mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (ext)
  {
    m_pCan->CAN_MB[uc_index].CAN_MAM = mask | CAN_MAM_MIDE;
    m_pCan->CAN_MB[uc_index].CAN_MID |= CAN_MAM_MIDE;
  }
  else
  {
    m_pCan->CAN_MB[uc_index].CAN_MAM = CAN_MAM_MIDvA(mask);
    m_pCan->CAN_MB[uc_index].CAN_MID &= ~CAN_MAM_MIDE;
  }
}

void CAN_SAM3X::mailbox_set_mode(uint8_t uc_index, uint8_t mode)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (mode > 5) mode = 0; //set disabled on invalid mode
  m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR &
                                      ~CAN_MMR_MOT_Msk) | (mode << CAN_MMR_MOT_Pos);
}

uint8_t CAN_SAM3X::mailbox_get_mode(uint8_t uc_index)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  return (uint8_t)(m_pCan->CAN_MB[uc_index].CAN_MMR >> CAN_MMR_MOT_Pos) & 0x7;
}

void CAN_SAM3X::mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val)
{
  uint8_t shift; //how many bits to shift
  uint32_t working;  //working copy of the relevant data int
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (bytepos > 7) bytepos = 7;
  shift = 8 * (bytepos & 3); //how many bytes to shift up into position
  if (bytepos < 4)   //low data block
  {
    working = m_pCan->CAN_MB[uc_index].CAN_MDL & ~(255 << shift); //mask out where we have to be
    working |= (val << shift);
    m_pCan->CAN_MB[uc_index].CAN_MDL = working;
  }
  else   //high data block
  {
    working = m_pCan->CAN_MB[uc_index].CAN_MDH & ~(255 << shift); //mask out where we have to be
    working |= (val << shift);
    m_pCan->CAN_MB[uc_index].CAN_MDH = working;
  }
}

void CAN_SAM3X::mailbox_set_datal(uint8_t uc_index, uint32_t val)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MDL = val;
}

void CAN_SAM3X::mailbox_set_datah(uint8_t uc_index, uint32_t val)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  m_pCan->CAN_MB[uc_index].CAN_MDH = val;
}


void CAN_SAM3X::mailbox_set_datalen(uint8_t uc_index, uint8_t dlen)
{
  if (uc_index > CANMB_NUMBER - 1) uc_index = CANMB_NUMBER - 1;
  if (dlen > 8) dlen = 8;
  m_pCan->CAN_MB[uc_index].CAN_MCR = (m_pCan->CAN_MB[uc_index].CAN_MCR &
                                      ~CAN_MCR_MDLC_Msk) | CAN_MCR_MDLC(dlen);
}

/* Set the RTR bit in the sent frame.
m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MRTR;
  */

/**
 * \brief Require to send out a frame.
 *
 * \param uc_index which mailbox to send frame. Load it up first
 *
 * \retval CAN_MAILBOX_NOT_READY: Failed because mailbox isn't ready for transmitting message.
 *       CAN_MAILBOX_TRANSFER_OK: Successfully send out a frame.
 */
uint32_t CAN_SAM3X::mailbox_tx_frame(uint8_t uc_index)
{
  uint32_t ul_status;

  /* Read the mailbox status firstly to check whether the mailbox is ready or not. */
  ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;
  if (!(ul_status & CAN_MSR_MRDY))
  {
    return CAN_MAILBOX_NOT_READY;
  }

  /* Set the MBx bit in the Transfer Command Register to send out the remote frame. */
  global_send_transfer_cmd((1 << uc_index));

  return CAN_MAILBOX_TRANSFER_OK;
}


/**
* \brief Find unused RX mailbox and return its number
*/
int CAN_SAM3X::findFreeRXMailbox()
{
  for (int c = 0; c < 8; c++)
  {
    if (mailbox_get_mode(c) == CAN_MB_RX_MODE)
    {
      if (mailbox_get_id(c) == 0)
      {
        return c;
      }
    }
  }
  return -1;
}

/**
* \brief Set up an RX mailbox (first free) for the given parameters.
*
* \param id - the post mask ID to match against
* \param mask - the mask to use for this filter
* \param extended - whether to use 29 bit filter
*
* \ret number of mailbox we just used (or -1 if there are no free boxes to use)
*/
int CAN_SAM3X::setRXFilter(uint32_t id, uint32_t mask, bool extended)
{
  int c = findFreeRXMailbox();
  if (c < 0) return -1;

  mailbox_set_accept_mask(c, mask, extended);
  mailbox_set_id(c, id, extended);
  enable_interrupt(getMailboxIer(c));

  return c;
}

/**
* \brief Set up an RX mailbox (given MB number) for the given parameters.
*
* \param pCan Which canbus hardware to use (CAN0 or CAN1)
* \param Rs pin to use for transceiver Rs control
* \param En pin to use for transceiver enable
*/
int CAN_SAM3X::setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
  if (mailbox > 7) return -1;

  mailbox_set_accept_mask(mailbox, mask, extended);
  mailbox_set_id(mailbox, id, extended);
  enable_interrupt(getMailboxIer(mailbox));
  return mailbox;
}

/*
 * Get the IER (interrupt mask) for the specified mailbox index.
 *
 * \param mailbox - the index of the mailbox to get the IER for
 * \retval the IER of the specified mailbox
 */
uint32_t CAN_SAM3X::getMailboxIer(int8_t mailbox)
{
  switch (mailbox)
  {
    case 0:
      return CAN_IER_MB0;
    case 1:
      return CAN_IER_MB1;
    case 2:
      return CAN_IER_MB2;
    case 3:
      return CAN_IER_MB3;
    case 4:
      return CAN_IER_MB4;
    case 5:
      return CAN_IER_MB5;
    case 6:
      return CAN_IER_MB6;
    case 7:
      return CAN_IER_MB7;
  }
  return 0;
}

/**
* \brief Handle a mailbox interrupt event
* \param mb which mailbox generated this event
*/
void CAN_SAM3X::mailbox_int_handler(uint8_t mb, uint32_t ul_status)
{
  if (mb > 7) mb = 7;
  if (m_pCan->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY)   //mailbox signals it is ready
  {
    switch (((m_pCan->CAN_MB[mb].CAN_MMR >> 24) & 7))   //what sort of mailbox is it?
    {
      case 1: //receive
      case 2: //receive w/ overwrite
      case 4: //consumer - technically still a receive buffer
        mailbox_read(mb, &rx_frame_buff[rx_buffer_head]);
        rx_buffer_head = (rx_buffer_head + 1) % SAM3X_SIZE_RX_BUFFER;
        break;
      case 3: //transmit
        if (tx_buffer_head != tx_buffer_tail)
        {
          //if there is a frame in the queue to send
          mailbox_set_id(mb, tx_frame_buff[tx_buffer_head].id, tx_frame_buff[tx_buffer_head].extended);
          mailbox_set_datalen(mb, tx_frame_buff[tx_buffer_head].length);
          mailbox_set_priority(mb, tx_frame_buff[tx_buffer_head].priority);
          for (uint8_t cnt = 0; cnt < 8; cnt++)
            mailbox_set_databyte(mb, cnt, tx_frame_buff[tx_buffer_head].data[cnt]);
          global_send_transfer_cmd((0x1u << mb));
          tx_buffer_head = (tx_buffer_head + 1) % SAM3X_SIZE_TX_BUFFER;
        }
        else
        {
          disable_interrupt(0x01 << mb);
        }
        break;
      case 5: //producer - technically still a transmit buffer
        break;
    }
  }
}

//Outside of object interrupt dispatcher. Needed because interrupt handlers
// can't really be members of a class
void CAN0_Handler(void)
{
  CAN.interruptHandler();
}
CAN_SAM3X CAN(0); // Create CAN channel on CAN bus 0

// Worry about multiple buses later
//void CAN1_Handler(void)
//{
//  CANbus1.interruptHandler();
//}
//CAN_SAM3X CANbus1(1);  // Create CAN channel on CAN bus 1

#endif // defined(__SAM3X8E__) // Arduino Due
