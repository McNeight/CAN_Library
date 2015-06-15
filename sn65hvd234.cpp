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


#if defined(ARDUINO_ARCH_SAM)

/**
  * \file
  *
  * Implementation of the SN65HVD234 drivers.
  *
  */

#include "sn65hvd234.h"

#include <string.h>

/**
 * \brief Initialize SN65HVD234 component data
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::Init()
{
  dwPin_Rs = 0u ;
  dwPin_EN = 0u ;

  return 0u ;
}


SSN65HVD234::SSN65HVD234(uint32_t Rs, uint32_t En)
{
  dwPin_Rs = Rs ;
  pinMode(dwPin_Rs, OUTPUT ) ;
  dwPin_EN = En;
  pinMode( dwPin_EN, OUTPUT ) ;
}

/**
 * \brief Initialize Rs pin of transceiver
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 * \param pPIO_Rs       pointer on PIOx base for transceiver Rs pin
 * \param dwPin_Rs      PIO pin index for transceiver Rs pin
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::SetRs(uint32_t Rs )
{
  dwPin_Rs = Rs ;

  pinMode(dwPin_Rs, OUTPUT ) ;

  return 0u ;
}

/**
 * \brief Initialize EN pin of transceiver
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 * \param pPIO_EN       pointer on PIOx base for transceiver EN pin
 * \param dwPin_EN      PIO pin index for transceiver EN pin
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::SetEN(uint32_t EN )
{
  dwPin_EN = EN ;

  pinMode( dwPin_EN, OUTPUT ) ;

  return 0u ;
}

/**
 * \brief Enable transceiver
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::Enable()
{
  // Raise EN of SN65HVD234 to High Level (Vcc)
  digitalWrite(dwPin_EN, HIGH ) ;

  return 0u ;
}

/**
 * \brief Disable transceiver
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::Disable()
{
  // Lower EN of SN65HVD234 to Low Level (0.0v)
  digitalWrite(dwPin_EN, LOW ) ;

  return 0u ;
}

/**
 * \brief Turn component into lowpower mode
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::EnableLowPower()
{
  // Raise Rs of SN65HVD234 to more than 0.75v
  digitalWrite(dwPin_Rs, HIGH ) ;

  // Now, SN65HVD234 is only listening

  return 0u ;
}

/**
 * \brief Restore Normal mode by leaving lowpower mode
 *
 * \param pComponent    pointer on SSN65HVD234_Data
 *
 * \return              0 if OK
 */
uint32_t SSN65HVD234::DisableLowPower()
{
  // Lower Rs of SN65HVD234 to 0.0v < 0.33v
  digitalWrite(dwPin_Rs, LOW ) ;

  return 0u ;
}

#endif // defined(ARDUINO_ARCH_SAM)