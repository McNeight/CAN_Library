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
05/07/14	1.0		PC		Released Library to the public through GitHub
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

Designed to be used with ATMEL ATMega328P with Arduino bootloader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers.
*/

#include "CAN.h"
#include <avr/pgmspace.h>

void CanClass::constructor()
{
}
