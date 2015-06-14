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
Fabian Greif for the initial SJA1000 library from https://github.com/dergraaf/avr-can-lib

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

#include <Arduino.h>
#include "CAN.h"
#include "CAN_SJA1000.h"

///////////////////////////////////////////////////////////////////
///							                                                ///
///	                 CAN library for SJA1000		           	    ///
///							                                                ///
///////////////////////////////////////////////////////////////////

//Initialize SPI communications and set MCP2515 into Config mode
CAN_SJA1000::CAN_SJA1000()
{
}

#endif // defined(ARDUINO_ARCH_AVR)
