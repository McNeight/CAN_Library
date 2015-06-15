LICENSING
=========

The licensing of this library is a bit of a pain, as it draws from
so many other open source projects. As code gets merged, cleaned up,
refactored, and then expanded upon, it can be hard to determine who
wrote which line. Attempting to maintain every single copyright header
was bulky and of questionable value. 

So, along with the acknowledgements in the README.md file, I've attempted
to list here the multiple sources that this code was pulled from, the authors
or contributors, and the license that was chosen by them.

I've chosen to license my contributions to this library under the LGPL 2.1.
After reading http://www.gnu.org/licenses/why-not-lgpl.html and considering
the argument against using the LGPL, I'm using it for three reasons:

#1 Compatibility with the LGPL 2.1 license used for Arduino's core libraries
#2 Plenty of free and non-free CAN libraries already exist, meaning that it
    does not "give free software any particular advantage"
#3 Section 7 explicitly details how to properly credit source that is combined
    with other source in a library

* avr-can-lib (https://github.com/dergraaf/avr-can-lib)
  * Fabian Greif
    * "2-clause BSD license" or FreeBSD license
```Arduino
/*
 * Copyright (c) 2007 Fabian Greif, Roboterclub Aachen e.V.
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
```
* MCP2515 library (http://forum.arduino.cc/index.php/topic,8730.0.html)
  * David Harding
    * No license given, and no copyright listed.

* CANduino library (http://code.google.com/p/canduino/)
  * Kyle Crockett
    * No license given, and no copyright listed.

* MCP2515 library (http://www.nunoalves.com/open_source/?p=475)
  * Nuno Alves
    * No license given, and no copyright listed.

* MCP2515 library (http://modelrail.otenko.com/arduino/arduino-controller-area-network-can)
  * Stevenh (real name?)
    * No license given, and no copyright listed.

* due_can (https://github.com/collin80/due_can)
  * Collin Kidder (collin80) for his work on the Arduino Due CAN interface
    * GNU Lesser General Public License (LGPL) version 2.1
```Arduino
/*
  Copyright (c) 2013 Arduino.  All right reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
```

* ArduinoDUE_OBD_FreeRunningCAN (https://github.com/togglebit/ArduinoDUE_OBD_FreeRunningCAN)
  * Daniel Kasamis
    * GNU Lesser General Public License (LGPL) version 2.1
    * However, without a copyright statement
```Arduino
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
```

* MCP_CAN_lib (https://github.com/coryjfowler/MCP_CAN_lib)
  * Cory Fowler
    * GNU Lesser General Public License (LGPL) version 2.1
/*
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  
  Author:Loovee
  Contributor: Cory J. Fowler
  2014-1-16
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/

* FlexCAN_Library (https://github.com/teachop/FlexCAN_Library)
  * teachop (real name?)
    * No license given, and no copyright listed.
