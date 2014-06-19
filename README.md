This library allows you to communicate with multiple types of CAN controllers
using a consistent API, making CAN communications easier through Arduino.
 
The idea behind this CAN library is to use a similar approach to Adafruit's
Unified Sensor library (https://github.com/adafruit/Adafruit_Sensor) by
standardizing CAN function calls, frame structure, filters, masks, buffers, etc
to be used with a wide variety of CAN controllers. This library currently
supports the following controllers:

* Microchip MCP2510 & MCP2515 through the SPI interface
  * Tested using various Arduino and Arduino-compatible controllers
* Atmel SAM3X family of MCU
  * Tested using the SAM3X8E on the Arduino Due
* Freescale Kinetis K2x family of MCU
  * Tested using the MK20DX256VLH7 on the PJRC Teensy 3.1

Since I have taken features from so many libraries I can’t tell what came
from where, so in order not to violate any GPL, LGPL or any other license
out there I am trying to give credit where credit is due. I am only a
contributor to this library and some of the work here might not be mine.
I can take credit in putting it together and releasing back to the public
to make any variation as needed. This is a work in progress library with
no current release.

For the MCP2515 library release see CAN_MCP2515 branch on this repo 

```Arduino
#include <Arduino.h>
#include <CAN.h>

#if defined(ARDUINO_ARCH_AVR) // Arduino with SPI interface to MCP2515 chip
#include <SPI.h>
#include <CAN_MCP2515.h>
#elif defined(ARDUINO_ARCH_SAM) // Arduino Due
#include <CAN_SAM3X.h>
#elif defined(__MK20DX256__) // Teensy 3.1
#include <CAN_K2X.h>
#else
#error “Your CAN controller is currently unsupported.”
#endif
```
