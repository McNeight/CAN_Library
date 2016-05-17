# CAN_Library [![Build Status](https://travis-ci.org/McNeight/CAN_Library.svg?branch=master)](https://travis-ci.org/McNeight/CAN_Library)

This library supports the [Controller Area Network (CAN bus or CAN)](https://en.wikipedia.org/wiki/CAN_bus) and allows you to communicate with multiple types of CAN controllers using a consistent API, making CAN communications across development platforms easier through Arduino.

The idea behind this CAN library is to use a similar approach to [Adafruit's Unified Sensor library](https://github.com/adafruit/Adafruit_Sensor "Adafruit's Unified Sensor library") by standardizing CAN function calls, frame structure, filters, masks, buffers, etc to be used with a wide variety of CAN controllers. This library currently supports the following controllers:

* Microchip MCP2510 & MCP2515 through the SPI interface
  * Tested using both the [SparkFun CAN-BUS Shield](https://www.sparkfun.com/products/10039 "SparkFun CAN-BUS Shield")
  as well as the [Seeed Studio CAN-BUS Shield](http://www.seeedstudio.com/depot/CANBUS-Shield-p-2256.html "Seeed Studio CAN-BUS Shield")
* Atmel SAM3X family of MCU
  * Tested using the SAM3X8E on the [Arduino Due](http://www.arduino.cc/en/Main/ArduinoBoardDue "Arduino Due")
* Freescale Kinetis K2x family of MCU
  * Tested using the MK20DX256VLH7 on the [PJRC Teensy 3.1](http://www.pjrc.com/teensy/teensy31.html "PJRC Teensy 3.1")

Since I have taken features from so many libraries I can’t tell what came from where, so in order not to violate any GPL, LGPL or any other license out there I am trying to give credit where credit is due. I am only a contributor to this library and some of the work here might not be mine. I can take credit in putting it together and releasing back to the public to make any variation as needed.

To start using this library, simply add the following two lines to your sketch:
```Arduino
#include <CAN.h>
#include <SPI.h> // required to resolve #define conflicts
```

Acknowledgements:
* Fabian Greif for the initial libraries for MCP2515, SJA1000 and AT90CAN
  * http://www.kreatives-chaos.com/artikel/universelle-can-bibliothek as well as his updates at https://github.com/dergraaf/avr-can-lib
* David Harding for his version of the MCP2515 library
  * http://forum.arduino.cc/index.php/topic,8730.0.html
* Kyle Crockett CANduino library with 16Mhz oscillator
  * http://code.google.com/p/canduino/
* Nuno Alves for the help on Extended ID messaging
* Stevenh for his work on library and all of the MCP research/work
  * http://modelrail.otenko.com/arduino/arduino-controller-area-network-can
* Collin Kidder (collin80) for his work on the Arduino Due CAN interface
  * https://github.com/collin80/due_can
* Daniel Kasamis (togglebit) both for his Due code
  * https://github.com/togglebit/ArduinoDUE_OBD_FreeRunningCAN
* as well as his DUE CANshield
  * http://togglebit.net/product/arduino-due-can-shield/
* Cory Fowler (coryjfowler) for 16 MHz bitrate timing information
  * https://github.com/coryjfowler/MCP2515_lib
* teachop for the FlexCAN library for the Teensy 3.1
  * https://github.com/teachop/FlexCAN_Library
