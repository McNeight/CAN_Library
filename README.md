CAN-Library
===========

Yet another CAN library for Arduino using MCP2515 controller.

I have tried most of the MCP2515 libraries out there and found that some were either: shield specific, purpose specific or protocol specific. These libraries were also very complicated and brought too many features out to the sketch that could be performed within the library.  These libraries did have some good features and I have tried to take the best features out there and put them here. I have also made it more “Arduino” user friendly by using known functions like read(), send(), begin(). 

Since I have taking features from so many libraries I can’t tell what came from where, so in order not to violate any GPL,  LGPL or any other license out there I am trying to give credit where credit is due. I am only a contributor to this library and some of the work here might not be mine. I can take credit in putting it together and releasing back to the public to make any variation as needed.

I will add more Arduino examples using CAN messages either CAN, J1939 or OBD2 (CAN) in the future. 

Features:

CAN V2.0B
8 byte length in the data field
Standard and extended data frames
Two receive buffers
Three Transmit Buffers
SPI Interface with selectable CS via Arduino Sketch

Supported Bit rates (Confirmed and tested with Vector CANalyzer and Peak pCAN usb)

10 kpbs; 20 kpbs; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

Intended to be used with ATMEL ATMega328P with Arduino bootlader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers. Have not tested with other Arduino ATMEL chips but it will more likely work.

Message structure allows to read messages for CAN (standard, extended), J1939 and CANopen with the correct message format.

Selectable SPI CS pin allows to use library with most of the MCP2515 CANshield variations out there (Sparkfun shield, Seeedstudio, breadboard CAN, etc)



