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

	Acknowledgments:
	Fabian Greif for the initial MCP2515 library http://www.kreatives-chaos.com/artikel/universelle-can-bibliothek
	David Harding for his version of the MCP2515 library http://forum.arduino.cc/index.php/topic,8730.0.html
	Kyle Crockett CANduino library with 16Mhz oscillator. (http://code.google.com/p/canduino/)
	Nuno Alves for the help on Extended ID messaging
	Stevenh for his work on library and all of the MCP research/work http://modelrail.otenko.com/arduino/arduino-controller-area-network-can

-------------------------------------------------------------------------------------------------------------
Change Log

DATE		VER		WHO			WHAT
07/07/13	0.1		PC		Modified and merge all MCP2515 libraries found. Stripped away most unused fuctions and corrected MCP2515 defs
09/12/13	0.2		PC		Added selectable CS SPI for CAN controller to use 1 IC to control several mcp2515
02/05/14	0.3		PC		Added filter and mask controls
05/01/14	0.4		PC		Cleaned up fuctions, variables and added message structures for J1939, CANopen and CAN. 
-------------------------------------------------------------------------------------------------------------

Features:

CAN V2.0B
8 byte length in the data field
Standard and extended data frames
Two receive buffers
Three Transmit Buffers
SPI Interface 

Supported Baud rates 
10 kpbs; 20 kpbs; 50 kbps; 100 kbps; 125 kbps; 250 kbps; 500 kbps; 1000 kbps

Desgined to be used with ATMEL ATMega328P with Arduino bootlader, MCP2515 Stand-Alone CAN Controller and MCP2561 High-Speed CAN Transceivers.
*/



#ifndef CAN_H_
#define CAN_H_

#include "Arduino.h"

//SPI Interface functions
#define RESET 0xC0  
#define READ 0x03 
#define WRITE 0x02 
#define READ_STATUS 0xA0
#define RX_STATUS 0xB0
#define BIT_MODIFY 0x05


//SPI functions to read CAN RX Buffers
#define READ_RX_BUF_0_ID 0x90
#define READ_RX_BUF_0_DATA 0x92
#define READ_RX_BUF_1_ID 0x94
#define READ_RX_BUF_1_DATA 0x96 

//SPI functions to load CAN TX Buffers
#define LOAD_TX_BUF_0_ID 0x40
#define LOAD_TX_BUF_0_DATA 0x41
#define LOAD_TX_BUF_1_ID 0x42
#define LOAD_TX_BUF_1_DATA 0x43
#define LOAD_TX_BUF_2_ID 0x44
#define LOAD_TX_BUF_2_DATA 0x45 

//SPI functions RTS (Request to send) messages from CAN TX buffers.
#define SEND_TX_BUF_0 0x81
#define SEND_TX_BUF_1 0x82
#define SEND_TX_BUF_2 0x84 
#define SEND_ALL 0X87


// MCP2515 CAN CONTROLLER REGISTERS. SEE MCP2515 DATASHEET SECTION 11.0 FOR FURTHER EXPLANATION 
// http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en010406

#define RXF0SIDH	0x00
#define RXF0SIDL	0x01
#define RXF0EID8	0x02
#define RXF0EID0	0x03
#define RXF1SIDH	0x04
#define RXF1SIDL	0x05
#define RXF1EID8	0x06
#define RXF1EID0	0x07
#define RXF2SIDH	0x08
#define RXF2SIDL	0x09
#define RXF2EID8	0x0A
#define RXF2EID0	0x0B
#define BFPCTRL		0x0C
#define TXRTSCTRL	0x0D
#define CANSTAT		0x0E
#define CANCTRL		0x0F

#define RXF3SIDH	0x10
#define RXF3SIDL	0x11
#define RXF3EID8	0x12
#define RXF3EID0	0x13
#define RXF4SIDH	0x14
#define RXF4SIDL	0x15
#define RXF4EID8	0x16
#define RXF4EID0	0x17
#define RXF5SIDH	0x18
#define RXF5SIDL	0x19
#define RXF5EID8	0x1A
#define RXF5EID0	0x1B
#define TEC			0x1C
#define REC         0x1D

#define RXM0SIDH	0x20
#define RXM0SIDL	0x21
#define RXM0EID8	0x22
#define RXM0EID0	0x23
#define RXM1SIDH	0x24
#define RXM1SIDL	0x25
#define RXM1EID8	0x26
#define RXM1EID0	0x27
#define CNF3		0x28
#define CNF2		0x29
#define CNF1		0x2A
#define CANINTE		0x2B
#define CANINTF		0x2C
#define EFLG		0x2D

#define TXB0CTRL	0x30
#define TXB0SIDH	0x31
#define TXB0SIDL	0x32
#define TXB0EID8	0x33
#define TXB0EID0	0x34
#define TXB0DLC		0x35
#define TXB0D0		0x36
#define TXB0D1		0x37
#define TXB0D2		0x38
#define TXB0D3		0x39
#define TXB0D4		0x3A
#define TXB0D5		0x3B
#define TXB0D6		0x3C
#define TXB0D7		0x3D

#define TXB1CTRL	0x40
#define TXB1SIDH	0x41
#define TXB1SIDL	0x42
#define TXB1EID8	0x43
#define TXB1EID0	0x44
#define TXB1DLC		0x45
#define TXB1D0		0x46
#define TXB1D1		0x47
#define TXB1D2		0x48
#define TXB1D3		0x49
#define TXB1D4		0x4A
#define TXB1D5		0x4B
#define TXB1D6		0x4C
#define TXB1D7		0x4D

#define TXB2CTRL	0x50
#define TXB2SIDH	0x51
#define TXB2SIDL	0x52
#define TXB2EID8	0x53
#define TXB2EID0	0x54
#define TXB2DLC		0x55
#define TXB2D0		0x56
#define TXB2D1		0x57
#define TXB2D2		0x58
#define TXB2D3		0x59
#define TXB2D4		0x5A
#define TXB2D5		0x5B
#define TXB2D6		0x5C
#define TXB2D7		0x5D

#define RXB0CTRL	0x60
#define RXB0SIDH	0x61
#define RXB0SIDL	0x62
#define RXB0EID8	0x63
#define RXB0EID0	0x64
#define RXB0DLC		0x65
#define RXB0D0		0x66
#define RXB0D1		0x67
#define RXB0D2		0x68
#define RXB0D3		0x69
#define RXB0D4		0x6A
#define RXB0D5		0x6B
#define RXB0D6		0x6C
#define RXB0D7		0x6D

#define RXB1CTRL	0x70
#define RXB1SIDH	0x71
#define RXB1SIDL	0x72
#define RXB1EID8	0x73
#define RXB1EID0	0x74
#define RXB1DLC		0x75
#define RXB1D0		0x76
#define RXB1D1		0x77
#define RXB1D2		0x78
#define RXB1D3		0x79
#define RXB1D4		0x7A
#define RXB1D5		0x7B
#define RXB1D6		0x7C
#define RXB1D7		0x7D

// MESSAGE TRANSMISSION REGISTER BIT DEFINITIONS. 

// TXBnCTRL – TRANSMIT BUFFER n CONTROL REGISTER (ADDRESS: 30h, 40h, 50h)
#define ABTF		6
#define MLOA		5
#define TXERR		4
#define TXREQ		3
#define TXP1		1
#define TXP0		0

// TXRTSCTRL – TXnRTS PIN CONTROL AND STATUS REGISTER (ADDRESS: 0Dh)
#define B2RTS		5
#define B1RTS		4
#define B0RTS		3
#define B2RTSM		2
#define B1RTSM		1
#define B0RTSM		0

// TXBnSIDH – TRANSMIT BUFFER n STANDARD IDENTIFIER HIGH (ADDRESS: 31h, 41h, 51h)
#define SID10		7
#define SID9		6
#define SID8		5
#define SID7		4
#define SID6		3
#define SID5		2
#define SID4		1
#define SID3		0

// TXBnSIDL – TRANSMIT BUFFER n STANDARD IDENTIFIER LOW (ADDRESS: 32h, 42h, 52h)
#define SID2		7
#define SID1		6
#define SID0		5
#define	EXIDE		3
#define EID17		1
#define EID16		0

// TXBnEID8 – TRANSMIT BUFFER n EXTENDED IDENTIFIER HIGH (ADDRESS: 33h, 43h, 53h)
#define EID15		7
#define EID14		6
#define EID13		5
#define EID12		4
#define EID11		3
#define EID10		2
#define EID9		1
#define EID8		0

// TXBnEID0 – TRANSMIT BUFFER n EXTENDED IDENTIFIER LOW (ADDRESS: 34h, 44h, 54h)
#define EID7		7
#define EID6		6
#define EID5		5
#define EID4		4
#define EID3		3
#define EID2		2
#define EID1		1
#define EID0		0

// TXBnDLC - TRANSMIT BUFFER n DATA LENGTH CODE (ADDRESS: 35h, 45h, 55h)
#define RTR			6
#define DLC3		3
#define DLC2		2
#define DLC1		1
#define DLC0		0


// MESSAGE RECEPTION REGISTER BIT DEFINITIONS.

// RXB0CTRL – RECEIVE BUFFER 0 CONTROL (ADDRESS: 60h)
#define RXM1		6
#define RXM0		5
#define RXRTR		3
#define BUKT		2
#define BUKT1		1
#define FILHIT0		0

// RXB1CTRL – RECEIVE BUFFER 1 CONTROL (ADDRESS: 70h). SAME AS RXB0CTRL BUT WITH THE FOLLOWING CHANGES
#define FILHIT2		2
#define FILHIT1		1

// BFPCTRL – RXnBF PIN CONTROL AND STATUS (ADDRESS: 0Ch)
#define B1BFS		5
#define B0BFS		4
#define B1BFE		3
#define B0BFE		2
#define B1BFM		1
#define B0BFM		0

// RXBnSIDH – RECEIVE BUFFER n STANDARD IDENTIFIER HIGH (ADDRESS: 61h, 71h)
#define SID10		7
#define SID9		6
#define SID8		5
#define SID7		4
#define SID6		3
#define SID5		2
#define SID4		1
#define SID3		0

// RXBnSIDL – RECEIVE BUFFER n STANDARD IDENTIFIER LOW (ADDRESS: 62h, 72h)
#define SID2		7
#define SID1		6
#define SID0		5
#define	SRR			4
#define	IDE			3
#define	EID17		1		
#define	EID16		0

// RXBnEID8 – RECEIVE BUFFER n EXTENDED IDENTIFIER HIGH (ADDRESS: 63h, 73h)
#define	EID15		7
#define	EID14		6
#define	EID13		5		
#define	EID12		4
#define	EID11		3
#define	EID10		2
#define	EID9		1
#define	EID8		0

// RXBnEID0 – RECEIVE BUFFER n EXTENDED IDENTIFIER LOW (ADDRESS: 64h, 74h)
#define	EID7		7
#define	EID6		6
#define	EID5		5		
#define	EID4		4
#define	EID3		3
#define	EID2		2
#define	EID1		1
#define	EID0		0

// RXBnDLC – RECEIVE BUFFER n DATA LENGTH CODE (ADDRESS: 65h, 75h)
#define	RTR			6
#define	DLC3		3
#define	DLC2		2
#define	DLC1		1
#define DLC0		0

// CONFIGURATION REGISTER BIT DEFINITIONS.

// CNF1 – CONFIGURATION 1 (ADDRESS: 2Ah)
#define SJW1		7
#define SJW0		6
#define BRP5		5
#define BRP4		4
#define BRP3		3
#define BRP2		2
#define BRP1		1
#define BRP0		0

// CNF2 – CONFIGURATION 2 (ADDRESS: 29h)
#define BTLMODE		7
#define SAM			6
#define PHSEG12		5
#define PHSEG11		4
#define PHSEG10		3
#define PRSEG2		2		
#define PRSEG1		1
#define PRSEG0		0

// CNF3 - CONFIGURATION 1 (ADDRESS: 28h)
#define SOF			7		
#define WAKFIL		6
#define PHSEG22		2
#define PHSEG21		1
#define PHSEG20		0

// CANCTRL – CAN CONTROL REGISTER (ADDRESS: 0Fh)
#define REQOP2		7
#define REQOP1		6
#define REQOP0		5
#define ABAT		4
#define OSM			3		
#define CLKEN		2
#define CLKPRE1		1
#define CLKPRE0		0

// CANSTAT – CAN STATUS REGISTER (ADDRESS: 0Eh)
#define OPMOD2		7
#define OPMOD1		6
#define OPMOD0		5
#define ICOD2		3
#define ICOD1		2
#define ICOD0		1

// EFLG – ERROR FLAG (ADDRESS: 2Dh)
#define RX1OVR		7
#define RX0OVR		6
#define TXB0		5
#define TXEP		4
#define RXEP		3
#define TXWAR		2
#define RXWAR		1
#define EWARN		0

// CANINTE – INTERRUPT ENABLE (ADDRESS: 2Bh)
#define MERRE		7
#define WAKIE		6
#define ERRIE		5
#define TX2IE		4
#define TX1IE		3
#define TX0IE		2
#define RX1IE		1
#define RX0IE		0

// CANINTF – INTERRUPT FLAG (ADDRESS: 2Ch)
#define MERRF		7
#define WAKIF		6
#define ERRIF		5
#define TX2IF		4
#define TX1IF		3
#define TX0IF		2
#define RX1IF		1
#define RX0IF		0

//FILTER AND MASK ADDRESES.

#define mask0 0x20
#define mask1 0x24
#define filter0 0x00
#define filter1 0x04
#define filter2 0x08
#define filter3 0x10
#define filter4 0x14
#define filter5 0x18


//MCP2515 MODES
#define CONFIG		1
#define NORMAL		2
#define SLEEP		3
#define LISTEN		4
#define LOOPBACK	5

#define YES 1
#define NO 0
#define FAIL 0
#define INIT 1
#define ERROR 0


//buffer definitions
#define buffer1		1
#define buffer2		2
#define buffer3		3
#define allBuffers  0

#define extID 1
#define stdID 0

// CAN Message Structures
typedef struct
{
	unsigned long ID; // Full ID
	byte rtr; // Remote Trasnmission Request
	byte length; // Data Length
	byte data[8]; // Message data
} CAN;


// J1939 Message Structures
typedef struct
{
	unsigned long ID; // Full Identifier
	byte PRIO; // Message priority
	unsigned short PGN; //Parameter Group Number
	byte rtr; // Remote Trasnmission Request
	byte SA; // Source Address
	byte DA; // Destination Address
	byte DLC; // Data length code
	byte data[8];// Message data
} J1939;

// CANopen Message Structures
typedef struct
{
	unsigned long COB_ID; // Communication object identifier
	unsigned short FC; // Function Code
	unsigned short NODE; // Node
	byte rtr; // Remote Trasnmission Request
	byte length; // Data length 
	byte data[8]; // Message Data
} CANopen;


// MCP class
class MCP
{
	public:
	
	
	MCP(byte CS_Pin);// SPI CS is selectable through sketch. Allows multiple CAN channels. 
	void begin (int mode, int CANspeed);// Initializes CAN communications. Note it also starts SPI communications
	void bitRate(int CANspeed);//sets up CAN bit rate
	void setMode(int mode) ;//puts CAN controller in one of five modes
	void reset(); //CAN software reset. Also puts MCP2515 into config mode
	byte readMode(); // reads CAN mode	
	unsigned short readRate(); // reads CANspeed
	
	void writeAddress(byte address, byte value);// writes MCP2515 register addresses
	byte readAddress(byte address); //reads MCP2515 registers
	void modifyAddress(byte address, byte mask, byte value); // MCP2515 SPI bit modification commands
	
	void clearRxBuffers(); // clears all receive buffers
	void clearTxBuffers(); // clears all receive buffers
	void clearFilters();   // clears all filters and masks 

	byte readStatus(); //reads several status bits for transmit and receive functions.
	byte readRXStatus(); //reads receive fuctions and filhits
	
	void loadMsg(byte buffer, unsigned long ID, byte frameType, byte length, byte *data); //Load Standard Data Frame Message into TX buffer X. Note this only load message to buffer. RTS is needed to send message

	void sendTx(byte buffer); //(RTS) Request to send individual TX buffers or all 
		
	void send(unsigned long ID,byte frameType, byte length,byte *data); // Load and send message. No RTS needed. 
	
	void read(unsigned long *ID, byte *length_out, byte *data_out); // Receive and display any message (J1939, CANopen, CAN) 
	void read(CAN *message); //Receive and display CAN message and allows use of the message structure for easier message handling  
	void read(J1939 *message); //Receive and display j1939 message and allows use of the message structure for easier message handling 
	void read(CANopen *message); //Receive and display CANopen message and allows use of the message structure for easier message handling 
		
	bool msgAvailable(); // check if message has been recieved on any of the buffers	
	

	// OTHER USEFUL FUNCTIONS

	void enableRTSPins (); // Enable hardware request to send (RTS) pins if they are available. It allows messages to be send by driving MCP2515 RTS pins low.  
	void setInterrupts(byte mask, byte writeVal); //Enable/disable interrupts
	void setMask(byte mask, byte b0, byte b1, byte b2, byte b3); // Set Masks for filters
	void setFilter(byte filter, byte b0, byte b1, byte b2, byte b3); //Set Receive filters
		
	private:
		byte CS; //SPI CS is selectable through sketch
	
};

#endif