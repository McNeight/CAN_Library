typedef union
{
  uint32_t id : 29;
  struct
  {
    uint8_t P : 3;   // Priority
    uint8_t EDP : 1; // Extended Data Page
    uint8_t DP : 1;  // Data Page
    uint8_t PF;      // Protocol Data Unit (PDU) Format
    union
    {
      uint8_t PS;      // Protocol Data Unit (PDU) Specific
      uint8_t DA;      // Destination Address (0 <= PS <= 239)
      uint8_t GE;      // Group Extension (240 <= PS <= 255)
    }
    uint8_t SA;      // Source Address
  }
} J1939Union;

typedef union
{
  uint32_t PGN : 24; // Parameter Group Number
  struct
  {
    uint8_t zero : 6; // 6 bits set to zero
    uint8_t EDP : 1; // Extended Data Page
    uint8_t DP : 1;  // Data Page
    uint8_t PF;      // Protocol Data Unit (PDU) Format
    uint8_t GE;      // Group Extension (240 <= PS <= 255)
  }
}

// SAE J1939 Message Structures
// Also:
//   ISO 11783 (ISOBUS or ISO Bus)
//   NMEA 2000
typedef struct
{
  uint32_t ID : 29; 			// Identifier
  uint8_t PRIO : 3; 		// Message priority
  uint16_t PGN; 	// Parameter Group Number
  uint8_t SA; 		// Source Address
  uint8_t DA; 		// Destination Address
  uint8_t DLC : 4; 		// Data length code
  uint8_t data[8];		// Message data
} CAN_DATA_FRAME_J1939;

uint8_t read(CAN_DATA_FRAME_J1939 *message); //Receive and display J1939 message and allows use of the message structure for easier message handling

//Receive and display J1939 messages. This allows use of the message structure for easier message handling if PGN, SA and DA are needed.
void CAN_MCP2515::read(CAN_DATA_FRAME_J1939 *message)
{
  byte len, i, buffer, status;
  unsigned short sid_h, sid_l, eid8, eid0, temp, rxbDlc;
  status = readStatus();
  if ((status & 0x01) == 0x01)
  {
    buffer = 0x90;
  }
  else if ((status & 0x02) == 0x02)
  {
    buffer = 0x94;
  }
  digitalWrite(CS, LOW);
  SPI.transfer(buffer);
  sid_h = SPI.transfer(0xFF); //id high
  sid_l = SPI.transfer(0xFF); //id low
  eid8 = SPI.transfer(0xFF); //extended id high
  eid0 = SPI.transfer(0xFF); //extended id low
  rxbDlc = SPI.transfer(0xFF); // get bits and other data from MCP2515 RXB DLC buffer (it contains data and RTR)
  len = ((rxbDlc) & 0x0F);
  //len = (SPI.transfer(0xFF) & 0x0F); //data length
  for (i = 0; i < len; i++)
  {
    message -> data[i] = SPI.transfer(0xFF);
  }
  digitalWrite(CS, HIGH);
  message -> DLC = len;
  if (bitRead ((sid_l), 3) == 1)   // check to see if this is an Extended ID Msg.
  {
    temp = (sid_l & 0xE0) >> 3; // keep SID0, SID1 and SID2 from SIDL
    sid_l = (sid_l & 0x03) | temp | ((sid_h & 0x07) << 5); //repack SIDL
    sid_h = ((unsigned short) sid_h >> 3); //shift SIDH
    message->ID = (((unsigned long) sid_h << 24) | ((unsigned long) sid_l << 16 ) | ((unsigned long) eid8 << 8) | (eid0 << 0)); // repack message ID
    message->PRIO = ((unsigned long) sid_h >> 2); // send this to display priority
    message->PGN = (((unsigned short) sid_l << 8 ) | ((unsigned short) eid8 << 0));// send this to display PGN
    message->SA = eid0;// send this to display Source Address
    message->DA = eid8;// send this to display Destination Address
  }
  //Standard frames not supported in J1939
  else
  {
    message->ID = 0; // not supported in J1939
    message->PRIO = 0;// not supported in J1939
    message->PGN = 0;// not supported in J1939
    message->SA = 0;// not supported in J1939
    message->DA = 0;// not supported in J1939
    message->DLC = 0;// not supported in J1939
  }
}
