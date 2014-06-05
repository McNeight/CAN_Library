// CANopen Message Structures
typedef struct
{
  uint16_t COB_ID		: 11; 	// Communication object identifier
  uint8_t FC			: 4;	// Function Code
  uint8_t NODE		: 7;	// Node
  uint8_t rtr			: 1;	// Remote Transmission Request
  uint8_t length		: 4;	// Data Length
  uint8_t data[8]; 			// Message data
} CAN_DATA_FRAME_CANopen;

uint8_t read(CAN_DATA_FRAME_CANopen *message); //Receive and display CANopen message and allows use of the message structure for easier message handling

//Receive and display CANopen message. This allows use of the message structure for easier message handling if
//communication object identifier (COB_ID), Function Code and Nodes are needed. See link for more info http://en.wikipedia.org/wiki/CANopen
void CAN_MCP2515::read(CAN_DATA_FRAME_CANopen *message)
{
  byte len, i, buffer, status;
  unsigned short sid_h, sid_l, eid8, eid0, rxbDlc;
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
    message->data[i] = SPI.transfer(0xFF);
  }
  digitalWrite(CS, HIGH);
  message->rtr = (bitRead(rxbDlc, 6));
  message->length = len;
  message->COB_ID = ((((unsigned short) sid_h) << 3) | ((sid_l & 0xE0) >> 5)); // Msg is standard frame. COB_ID
  message->FC = (((unsigned short) sid_h) << 3);// Contains CAN open Function Code
  message->NODE = ((sid_l & 0xE0) >> 5); // Contains CANopen Node
}
