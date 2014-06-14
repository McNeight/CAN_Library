// -------------------------------------------------------------
// a simple Arduino Teensy3.1 CAN driver
// by teachop
//

#if defined(__MK20DX256__) // Teensy 3.1

#include "CAN_K2X.h"

static const int txb = 8; // with default settings, all buffers before this are consumed by the FIFO
static const int txBuffers = 8;
static const int rxb = 0;

// -------------------------------------------------------------
CAN_K2X::CAN_K2X()
{
  // set up the pins, 3=PTA12=CAN0_TX, 4=PTA13=CAN0_RX
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);// | PORT_PCR_PE | PORT_PCR_PS;
  // select clock source
  SIM_SCGC6 |=  SIM_SCGC6_FLEXCAN0;
  FLEXCAN0_CTRL1 |= FLEXCAN_CTRL_CLK_SRC;

  // enable CAN
  FLEXCAN0_MCR |=  FLEXCAN_MCR_FRZ;
  FLEXCAN0_MCR &= ~FLEXCAN_MCR_MDIS;
  while (FLEXCAN0_MCR & FLEXCAN_MCR_LPM_ACK)
    ;
  // soft reset
  FLEXCAN0_MCR ^=  FLEXCAN_MCR_SOFT_RST;
  while (FLEXCAN0_MCR & FLEXCAN_MCR_SOFT_RST)
    ;
  // wait for freeze ack
  while (!(FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK))
    ;
  // disable self-reception
  FLEXCAN0_MCR |= FLEXCAN_MCR_SRX_DIS;

  //enable RX FIFO
  FLEXCAN0_MCR |= FLEXCAN_MCR_FEN;

  // Default mask is allow everything
  defaultMask.rtr = 0;
  defaultMask.ext = 0;
  defaultMask.id = 0;
}


// -------------------------------------------------------------
void CAN_K2X::end(void)
{
  // enter freeze mode
  FLEXCAN0_MCR |= (FLEXCAN_MCR_HALT);
  while (!(FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK))
    ;
}


// -------------------------------------------------------------
void CAN_K2X::begin(uint32_t bitrate)
{
  // segment timings from freescale loopback test
  if ( 250000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(3) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(15));
  }
  else if ( 500000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(1)
                      | FLEXCAN_CTRL_PSEG1(3) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(7));
  }
  else if ( 1000000 == bitrate )
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(3) | FLEXCAN_CTRL_RJW(0)
                      | FLEXCAN_CTRL_PSEG1(0) | FLEXCAN_CTRL_PSEG2(1) | FLEXCAN_CTRL_PRESDIV(5));
  }
  else     // 125000
  {
    FLEXCAN0_CTRL1 = (FLEXCAN_CTRL_PROPSEG(2) | FLEXCAN_CTRL_RJW(2)
                      | FLEXCAN_CTRL_PSEG1(3) | FLEXCAN_CTRL_PSEG2(3) | FLEXCAN_CTRL_PRESDIV(31));
  }

  FLEXCAN0_RXMGMASK = 0;

  //enable reception of all messages that fit the mask
  if (mask.ext)
  {
    FLEXCAN0_RXFGMASK = ((mask.rtr ? 1 : 0) << 31) | ((mask.ext ? 1 : 0) << 30) | ((mask.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
  }
  else
  {
    FLEXCAN0_RXFGMASK = ((mask.rtr ? 1 : 0) << 31) | ((mask.ext ? 1 : 0) << 30) | (FLEXCAN_MB_ID_IDSTD(mask.id) << 1);
  }

  // start the CAN
  FLEXCAN0_MCR &= ~(FLEXCAN_MCR_HALT);
  // wait till exit of freeze mode
  while (FLEXCAN0_MCR & FLEXCAN_MCR_FRZ_ACK);

  // wait till ready
  while (FLEXCAN0_MCR & FLEXCAN_MCR_NOT_RDY);

  //set tx buffers to inactive
  for (int i = txb; i < txb + txBuffers; i++)
  {
    FLEXCAN0_MBn_CS(i) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  }
}


// -------------------------------------------------------------
void CAN_K2X::setFilter(const CAN_Filter &filter, uint8_t n)
{
  if ( 8 > n )
  {
    if (filter.ext)
    {
      FLEXCAN0_IDFLT_TAB(n) = ((filter.rtr ? 1 : 0) << 31) | ((filter.ext ? 1 : 0) << 30) | ((filter.id & FLEXCAN_MB_ID_EXT_MASK) << 1);
    }
    else
    {
      FLEXCAN0_IDFLT_TAB(n) = ((filter.rtr ? 1 : 0) << 31) | ((filter.ext ? 1 : 0) << 30) | (FLEXCAN_MB_ID_IDSTD(filter.id) << 1);
    }
  }
}


// -------------------------------------------------------------
uint8_t CAN_K2X::available(void)
{
  //In FIFO mode, the following interrupt flag signals availability of a frame
  return (FLEXCAN0_IFLAG1 & FLEXCAN_IMASK1_BUF5M) ? 1 : 0;
}


// -------------------------------------------------------------
CAN_Frame CAN_K2X::read()
{
  unsigned long int startMillis;
  CAN_Frame msg;
  msg.timeout = CAN_TIMEOUT_T3;

  startMillis = msg.timeout ? millis() : 0;

  while ( !available() )
  {
    if ( !msg.timeout || (msg.timeout <= (millis() - startMillis)) )
    {
      // early EXIT nothing here
      msg.valid = false;
      return msg;
    }
    yield();
  }

  // get identifier and dlc
  msg.length = FLEXCAN_get_length(FLEXCAN0_MBn_CS(rxb));
  msg.extended = (FLEXCAN0_MBn_CS(rxb) & FLEXCAN_MB_CS_IDE) ? 1 : 0;
  msg.id  = (FLEXCAN0_MBn_ID(rxb) & FLEXCAN_MB_ID_EXT_MASK);
  if (!msg.extended)
  {
    msg.id >>= FLEXCAN_MB_ID_STD_BIT_NO;
  }

  // copy out message
  uint32_t dataIn = FLEXCAN0_MBn_WORD0(rxb);
  msg.data[3] = dataIn;
  dataIn >>= 8;
  msg.data[2] = dataIn;
  dataIn >>= 8;
  msg.data[1] = dataIn;
  dataIn >>= 8;
  msg.data[0] = dataIn;
  if ( 4 < msg.length )
  {
    dataIn = FLEXCAN0_MBn_WORD1(rxb);
    msg.data[7] = dataIn;
    dataIn >>= 8;
    msg.data[6] = dataIn;
    dataIn >>= 8;
    msg.data[5] = dataIn;
    dataIn >>= 8;
    msg.data[4] = dataIn;
  }
  for ( int loop = msg.length; loop < 8; ++loop )
  {
    msg.data[loop] = 0;
  }

  //notify FIFO that message has been read
  FLEXCAN0_IFLAG1 = FLEXCAN_IMASK1_BUF5M;

  msg.valid = true;
  return msg;
}

void CAN_K2X::flush()
{
}

// -------------------------------------------------------------
uint8_t CAN_K2X::write(const CAN_Frame &msg)
{
  unsigned long int startMillis;

  // Might as well check right off the bat
  if (msg.valid == false)
  {
    return 0;
  }
  startMillis = msg.timeout ? millis() : 0;

  // find an available buffer
  int buffer = -1;
  for ( int index = txb; ; )
  {
    if ((FLEXCAN0_MBn_CS(index) & FLEXCAN_MB_CS_CODE_MASK) == FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE))
    {
      buffer = index;
      break;// found one
    }
    if ( !msg.timeout )
    {
      if ( ++index >= (txb + txBuffers) )
      {
        return 0;// early EXIT no buffers available
      }
    }
    else
    {
      // blocking mode, only 1 txb used to guarantee frames in order
      if ( msg.timeout <= (millis() - startMillis) )
      {
        return 0; // timed out
      }
      yield();
    }
  }

  // transmit the frame
  FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_INACTIVE);
  if (msg.extended)
  {
    FLEXCAN0_MBn_ID(buffer) = (msg.id & FLEXCAN_MB_ID_EXT_MASK);
  }
  else
  {
    FLEXCAN0_MBn_ID(buffer) = FLEXCAN_MB_ID_IDSTD(msg.id);
  }
  FLEXCAN0_MBn_WORD0(buffer) = (msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3];
  FLEXCAN0_MBn_WORD1(buffer) = (msg.data[4] << 24) | (msg.data[5] << 16) | (msg.data[6] << 8) | msg.data[7];
  if (msg.extended)
  {
    FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                              | FLEXCAN_MB_CS_LENGTH(msg.length) | FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE;
  }
  else
  {
    FLEXCAN0_MBn_CS(buffer) = FLEXCAN_MB_CS_CODE(FLEXCAN_MB_CODE_TX_ONCE)
                              | FLEXCAN_MB_CS_LENGTH(msg.length);
  }

  return 1;
}

#endif // defined(__MK20DX256__) // Teensy 3.1
