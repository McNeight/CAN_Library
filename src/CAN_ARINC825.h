// The Logical Communication Channels (LCCs) are coded into the upper
// three bits [28:26] of the identifier fields
#define CAN_ARINC825_LCC_MASK          0x1C000000

// The Exception Event Channel (EEC) shall only be used for fast and high
// priority transmission to override other message transfers. These events will
// usually require some sort of immediate action (i.e., system degradation,
// shifting functions to other units or communicating the events to higher
// order systems). This channel is used for one-to-many communication.
#define CAN_ARINC825_LCC_EEC           0x0

// The Normal Operation Channel (NOC) shall be used for the transmission of
// aircraft operational periodic or aperiodic data based upon one-to-many
// communication. All data that is not assigned to another channel shall be
// transmitted over the NOC.
#define CAN_ARINC825_LCC_NOC           0x2

// The Node Service Channel (NSC) provides peer-to-peer communication for
// client/server type services. These services may be connectionless (no
// response transmitted) or connection-oriented (handshake type
// communication).
#define CAN_ARINC825_LCC_NSC           0x4

class CAN_ARINC825
{
}