#ifndef __MCP2515_H__
#define __MCP2515_H__

#include "type.h"

#define SSP_INTERFACE	1		// Select SSP-Interface to use (0 or 1)

#if SSP_INTERFACE==0
	#define SSP_CS		SSP0_CS
#elif SSP_INTERFACE==1
	#define SSP_CS		SSP1_CS
#endif

#define CAN_RX_INT_PORT			0
#define CAN_RX_INT_PIN			8

#define MCP_TIMEOUT				500		// Number of retries (one per millisecond) to switch Mode of MCP2515

/* CAN Sampling Timing */
#define TRX_TSJW_TQ				1
#define TRX_TPROP_TQ			1
#define TRX_TPSH1_TQ			3
#define TRX_TPSH2_TQ			3
#define TRX_DELAY_TQ			2		// Not defined in Datasheet, approx. 1-2TQ

/* Calculations for Register Values */

#define TSJW_LENGTH				(TRX_TSJW_TQ - 1) << SJW0
#define TPROP_LENGTH			(TRX_TPROP_TQ - 1) << PRSEG0
#define PHSEG1_LENGTH			(TRX_TPSH1_TQ - 1) << PHSEG10
#define PHSEG2_LENGTH			(TRX_TPSH2_TQ - 1) << PHSEG20

/* Plausibility Checks */

#if (TRX_TPROP_TQ + TRX_TPSH1_TQ < TRX_TPSH2_TQ)
	#warning PropSeg + PS1 must be greater than or equal to PS2
#endif

#if (TRX_TPSH2_TQ < TRX_TSJW_TQ)
	#warning PS2 must be greater than SJW
#endif

#if (TRX_TPROP_TQ + TRX_TPSH1_TQ < TRX_DELAY_TQ)
	#warning PS2 must be greater than or equal to T_Delay
#endif

/* Baudrate Prescaler Calculation */

#define TRX_TBIT_TQ				(1 + TRX_TPROP_TQ + TRX_TPSH1_TQ + TRX_TPSH2_TQ)
#define TRX_FOSC				16000000
#define BR_PRESCALER(BAUDRATE)	((uint8_t)((uint8_t)(TRX_FOSC / ( BAUDRATE * TRX_TBIT_TQ)) / 2) - 1)	// Calculate Precscaler Register

/* Defines for CAN Filter matching criteria */
#define CAN_FILTER_ALL_MSG	0x3
#define CAN_FILTER_ONLY_STD 0x2
#define CAN_FILTER_ONLY_EXT 0x1
#define CAN_FILTER_ALL_FRM	0x0

#pragma pack(1)		// Pack bitfields in one byte

/* Data Structures for easy access to important registers */
typedef struct
{
	uint8_t rx0if  : 1;
	uint8_t rx1if  : 1;
	uint8_t tx0req : 1;
	uint8_t tx0if  : 1;
	uint8_t tx1req : 1;
	uint8_t tx1if  : 1;
	uint8_t tx2req : 1;
	uint8_t tx2if  : 1;
} trx_int_status_;	// Data provided by the "READ STATUS" Command

typedef struct {
	uint8_t msg_buf  : 2;
	uint8_t rsv	   : 1;		// Placeholder for unused bit
	uint8_t msg_type : 2;
	uint8_t flt_mat  : 3;
} trx_rx_status_;	// Data provided by the "READ RX STATUS" Command

typedef struct {
	uint8_t rx1ovr : 1;
	uint8_t rx0ovr : 1;
	uint8_t txbo   : 1;
	uint8_t txep   : 1;
	uint8_t rxep   : 1;
	uint8_t txwar  : 1;
	uint8_t rxwar  : 1;
	uint8_t wwarn  : 1;
} trx_error_;	// EFLG Register	(Error Flags)

typedef struct {
	uint8_t opmod : 3;
	uint8_t rsv   : 1;		// Placeholder for unused bit
	uint8_t icod  : 3;
	uint8_t rsv_  : 1;		// Placeholder for unused bit
} trx_config_;	// CANSTAT Register	(MCP2515 Status)

typedef struct {
	uint8_t merrf : 1;
	uint8_t wakif : 1;
	uint8_t errif : 1;
	uint8_t tx2if : 1;
	uint8_t tx1if : 1;
	uint8_t tx0if : 1;
	uint8_t rx1if : 1;
	uint8_t rx0if : 1;
} trx_int_flags_;	// CANINTF Register	(Interrupt Flags)

typedef struct {
	uint8_t merre : 1;
	uint8_t wakie : 1;
	uint8_t errie : 1;
	uint8_t tx2ie : 1;
	uint8_t tx1ie : 1;
	uint8_t tx0ie : 1;
	uint8_t rx1ie : 1;
	uint8_t rx0ie : 1;
} trx_int_enable_;	// CANINTE Register (Interrupt Enable)

/* Initialisation */
void TRX_init(uint32_t baudrate);
void TRX_reset(void);

/* Basic SPI Communication Functions */
void TRX_write_register(uint8_t address, uint32_t size, uint8_t *data);
void TRX_read_register(uint8_t address, uint32_t size, uint8_t *data);
void TRX_bit_modify(uint8_t address, uint8_t mask, uint8_t data);

/* Command Functions */
void TRX_read_rx_buffer(uint8_t buffer_num, uint32_t *id, uint8_t *ext, uint8_t *rtr, uint8_t *size, uint8_t *data);
void TRX_load_tx_buffer(uint8_t buffer_num, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t size, uint8_t * data);
void TRX_RTS(uint8_t buffer_num);
void TRX_read_rx_status(uint8_t *rx_status);
void TRX_read_status(uint8_t *status);

/* Configuration Functions */
void TRX_set_baudrate(uint32_t baudrate);

void TRX_set_filter_mode(uint8_t buffer_num, uint8_t filter_mode);
void TRX_select_filter(uint8_t buffer_num, uint8_t filter_num, uint8_t extended);
void TRX_set_filter_id(uint8_t filter_num, uint32_t id, uint8_t extended);
void TRX_set_mask_id(uint8_t mask_num, uint32_t id, uint8_t extended);

/* Wrapper Functions for Interrupt Handling */
void TRX_enable_interrupts(uint8_t interrupts);
void TRX_disable_interrupts(uint8_t interrupts);
void TRX_clear_interrupts(uint8_t interrupts);
void TRX_read_interrupt(uint8_t * interrupt);

/* Operation Mode Functions */
void TRX_read_mode(uint8_t * mode);
void TRX_switch_mode(uint8_t mode);

/* Communication Behaviour Settings */
void TRX_abort_transmission(void);
void TRX_retry_transmission(uint8_t enable);

/* Registers of the MCP2515 */

#define RXF0SIDH	0x00		// FILTER 0 STANDARD IDENTIFIER HIGH
#define RXF0SIDL	0x01		// FILTER 0 STANDARD IDENTIFIER LOW
#define RXF0EID8	0x02		// FILTER 0 EXTENDED IDENTIFIER HIGH
#define RXF0EID0	0x03		// FILTER 0 EXTENDED IDENTIFIER LOW
#define RXF1SIDH	0x04		// FILTER 1 STANDARD IDENTIFIER HIGH
#define RXF1SIDL	0x05		// FILTER 1 STANDARD IDENTIFIER LOW
#define RXF1EID8	0x06		// FILTER 1 EXTENDED IDENTIFIER HIGH
#define RXF1EID0	0x07		// FILTER 1 EXTENDED IDENTIFIER LOW
#define RXF2SIDH	0x08		// FILTER 2 STANDARD IDENTIFIER HIGH
#define RXF2SIDL	0x09		// FILTER 2 STANDARD IDENTIFIER LOW
#define RXF2EID8	0x0A		// FILTER 2 EXTENDED IDENTIFIER HIGH
#define RXF2EID0	0x0B		// FILTER 2 EXTENDED IDENTIFIER LOW
#define BFPCTRL		0x0C		// RXnBF PIN CONTROL AND STATUS
#define TXRTSCTRL	0x0D		// TXnRTS PIN CONTROL AND STATUS REGISTER
#define CANSTAT		0x0E		// CAN Status Register
#define CANCTRL		0x0F		// CAN CONTROL REGISTER

#define RXF3SIDH	0x10		// FILTER 3 STANDARD IDENTIFIER HIGH
#define RXF3SIDL	0x11		// FILTER 3 STANDARD IDENTIFIER LOW
#define RXF3EID8	0x12		// FILTER 3 EXTENDED IDENTIFIER HIGH
#define RXF3EID0	0x13		// FILTER 3 EXTENDED IDENTIFIER LOW
#define RXF4SIDH	0x14		// FILTER 4 STANDARD IDENTIFIER HIGH
#define RXF4SIDL	0x15		// FILTER 4 STANDARD IDENTIFIER LOW
#define RXF4EID8	0x16		// FILTER 4 EXTENDED IDENTIFIER HIGH
#define RXF4EID0	0x17		// FILTER 4 EXTENDED IDENTIFIER LOW
#define RXF5SIDH	0x18		// FILTER 5 STANDARD IDENTIFIER HIGH
#define RXF5SIDL	0x19		// FILTER 5 STANDARD IDENTIFIER LOW
#define RXF5EID8	0x1A		// FILTER 5 EXTENDED IDENTIFIER HIGH
#define RXF5EID0	0x1B		// FILTER 5 EXTENDED IDENTIFIER LOW
#define TEC			0x1C		// Transmit Error Counter
#define REC         0x1D		// Receive	Error Counter

#define RXM0SIDH	0x20		// MASK 0 STANDARD IDENTIFIER HIGH
#define RXM0SIDL	0x21		// MASK 0 STANDARD IDENTIFIER LOW
#define RXM0EID8	0x22		// MASK 0 EXTENDED IDENTIFIER HIGH
#define RXM0EID0	0x23		// MASK 0 EXTENDED IDENTIFIER LOW
#define RXM1SIDH	0x24		// MASK 1 STANDARD IDENTIFIER HIGH
#define RXM1SIDL	0x25		// MASK 1 STANDARD IDENTIFIER LOW
#define RXM1EID8	0x26		// MASK 1 EXTENDED IDENTIFIER HIGH
#define RXM1EID0	0x27		// MASK 1 EXTENDED IDENTIFIER LOW
#define CNF3		0x28		// CONFIGURATION 1
#define CNF2		0x29		// CONFIGURATION 1
#define CNF1		0x2A		// CONFIGURATION 1
#define CANINTE		0x2B		// INTERRUPT ENABLE
#define CANINTF		0x2C		// INTERRUPT FLAG
#define EFLG		0x2D		// Error Flag

#define TXB0CTRL	0x30		// TRANSMIT BUFFER 0 CONTROL REGISTER
#define TXB0SIDH	0x31		// TRANSMIT BUFFER 0 STANDARD IDENTIFIER HIGH
#define TXB0SIDL	0x32		// TRANSMIT BUFFER 0 STANDARD IDENTIFIER LOW
#define TXB0EID8	0x33		// TRANSMIT BUFFER 0 EXTENDED IDENTIFIER HIGH
#define TXB0EID0	0x34		// TRANSMIT BUFFER 0 EXTENDED IDENTIFIER LOW
#define TXB0DLC		0x35		// TRANSMIT BUFFER 0 DATA LENGTH CODE
#define TXB0D0		0x36		// TRANSMIT BUFFER 0 DATA BYTE 0
#define TXB0D1		0x37		// TRANSMIT BUFFER 0 DATA BYTE 1
#define TXB0D2		0x38		// TRANSMIT BUFFER 0 DATA BYTE 2
#define TXB0D3		0x39		// TRANSMIT BUFFER 0 DATA BYTE 3
#define TXB0D4		0x3A		// TRANSMIT BUFFER 0 DATA BYTE 4
#define TXB0D5		0x3B		// TRANSMIT BUFFER 0 DATA BYTE 5
#define TXB0D6		0x3C		// TRANSMIT BUFFER 0 DATA BYTE 6
#define TXB0D7		0x3D		// TRANSMIT BUFFER 0 DATA BYTE 7

#define TXB1CTRL	0x40		// TRANSMIT BUFFER 1 CONTROL REGISTER
#define TXB1SIDH	0x41		// TRANSMIT BUFFER 1 STANDARD IDENTIFIER HIGH
#define TXB1SIDL	0x42		// TRANSMIT BUFFER 1 STANDARD IDENTIFIER LOW
#define TXB1EID8	0x43		// TRANSMIT BUFFER 1 EXTENDED IDENTIFIER HIGH
#define TXB1EID0	0x44		// TRANSMIT BUFFER 1 EXTENDED IDENTIFIER LOW
#define TXB1DLC		0x45		// TRANSMIT BUFFER 1 DATA LENGTH CODE
#define TXB1D0		0x46		// TRANSMIT BUFFER 1 DATA BYTE 0
#define TXB1D1		0x47		// TRANSMIT BUFFER 1 DATA BYTE 1
#define TXB1D2		0x48		// TRANSMIT BUFFER 1 DATA BYTE 2
#define TXB1D3		0x49		// TRANSMIT BUFFER 1 DATA BYTE 3
#define TXB1D4		0x4A		// TRANSMIT BUFFER 1 DATA BYTE 4
#define TXB1D5		0x4B		// TRANSMIT BUFFER 1 DATA BYTE 5
#define TXB1D6		0x4C		// TRANSMIT BUFFER 1 DATA BYTE 6
#define TXB1D7		0x4D		// TRANSMIT BUFFER 1 DATA BYTE 7

#define TXB2CTRL	0x50		// TRANSMIT BUFFER 2 CONTROL REGISTER
#define TXB2SIDH	0x51		// TRANSMIT BUFFER 2 STANDARD IDENTIFIER HIGH
#define TXB2SIDL	0x52		// TRANSMIT BUFFER 2 STANDARD IDENTIFIER LOW
#define TXB2EID8	0x53		// TRANSMIT BUFFER 2 EXTENDED IDENTIFIER HIGH
#define TXB2EID0	0x54		// TRANSMIT BUFFER 2 EXTENDED IDENTIFIER LOW
#define TXB2DLC		0x55		// TRANSMIT BUFFER 2 DATA LENGTH CODE
#define TXB2D0		0x56		// TRANSMIT BUFFER 2 DATA BYTE 0
#define TXB2D1		0x57		// TRANSMIT BUFFER 2 DATA BYTE 1
#define TXB2D2		0x58		// TRANSMIT BUFFER 2 DATA BYTE 2
#define TXB2D3		0x59		// TRANSMIT BUFFER 2 DATA BYTE 3
#define TXB2D4		0x5A		// TRANSMIT BUFFER 2 DATA BYTE 4
#define TXB2D5		0x5B		// TRANSMIT BUFFER 2 DATA BYTE 5
#define TXB2D6		0x5C		// TRANSMIT BUFFER 2 DATA BYTE 6
#define TXB2D7		0x5D		// TRANSMIT BUFFER 2 DATA BYTE 7

#define RXB0CTRL	0x60		// RECEIVE BUFFER 0 CONTROL
#define RXB0SIDH	0x61		// RECEIVE BUFFER 0 STANDARD IDENTIFIER HIGH
#define RXB0SIDL	0x62		// RECEIVE BUFFER 0 STANDARD IDENTIFIER LOW
#define RXB0EID8	0x63		// RECEIVE BUFFER 0 EXTENDED IDENTIFIER HIGH
#define RXB0EID0	0x64		// RECEIVE BUFFER 0 EXTENDED IDENTIFIER LOW
#define RXB0DLC		0x65		// RECEIVE BUFFER 0 STANDARD IDENTIFIER LOW
#define RXB0D0		0x66		// RECEIVE BUFFER 0 DATA BYTE 0
#define RXB0D1		0x67		// RECEIVE BUFFER 0 DATA BYTE 1
#define RXB0D2		0x68		// RECEIVE BUFFER 0 DATA BYTE 2
#define RXB0D3		0x69		// RECEIVE BUFFER 0 DATA BYTE 3
#define RXB0D4		0x6A		// RECEIVE BUFFER 0 DATA BYTE 4
#define RXB0D5		0x6B		// RECEIVE BUFFER 0 DATA BYTE 5
#define RXB0D6		0x6C		// RECEIVE BUFFER 0 DATA BYTE 6
#define RXB0D7		0x6D		// RECEIVE BUFFER 0 DATA BYTE 7

#define RXB1CTRL	0x70		// RECEIVE BUFFER 1 CONTROL
#define RXB1SIDH	0x71		// RECEIVE BUFFER 1 STANDARD IDENTIFIER HIGH
#define RXB1SIDL	0x72		// RECEIVE BUFFER 1 STANDARD IDENTIFIER LOW
#define RXB1EID8	0x73		// RECEIVE BUFFER 1 EXTENDED IDENTIFIER HIGH
#define RXB1EID0	0x74		// RECEIVE BUFFER 1 EXTENDED IDENTIFIER LOW
#define RXB1DLC		0x75		// RECEIVE BUFFER 1 STANDARD IDENTIFIER LOW
#define RXB1D0		0x76		// RECEIVE BUFFER 1 DATA BYTE 0
#define RXB1D1		0x77		// RECEIVE BUFFER 1 DATA BYTE 1
#define RXB1D2		0x78		// RECEIVE BUFFER 1 DATA BYTE 2
#define RXB1D3		0x79		// RECEIVE BUFFER 1 DATA BYTE 3
#define RXB1D4		0x7A		// RECEIVE BUFFER 1 DATA BYTE 4
#define RXB1D5		0x7B		// RECEIVE BUFFER 1 DATA BYTE 5
#define RXB1D6		0x7C		// RECEIVE BUFFER 1 DATA BYTE 6
#define RXB1D7		0x7D		// RECEIVE BUFFER 1 DATA BYTE 7

/* Bit Positions of the Registers of the MCP2515 */

/* Bit Positions of TXRTSCTRL */
#define B2RTS		5		// TX2RTS Pin State bit
#define B1RTS		4		// TX1RTX Pin State bit
#define B0RTS		3		// TX0RTS Pin State bit
#define B2RTSM		2		// TX2RTS Pin mode bit
#define B1RTSM		1		// TX1RTS Pin mode bit
#define B0RTSM		0		// TX0RTS Pin mode bit

/* Bit Positions of CANSTAT */
#define OPMOD2		7		// Operation mode bits
#define OPMOD1		6
#define OPMOD0		5
#define ICOD2		3		// Interrupt Flag Code bits
#define ICOD1		2
#define ICOD0		1

/* Bit Positions of CANCTRL */
#define REQOP2		7		// Request Operation mode bits
#define REQOP1		6		// Request Operation mode bits
#define REQOP0		5		// Request Operation mode bits
#define CLKPRE1		1		// CLKOUT Pin Prescaler bits
#define CLKPRE0		0		// CLKOUT Pin Prescaler bits


/* Bit Positions of CNF3 */
#define SOF			7		// Start-of-Frame signal bit
#define PHSEG22		2		// PS2 Length bits
#define PHSEG21		1
#define PHSEG20		0

/* Bit Positions of CNF2 */
#define PHSEG12		5		// PS1 Length bits
#define PHSEG11		4
#define PHSEG10		3
#define PRSEG2		2		// Propagation Segment Length bits
#define PRSEG1		1
#define PRSEG0		0

/* Bit Positions of CNF1 */
#define SJW1		7		// Synchronization Jump Width Length bits
#define SJW0		6
#define BRP5		5		// Baud Rate Prescaler bits
#define BRP4		4
#define BRP3		3
#define BRP2		2
#define BRP1		1
#define BRP0		0

/* Bit Positions of EFLG */
#define RX1OVR		7		// Receive Buffer 1 Overflow Flag bit
#define RX0OVR		6		// Receive Buffer 0 Overflow Flag bit
#define TXB0		5		// Bus-Off Error Flag bit
#define TXEP		4		// Transmit Error-Passive Flag bit
#define RXEP		3		// Receive Error-Passive Flag bit
#define TXWAR		2		// Transmit Error Warning Flag bit
#define RXWAR		1		// Receive Error Warning Flag bit
#define EWARN		0		// Error Warning Flag bit

/* Bit Positions of TXBnCTRL (n = 0, 1, 2) */
#define ABTF		6		// Message Aborted Flag bit
#define MLOA		5		// Message Lost Arbitration bit
							// TXP<1:0>: Transmit Buffer Priority bits
#define TXP1		1		// 11 = Highest Message Priority ,10 = High Intermediate Message Priority
#define TXP0		0		// 01 = Low Intermediate Message Priority, 00 = Lowest Message Priority

/* Bit Positions of RXB0CTRL */
#define RXM1		6		// Receive Buffer Operating mode bits
#define RXM0		5		// Receive Buffer Operating mode bits
#define RXRTR		3		// Received Remote Transfer Request bit
#define BUKT1		1		// Read-only Copy of BUKT bit (used internally by the MCP2515)
#define FILHIT0		0

/* Bit Positions of TXBnSIDL (n = 0, 1) */
#define	EXIDE		3		// Extended Identifier Enable bit

/* Bit Positions of RXB1CTRL, RXM1, RXM0, RXRTR and FILHIT0 already defined for RXB0CTRL */
#define FILHIT2		2		// Filter Hit bits - indicates which acceptance filter enabled reception of message
#define FILHIT1		1		// Filter Hit bits - indicates which acceptance filter enabled reception of message

/* Bit Positions of RXBnSIDL (n = 0, 1) */
#define	SRR			4		// Standard Frame Remote Transmit Request bit (valid only if IDE bit = ‘0’)
#define	IDE			3		// Extended Identifier Flag bit

/* Bit Positions of RXBnDLC (n = 0, 1) TXBnDLC */
#define	RTR			6		// Extended Frame Remote Transmission Request bit
#define	DLC3		3		// Data Length Code bits
#define	DLC2		2		// Data Length Code bits
#define	DLC1		1		// Data Length Code bits
#define DLC0		0		// Data Length Code bits

/* Bit Masks for Register Settings */

/* RXBnCTRL */
#define RXM_RCV_ALL     0x60
#define RXM_VALID_EXT   0x40
#define RXM_VALID_STD   0x20
#define RXM_VALID_ALL   0x00

/* RXBnCTRL */
#define RXM             0xC0	// Receive Buffer Operating mode bits // Turn mask/filters off; receive any message
#define BUKT            0x04	// Rollover Enable bit
#define FILHIT25		0x07	// Filter Hit Bits fpr RXF2-RXF5 in RXB1CTRL

/* CANINTE */
#define RX0IE           0x01	// Receive Buffer 0 Full Interrupt Enable bit
#define RX1IE           0x02	// Receive Buffer 1 Full Interrupt Enable bit
#define TX0IE           0x04	// Transmit Buffer 0 Empty Interrupt Enable bit
#define TX1IE           0x80	// Transmit Buffer 1 Empty Interrupt Flag bit
#define TX2IE           0x10	// Transmit Buffer 2 Empty Interrupt Flag bit
#define ERRIE           0x20	// Error Interrupt Enable bit
#define WAKIE           0x40	// Wake-up Interrupt Enable bit
#define MERRE           0x80	// Message Error Interrupt Enable bit

/* CANSTAT */
#define OPMODE_CONFIG   0x80
#define OPMODE_LISTEN   0x60
#define OPMODE_LOOPBACK 0x40
#define OPMODE_SLEEP    0x20
#define OPMODE_NORMAL   0x00

/* CANCTRL */
#define REQOP           0xE0	// Request Operation mode bits
#define ABAT            0x10	// Abort All Pending Transmissions bit
#define	OSM         	0x08	// One-Shot mode bit
#define CLKEN           0x04	// CLKOUT Pin Enable bit
#define CLKPRE          0x03	// CLKOUT Pin Prescaler bits

#define SOF_ENABLED     0x80
#define SOF_DISABLED    0x00
#define WAKFIL_ENABLED  0x40
#define WAKFIL_DISABLED 0x00

#define BTLMODE_CNF3    0x80
#define SMPL_3X         0x40
#define SMPL_1X         0x00

#define OSM_ENABLED     0x08
#define EXIDE_SET       0x08
#define EXIDE_RESET     0x00

#endif  /* __MCP2515_H__ */
