
///////////////////////////////////////////////////////////////////////////////
//! \file SP.h
//! \brief Header file for the Satellite Processor module
//!
//! This file provides all of the defines and function prototypes for the
//! \ref SP Module.
//!
//! @addtogroup SP
//! @{
//!
//! @addtogroup comm Software UART
//! The software UART module allows the user to define a UART interface on
//! any two GPIO pins, provided that the RX pin is interrupt capable. The
//! module requires the use of TimerA.
//! @{
///////////////////////////////////////////////////////////////////////////////


#ifndef SP_H_
#define SP_H_

#include "../std.h"
#include "../comm.h"

//! \def NUMBER_SPBOARDS
//! \brief This is the number of SP boards allowed on the CP
#define NUMBER_SPBOARDS			4

//! \def NUMBER_TRANSDUCERS
//! \brief This is the number of transducers on a SP
#define NUMBER_TRANSDUCERS	4

//! \def TYPE_IS_SENSOR
//! \brief The transducer type definition for a sensor
#define TYPE_IS_SENSOR			0x53 //ascii S
//! \def TYPE_IS_ACTUATUATOR
//! \brief The transducer type definition for an actuator
#define TYPE_IS_ACTUATUATOR	0x41 //ascii A

//! \def SP_STATE_ACTIVE
//! \brief SP state active
#define SP_STATE_ACTIVE			0xAC00

//! \def SP_INACTIVE
//! \brief SP state inactive
#define SP_STATE_INACTIVE		0x1A00

//! \def SP_NAME_LENGTH
//! \brief The number of characters allotted to the name of the SP board
#define SP_NAME_LENGTH			8

//! \def YES_PRINT
//! \brief Allows printing of SP information
#define YES_PRINT						1

//! \def NO_PRINT
//! \brief Prevents printing of SP information
#define NO_PRINT						0

//! \def NUM_OF_LABELS
//! \brief This is the number of SP board labels
#define NUM_OF_LABELS				0x12

//! \def LABEL_LENGTH
//! \brief This is the length of an SP label
#define LABEL_LENGTH				0x10

//! \struct
//! \brief The structure holds the bit values and registers
//! associated with the active satellite processor
struct S_Active_SP_Fields
{
	volatile uint8 m_ucActiveSP;
	volatile uint8 m_ucActiveSP_BitTx;
	volatile uint8 m_ucActiveSP_BitRx;
	volatile uint8 m_ucActiveSP_BitRst;
	volatile uint8 m_ucActiveSP_BitTck;
	volatile uint8 m_ucActiveSP_BitEn;
	volatile uint8 m_ucActiveSP_BitInt;
	volatile uint8 * m_ucActiveSP_RegDir;
	volatile uint8 * m_ucActiveSP_RegOut;
};

//! \struct
//! \brief The structure holds information regarding the attached SPs (active or inactive)
struct S_Attached_SP_Fields
{
	volatile uint8 m_ucAttachedSPs;																															//!< Bit field for quickly determining if an SP is present
	volatile uint8 m_ucaSP_MSG_Version[NUMBER_SPBOARDS];																				//!< Message versions of the attached SPs
	volatile uint16 m_uiaSP_Status[NUMBER_SPBOARDS];																							//!< Status of the SP
	volatile int8 m_ucaSP_Name[NUMBER_SPBOARDS][SP_NAME_LENGTH];																//!< ASCII label of the attached SP boards //set to 8
	volatile uint32 m_ulaSP_SerialNumber[NUMBER_SPBOARDS];																			//!< Unique serial number of the SP
	volatile uint8 m_ucaSP_NumTransducers[NUMBER_SPBOARDS];																			//!< Number of transducers on the SP
	volatile uint8 m_ucaSP_TypeTransducers[NUMBER_SPBOARDS][NUMBER_TRANSDUCERS];								//!< Type of transducers on the SP
	volatile int8 m_cpaSPLabels[NUMBER_SPBOARDS][NUM_OF_LABELS][LABEL_LENGTH]; 									//!< The transducer labels in ASCII
};

//! \typedef enum eSP_DriverState
//! \Brief Indicates the state of the SP driver
typedef enum eSP_DriverState
{
 SP_DRIVER_SHUTDOWN,
 SP_DRIVER_ACTIVE
} SP_DriverState_t;

//***************** SP Pin defines *************************************//
//! @name SP_Pin_Definitions
//! These define every pin of all SP boards on the CP
//! @{
//! \def P_RX_DIR
//! \brief The PxDIR register of the RX from SP pin
#define P_RX_DIR        P2DIR
//! \def P_RX_IN
//! \brief The PxIN register of the RX from SP pin
#define P_RX_IN         P2IN
//! \def P_RX_IES
//! \brief The PxIES register of the RX from SP pin
#define P_RX_IES        P2IES
//! \def P_RX_IFG
//! \brief The PxIFG register of the RX from SP pin
#define P_RX_IFG        P2IFG
//! \def P_RX_IE
//! \brief The PxIE register of the RX from SP pin
#define P_RX_IE         P2IE

//! \def P_INT_DIR
//! \brief The PxDIR register of the INT to SP pin
#define P_INT_DIR       P1DIR
//! \def P_INT_OUT
//! \brief The PxOUT register that the INT to SP pin is on
#define P_INT_OUT       P1OUT

//! \def SP1
//! \brief SP board in location 1
#define SP1				0
//! \def SP1_ProcessorID
//! \brief Processor ID for SP board in location 1
#define SP1_ProcessorID	1
//! \def SP1_BIT
//! \brief SP1 bit mask
#define SP1_BIT			BIT0
//! \def P_SP1_DIR
//! \brief PxDIR of SP board in location 1
#define P_SP1_DIR    	P4DIR
//! \def P_SP1_SEL
//! \brief PxSEL of the SP board in location 1
#define P_SP1_SEL    	P4SEL
//! \def P_SP1_OUT
//! \brief PxOUT SP board in location 1
#define P_SP1_OUT    	P4OUT
//! \def SP1_EN_BIT
//! \brief The enable pin of SP board in location 1
#define SP1_EN_BIT    	BIT0
//! \def SP1_INT_BIT
//! \brief The interrupt pin of SP board in location 1
#define SP1_INT_BIT    	BIT0
//! \def SP1_TX_BIT
//! \brief The transmit pin of SP board in location 1
#define SP1_TX_BIT    	BIT1
//! \def SP1_RST_BIT
//! \brief The reset pin of SP board in location 1
#define SP1_RST_BIT    	BIT2
//! \def SP1_TCK_BIT
//! \brief The test clock pin of SP board in location 1
#define SP1_TCK_BIT    	BIT3
//! \def SP1_RX_BIT
//! \brief The receive pin of SP board in location 1
#define SP1_RX_BIT    	BIT0
//! \def SP1_IV_NUMBER
//! \brief The interrupt vector number of the RX pin of SP board in location 1
#define SP1_IV_NUMBER   P2IV_P2IFG0

//! \def SP2
//! \brief SP board in location 2
#define SP2				1
//! \def SP2_ProcessorID
//! \brief Processor ID for SP board in location 2
#define SP2_ProcessorID	2
//! \def SP2_BIT
//! \brief SP2 bit mask
#define SP2_BIT			BIT1
//! \def P_SP2_DIR
//! \brief PxDIR of SP board in location 2
#define P_SP2_DIR	    P4DIR
//! \def P_SP2_SEL
//! \brief PxSEL of the SP board in location 2
#define P_SP2_SEL	    P4SEL
//! \def P_SP2_OUT
//! \brief PxOUT SP board in location 2
#define P_SP2_OUT	    P4OUT
//! \def SP2_EN_BIT
//! \brief The enable pin of SP board in location 2
#define SP2_EN_BIT    	BIT4
//! \def SP2_INT_BIT
//! \brief The interrupt pin of SP board in location 2
#define SP2_INT_BIT    	BIT1
//! \def SP2_TX_BIT
//! \brief The transmit pin of SP board in location 2
#define SP2_TX_BIT    	BIT5
//! \def SP2_RST_BIT
//! \brief The reset pin of SP board in location 2
#define SP2_RST_BIT    	BIT6
//! \def SP2_TCK_BIT
//! \brief The test clock pin of SP board in location 2
#define SP2_TCK_BIT    	BIT7
//! \def SP2_RX_BIT
//! \brief The receive pin of SP board in location 2
#define SP2_RX_BIT    	BIT1
//! \def SP2_IV_NUMBER
//! \brief The interrupt vector number of the RX pin of SP board in location 2
#define SP2_IV_NUMBER   P2IV_P2IFG1

//! \def SP3
//! \brief SP board in location 3
#define SP3				2
//! \def SP3_ProcessorID
//! \brief Processor ID for SP board in location 3
#define SP3_ProcessorID	3
//! \def SP3_BIT
//! \brief SP3 bit mask
#define SP3_BIT			BIT2
//! \def P_SP3_DIR
//! \brief PxDIR of SP board in location 3
#define P_SP3_DIR	    P5DIR
//! \def P_SP3_SEL
//! \brief PxDIR of SP board in location 3
#define P_SP3_SEL	    P5SEL
//! \def P_SP3_OUT
//! \brief PxOUT SP board in location 3
#define P_SP3_OUT 	   	P5OUT
//! \def SP3_EN_BIT
//! \brief The enable pin of SP board in location 3
#define SP3_EN_BIT    	BIT0
//! \def SP3_INT_BIT
//! \brief The interrupt pin of SP board in location 3
#define SP3_INT_BIT    	BIT2
//! \def SP3_TX_BIT
//! \brief The reset pin of SP board in location 3
#define SP3_TX_BIT    	BIT1
//! \def SP3_RST_BIT
//! \brief The reset pin of SP board in location 3
#define SP3_RST_BIT    	BIT2
//! \def SP3_TCK_BIT
//! \brief The test clock pin of SP board in location 3
#define SP3_TCK_BIT    	BIT3
//! \def SP3_RX_BIT
//! \brief The receive pin of SP board in location 3
#define SP3_RX_BIT    	BIT2
//! \def SP3_IV_NUMBER
//! \brief The interrupt vector number of the RX pin of SP board in location 3
#define SP3_IV_NUMBER   P2IV_P2IFG2

//! \def SP3
//! \brief SP board in location 4
#define SP4				3
//! \def SP4_ProcessorID
//! \brief Processor ID for SP board in location 4
#define SP4_ProcessorID	4
//! \def SP4_BIT
//! \brief SP4 bit mask
#define SP4_BIT			BIT3
//! \def SP4_DIR
//! \brief PxDIR of SP board in location 4
#define P_SP4_DIR	    	P7DIR
//! \def SP4_SEL
//! \brief PxSEL of SP board in location 4
#define P_SP4_SEL	    	P7SEL
//! \def SP4_OUT
//! \brief PxOUT SP board in location 4
#define P_SP4_OUT	    	P7OUT
//! \def SP4_EN_BIT
//! \brief The enable pin of SP board in location 4
#define SP4_EN_BIT    	BIT2
//! \def SP4_INT_BIT
//! \brief The interrupt pin of SP board in location 4
#define SP4_INT_BIT    	BIT3
//! \def SP4_TX_BIT
//! \brief The reset pin of SP board in location 4
#define SP4_TX_BIT    	BIT3
//! \def SP4_RST_BIT
//! \brief The reset pin of SP board in location 4
#define SP4_RST_BIT    	BIT4
//! \def SP4_TCK_BIT
//! \brief The test clock pin of SP board in location 4
#define SP4_TCK_BIT    	BIT5
//! \def SP4_RX_BIT
//! \briefThe receive pin of SP board in location 4
#define SP4_RX_BIT    	BIT3
//! \def SP4_IV_NUMBER
//! \brief The interrupt vector number of the RX pin of SP board in location 4
#define SP4_IV_NUMBER   P2IV_P2IFG3
//! @}

//! \def SP_DIRReg
//! \brief This value is used as an index in the SP control register array
#define SP_DIRReg	0

//! \def SP_OUTReg
//! \brief This value is used as an index in the SP control register array
#define SP_OUTReg 1

//! \def RX_BUFFER_SIZE
//! \brief The number of bytes to allocate for the UART RX buffer
#define RX_BUFFER_SIZE 0x20

// Status Flags
//! \name Status Flags
//! These are bit defines that are used to set and clear the
//! g_ucCOMM_Flags register.
//! @{
//! \def COMM_RUNNING
//! \brief Bit define - Indicates UART is running
#define COMM_RUNNING 	BIT0
//! \def COMM_TX_BUSY
//! \brief Bit define - Indicates TX in progress
#define COMM_TX_BUSY 	BIT1
//! \def COMM_RX_BUSY
//! \brief Bit define - Indicates RX in progress
#define COMM_RX_BUSY 	BIT2
//! \def COMM_SP1_BUSY
//! \brief Bit define - Indicates comm with SP in location 1
#define COMM_SP1_BUSY	BIT3
//! \def COMM_SP2_BUSY
//! \brief Bit define - Indicates comm with SP in location 2
#define COMM_SP2_BUSY	BIT4
//! \def COMM_SP3_BUSY
//! \brief Bit define - Indicates comm with SP in location 3
#define COMM_SP3_BUSY	BIT5
//! \def COMM_SP4_BUSY
//! \brief Bit define - Indicates comm with SP in location 4
#define COMM_SP4_BUSY	BIT6
//! @}

//******************  Baud Rate Delays  *************************************//
//! @name Baud Rate Start Delays
//! These values are used to delay from the start bit to the middle of the
//! first data bit. Computed for a 4MHz SMCLK.
//! @{
//! \def BAUD_460800_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_460800_DELAY  0x0004
//! \def BAUD_345600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_345600_DELAY  0x0006
//! \def BAUD_230400_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_230400_DELAY  0x0008
//! \def BAUD_115200_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_115200_DELAY  BAUD_115200 + BAUD_115200/2 - 37//0x0010//Slightly low to account for the set-up cycles that we delayed. At these times that's necessary...
//! \def BAUD_57600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_57600_DELAY   BAUD_57600 + BAUD_57600/2 - 36
//! \def BAUD_19200_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_19200_DELAY   BAUD_19200 + BAUD_19200/2 - 36
//! \def BAUD_9600_DELAY
//! \brief Timer count for specific data rate delay, computed for 4Mhz SMCLK.
#define BAUD_9600_DELAY    BAUD_9600 + BAUD_9600/2 - 36
//! @}


//*****************  Baud rate defines *************************************//
//! @name Baud Rate Defines
//! These values are computed for a 4MHz SMCLK to produce the corresponding
//! baud rates
//! @{
//! \def BAUD_460800
//! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_460800 0x0008
//! \def BAUD_345600
//! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_345600 0x000B
//! \def BAUD_230400
//! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_230400 0x0011
//! \def BAUD_115200
//! \brief Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_115200 0x0023
//! \def BAUD_57600
//! Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_57600  0x0045
//! \def BAUD_19200
//! Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_19200  0x00D0
//! \def BAUD_9600
//! Timer count for specific UART data rate, computed for 4Mhz SMCLK
#define BAUD_9600   0x01A0
//! \def SP_OPMODE_BAUD
//! The preferred baud rate for SP comm in operational mode
#define SP_OPMODE_BAUD			BAUD_19200
//! \def SP_RPRGM_BAUD
//! The preferred baud rate for SP comm in reprogram mode
#define SP_RPGRM_BAUD				BAUD_9600

//! @}




// Return codes
//! \name Return Codes
//! Possible return codes from the \ref SP functions
//! @{
//! \def COMM_BUFFER_UNDERFLOW
//! \brief Function return code
//!
//! Getting this return code means that the user has tried to pull more
//! data that is available from g_ucaRXBuffer
#define COMM_ERROR 							0x01
//! \def COMM_OK
//! \brief Function return code
#define COMM_OK               	0x00
//! \def DRIVER_NOT_INITIALIZED
//! \brief Function return code when driver isn't initialized
#define SP_BAD_STATE						0x02
//! \def SP_NOT_ATTACHED
//! \brief Indicates that the desired SP location is empty
#define SP_NOT_ATTACHED					0x04
//! \def COMM_PARITY_ERR
//! \brief Indicates that a received byte fails the parity check
#define COMM_PARITY_ERR					0x08
//! @}

// Comm.c function prototypes
//! @name Control Functions
//! These functions handle controlling the \ref SP Module.
//! @{
uint8 ucSP_Init(void);
uint8 ucSP_SetState(uint16 ucBaud, uint8 ucSPnumber);
void  vSP_ShutdownComm(void);
uint8 ucSP_WaitForMessage(void);
uint8  ucSP_WaitForLabelMessage(void);
//! @}

//! @name Transmit Functions
//! These functions transmit information on the \ref SP Module.
//! @{
static void vSP_SendByte(uint8 ucChar);
uint8 ucSP_Send32BitDataMessage(union SP_DataMessage * p_DataMessage, uint8 ucSPNumber);
uint8 ucSP_Send128BitDataMessage(union SP_DataMessage * p_128BitDataMessage, uint8 ucSPnumber);
uint8 ucSP_SendLabelMessage(union SP_LabelMessage * p_LabelMessage, uint8 ucSPnumber);
//! @}

//! @name Receive Functions
//! These functions take information from the \ref SP Module and format
//! it appropriately.
//! @{
void vSP_Clean32BitBuffer(union SP_DataMessage * message);
void vSP_Clean128BitBuffer(union SP_DataMessage * message);
uint8 ucSP_Grab32BitDataMessageFromBuffer(union SP_DataMessage * message);
uint8 ucSP_Grab128BitDataMessageFromBuffer(union SP_DataMessage * message);
uint8 ucSP_GrabLabelMessageFromBuffer(union SP_LabelMessage * message);
//! @}

// SP.c function prototypes
//! @name Control Functions
//! These functions handle controlling the SP boards
//! @{
void vSP_TurnoffAll(void);
void vSP_TurnOff(uint8 ucSPNumber);
//! @}

//! @name Start up functions
//! These functions handle the initialization of the SP boards
//! @{
void vSP_Init(void);
uint8 ucSP_Start(uint8 ucSPNumber, uint8 ucPrint);
void vSP_SetRole(void);
void vSP_GetSPMsgVersions(void);
//! @}

//! @name Runtime Functions
//! These functions handle the runtime SP operations
//! @{
uint8 vSP_RequestData(uint8 ucSPnumber);
uint8 vSP_Request128BitsData(uint8 ucSPnumber);
uint8 vSP_Request32BitsData(uint8 ucSPnumber);
void vSP_RequestSP_TransducerLabels(uint8 ucSPNumber);
void vSP_RequestSingle_TransducerLabel(uint8 ucSPnumber, uint8 ucLabelIndex);
void vSP_RequestAll_TransducerLabels(void);
uint8 ucSP_SendCommand(uint8 ucSPnumber, uint8 ucTransducer, uint8 ucByte1High, uint8 ucByte1Low, uint8 ucByte2High, uint8 ucByte2Low);
void vSP_DisplaySingleLabel(uint8 ucSPNumber, uint8 ucTransducer);
void vSP_DisplayLabels(uint8 ucSPNumber);
void vSP_Display(void);
uint8 ucSP_SendInterrogate(uint8 ucSPnumber);
uchar ucSP_FetchNumTransducers(uchar ucSP_Number);
uchar ucSP_FetchTransType(uchar ucSPNumber, uchar ucTransNumber);
uint16 uiSP_FetchSPState(uchar ucSPNumber);
void vSP_FetchTransName(uchar ucSPNumber, uchar ucTransNum, char *p_caName);
uint8 ucSP_IsAttached(uchar ucSPNumber);
//! @}


#endif /* SP_H_ */
