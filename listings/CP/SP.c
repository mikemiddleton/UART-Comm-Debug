#include <msp430x54x.h>
#include "SP.h"
#include "../comm.h"
#include "../hal/config.h"
#include "../std.h"
#include "../serial.h"

//! \struct
//! \brief The structure holds the bit values and registers
//! associated with the active satellite processor
struct S_Active_SP_Fields S_ActiveSP;

//! \struct
//! \brief The structure holds the bit values and registers
//! associated with the active satellite processor
struct S_Attached_SP_Fields S_AttachedSP;

//******************  Control and Indication Variables  *********************//
//! @name Control and Indication Variables
//! These variables are used to indicate to the system the current status
//! of the \ref SP Module and to store the baud rate timer information.
//! @{
//! \var volatile uint8 g_ucCOMM_Flags
//! \brief This 8-bit field indicates the status of the COMM module.
volatile uint8 g_ucCOMM_Flags;

//! \var static volatile uint8 ucaActiveSPFlag[]
//! \brief This array stores the Active SP board
static volatile uint8 ucaSPBit[NUMBER_SPBOARDS] =
{ SP1_BIT, SP2_BIT, SP3_BIT, SP4_BIT };

//! \var static const uint8 g_ucaSPRstPins[]
//! \brief This array stores the SP boards Reset pins
static const uint8 g_ucaSPRstPins[NUMBER_SPBOARDS] =
{ SP1_RST_BIT, SP2_RST_BIT, SP3_RST_BIT, SP4_RST_BIT };

//! \var static const uint8 g_ucaSPTckPins[]
//! \brief This array stores the SP boards test clock pins
static const uint8 g_ucaSPTckPins[NUMBER_SPBOARDS] =
{ SP1_TCK_BIT, SP2_TCK_BIT, SP3_TCK_BIT, SP4_TCK_BIT };

//! \var static const uint8 g_ucaSPIntPins[]
//! \brief This array stores the SP boards Interrupt pins
static const uint8 g_ucaSPIntPins[NUMBER_SPBOARDS] =
{ SP1_INT_BIT, SP2_INT_BIT, SP3_INT_BIT, SP4_INT_BIT };

//! \var static const uint8 g_ucaSPEnPins[]
//! \brief This array stores the SP boards enable pins
static const uint8 g_ucaSPEnPins[NUMBER_SPBOARDS] =
{ SP1_EN_BIT, SP2_EN_BIT, SP3_EN_BIT, SP4_EN_BIT };

//! \enum volatile SP_DriverState_t g_eSP_DriverState
//! \brief This enum indicates the state of the SP driver
//!
//! This tells the system if the hardware and other common variables
//! have been set properly before beginning calls to the SPs.
volatile SP_DriverState_t g_eSP_DriverState = SP_DRIVER_SHUTDOWN;

//! \var uint16 g_unCOMM_BaudRateControl
//! \brief This is the value used to control the baud rate.
//!
//! This value is the number of timer ticks corresponding to one bit period
//! for the baud rate. It should be set from the \ref comm_baud
//! "Baud Rate Defines".
uint16 g_unCOMM_BaudRateControl;

//! \var uint16 g_unCOMM_BaudRateDelayControl
//! \brief This is the value used to delay from the start bit
//!
//! This value is the number of timer ticks to wait from the beginning of the
//! start bit to the middle of the first data bit. It should be set from the
//! \ref SP_baud_delay "Baud Rate Start Delays".
uint16 g_unCOMM_BaudRateDelayControl;
//! @}

//******************  RX Variables  *****************************************//
//! @name Receive Variables
//! These variables are used in the receiving of data on the \ref SP Module.
//! @{
//! \var volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE]
//! \brief The software UART RX Buffer
volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE];

//! \var volatile uint8 g_ucRXBufferIndex
//! \brief This index into g_ucaRXBuffer showing the current write position.
volatile uint8 g_ucRXBufferIndex;

//! \var volatile uint8 g_ucRXMessageSize
//! \brief This holds the size of the message received
volatile uint8 g_ucRXMessageSize;

//! \var static const uint8 g_ucaSPRxPins[]
//! \brief This array stores the SP boards Rx pins
static const uint8 g_ucaSPRxPins[NUMBER_SPBOARDS] =
{ SP1_RX_BIT, SP2_RX_BIT, SP3_RX_BIT, SP4_RX_BIT };
//! @}


//! \var static const uint8 g_ucaSPTxPins[]
//! \brief This array stores the SP boards Tx pins
static const uint8 g_ucaSPTxPins[NUMBER_SPBOARDS] =
{ SP1_TX_BIT, SP2_TX_BIT, SP3_TX_BIT, SP4_TX_BIT };

//******************  Register Variables  *****************************************//
//! @name Register Variables
//! There variables are used for register configuration in the \ref SP
//! Module.
//! @{

//! \var static const uint8 g_ucaSPCtlReg[][]
//! \brief This array stores the control registers of the SP board
//!
//! The elements in this array are the addresses of the SP ports
//!
static volatile uint8 * g_pucSPCtlReg[NUMBER_SPBOARDS][2] =
{
{ &P_SP1_DIR, &P_SP1_OUT },
{ &P_SP2_DIR, &P_SP2_OUT },
{ &P_SP3_DIR, &P_SP3_OUT },
{ &P_SP4_DIR, &P_SP4_OUT } };

union SP_DataMessage g_SendDataMsg;
union SP_DataMessage g_RecDataMsg;

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns on the SP board in location 1
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP1_Turnon(void)
{
	P_SP1_OUT |= (SP1_EN_BIT | SP1_TX_BIT | SP1_RST_BIT);
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns off the SP board in location 1
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP1_Turnoff(void)
{
	P_SP1_OUT &= ~(SP1_EN_BIT | SP1_TX_BIT | SP1_RST_BIT);
	P_RX_IE &= ~SP1_INT_BIT;
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns on the SP board in location 2
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP2_Turnon(void)
{
	P_SP2_OUT |= (SP2_EN_BIT | SP2_TX_BIT | SP2_RST_BIT);
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns off the SP board in location 2
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP2_Turnoff(void)
{
	P_SP2_OUT &= ~(SP2_EN_BIT | SP2_TX_BIT | SP2_RST_BIT);
	P_RX_IE &= ~SP2_INT_BIT;
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns on the SP board in location 3
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP3_Turnon(void)
{
	P_SP3_OUT |= (SP3_EN_BIT | SP3_TX_BIT | SP3_RST_BIT);
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns off the SP board in location 3
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP3_Turnoff(void)
{
	P_SP3_OUT &= ~(SP3_EN_BIT | SP3_TX_BIT | SP3_RST_BIT);
	P_RX_IE &= ~SP3_INT_BIT;
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns on the SP board in location 4
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP4_Turnon(void)
{
	P_SP4_OUT |= (SP4_EN_BIT | SP4_TX_BIT | SP4_RST_BIT);
}

/////////////////////////////////////////////////////////////////////////////
//! \brief Turns off the SP board in location 4
//!
//!
//! @param none
//! @return none.
////////////////////////////////////////////////////////////////////////////
static inline void vSP4_Turnoff(void)
{
	P_SP4_OUT &= ~(SP4_EN_BIT | SP4_TX_BIT | SP4_RST_BIT);
	P_RX_IE &= ~SP4_INT_BIT;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Turns on an SP board
//!
//! This turns on the selected SP board; function only called within the module.
//!   \param ucSPNumber
//!   \return None
///////////////////////////////////////////////////////////////////////////////
static void vSP_TurnOn(uint8 ucSPNumber)
{

	switch (ucSPNumber)
	{
		case SP1:
			vSP1_Turnon();
		break;

		case SP2:
			vSP2_Turnon();
		break;

		case SP3:
			vSP3_Turnon();
		break;

		case SP4:
			vSP4_Turnon();
		break;

		default:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Turns off an SP board
//!
//! This turns off the selected SP board; function only called within the module.
//!   \param ucSPNumber
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_TurnOff(uint8 ucSPNumber)
{

	switch (ucSPNumber)
	{
		case SP1:
			vSP1_Turnoff();
		break;

		case SP2:
			vSP2_Turnoff();
		break;

		case SP3:
			vSP3_Turnoff();
		break;

		case SP4:
			vSP4_Turnoff();
		break;

		default:
		break;
	}
}


///////////////////////////////////////////////////////////////////////////////
//! \brief Removes an SP board from the attached SP structure
//!
///////////////////////////////////////////////////////////////////////////////
static void vSP_DetachBoard(uchar ucSP_Number)
{
	uchar ucLabelCounter, ucCounter;

	//Inform the driver that the SP is not attached
	S_AttachedSP.m_ucAttachedSPs &= ~ucaSPBit[ucSP_Number];

	// Set labels to 0
	for (ucLabelCounter = 0; ucLabelCounter < NUM_OF_LABELS; ucLabelCounter++)
		for (ucCounter = 0; ucCounter < LABEL_LENGTH; ucCounter++)
		{
			S_AttachedSP.m_cpaSPLabels[ucSP_Number][ucLabelCounter][ucCounter] = 0;
		}

	// Set the message version to 0
	S_AttachedSP.m_ucaSP_MSG_Version[ucSP_Number] = 0;

	//Set the name of the SP board into the SP name array to "EMTY"
	S_AttachedSP.m_ucaSP_Name[ucSP_Number][0x00] = 0x45;
	S_AttachedSP.m_ucaSP_Name[ucSP_Number][0x01] = 0x4D;
	S_AttachedSP.m_ucaSP_Name[ucSP_Number][0x02] = 0x54;
	S_AttachedSP.m_ucaSP_Name[ucSP_Number][0x03] = 0x59;

	// Clear remaining name field
	for (ucCounter = 4; ucCounter < SP_NAME_LENGTH; ucCounter++)
	{
		S_AttachedSP.m_ucaSP_Name[ucSP_Number][ucCounter] = 0;
	}

	S_AttachedSP.m_ucaSP_NumTransducers[ucSP_Number] = 0x00;
	S_AttachedSP.m_ulaSP_SerialNumber[ucSP_Number] = 0x00000000;

	for (ucCounter = 0; ucCounter < NUMBER_TRANSDUCERS; ucCounter++)
	{
		S_AttachedSP.m_ucaSP_TypeTransducers[ucSP_Number][ucCounter] = 0x00;
	}

	// Set the state of the board to inactive
	S_AttachedSP.m_uiaSP_Status[ucSP_Number] = SP_STATE_INACTIVE;

}//END vSP_DetachBoard

///////////////////////////////////////////////////////////////////////////////
//! \brief Sets pin configurations that change during runtime
//!
//!
//!   \param ucSPNumber
//!   \return 0,1 success or failure
///////////////////////////////////////////////////////////////////////////////
static uint8 ucSP_SetActiveBoard(uint8 ucSPNumber)
{

	uint8 ucSPBit;

	ucSPBit = ucaSPBit[ucSPNumber];

	if (S_AttachedSP.m_ucAttachedSPs & ucSPBit)
	{
		S_ActiveSP.m_ucActiveSP = ucaSPBit[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitTx = g_ucaSPTxPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitRx = g_ucaSPRxPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitRst = g_ucaSPRstPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitTck = g_ucaSPTckPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitEn = g_ucaSPEnPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_BitInt = g_ucaSPIntPins[ucSPNumber];
		S_ActiveSP.m_ucActiveSP_RegDir = g_pucSPCtlReg[ucSPNumber][SP_DIRReg];
		S_ActiveSP.m_ucActiveSP_RegOut = g_pucSPCtlReg[ucSPNumber][SP_OUTReg];
		return 0;
	}

	//the requested board is not attached
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//! \brief Shuts off all active SP boards
//!
//! This shuts off all the SP boards in use by checking which g_ucCOMM_Flags are set.
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_TurnoffAll(void)
{
	vSP1_Turnoff();
	vSP2_Turnoff();
	vSP3_Turnoff();
	vSP4_Turnoff();
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sets pin configurations that remain unchanged
//!
//!
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_Init(void)
{
	uchar ucSP_Number;

	// Don't allow this function to run if the driver is already running.
	if (g_eSP_DriverState != SP_DRIVER_SHUTDOWN)
	{
		return SP_BAD_STATE;
	}

	S_ActiveSP.m_ucActiveSP = 0xFF;
	S_ActiveSP.m_ucActiveSP_BitTx = NULL;
	S_ActiveSP.m_ucActiveSP_BitRx = NULL;
	S_ActiveSP.m_ucActiveSP_BitRst = NULL;
	S_ActiveSP.m_ucActiveSP_BitTck = NULL;
	S_ActiveSP.m_ucActiveSP_BitEn = NULL;
	S_ActiveSP.m_ucActiveSP_BitInt = NULL;
	S_ActiveSP.m_ucActiveSP_RegDir = NULL;
	S_ActiveSP.m_ucActiveSP_RegOut = NULL;

	//Initialize the attached SP structure
	S_AttachedSP.m_ucAttachedSPs = 0x00;

	for (ucSP_Number = 0; ucSP_Number < NUMBER_SPBOARDS; ucSP_Number++)
	{
		vSP_DetachBoard(ucSP_Number);
	}

	//initialize the communication flag
	g_ucCOMM_Flags = 0x00;

// Set the initial pin configurations
//SP1
	P_SP1_DIR |= (SP1_EN_BIT | SP1_RST_BIT | SP1_TCK_BIT); //set the enable, reset, and tck pins as outputs
	P_SP1_SEL &= ~(SP1_EN_BIT | SP1_RST_BIT | SP1_TCK_BIT); //set the enable, reset, and tck pins as GPIO
	P_SP1_OUT &= ~(SP1_EN_BIT | SP1_TX_BIT | SP1_RST_BIT | SP1_TCK_BIT); //set the enable, transmit, and tck pins low

	//SP2
	P_SP2_DIR |= (SP2_EN_BIT | SP2_RST_BIT | SP2_TCK_BIT); //set the enable, reset, and tck pins as outputs
	P_SP2_SEL &= ~(SP2_EN_BIT | SP2_RST_BIT | SP2_TCK_BIT); //set the enable, reset, and tck pins as GPIO
	P_SP2_OUT &= ~(SP2_EN_BIT | SP2_TX_BIT | SP2_RST_BIT | SP2_TCK_BIT); //set the enable, transmit, and tck pins low

	//SP3
	P_SP3_DIR |= (SP3_EN_BIT | SP3_RST_BIT | SP3_TCK_BIT); //set the enable, reset, and tck pins as outputs
	P_SP3_SEL &= ~(SP3_EN_BIT | SP3_RST_BIT | SP3_TCK_BIT); //set the enable, reset, and tck pins as GPIO
	P_SP3_OUT &= ~(SP3_EN_BIT | SP3_TX_BIT | SP3_RST_BIT | SP3_TCK_BIT); //set the enable, transmit, and tck pins low

	//SP4
	P_SP4_DIR |= (SP4_EN_BIT | SP4_RST_BIT | SP4_TCK_BIT); //set the enable, reset, and tck pins as outputs
	P_SP4_SEL &= ~(SP4_EN_BIT | SP4_RST_BIT | SP4_TCK_BIT); //set the enable, reset, and tck pins as GPIO
	P_SP4_OUT &= ~(SP4_EN_BIT | SP4_TX_BIT | SP4_RST_BIT | SP4_TCK_BIT); //set the enable, transmit, and tck pins low

	g_eSP_DriverState = SP_DRIVER_ACTIVE;

	return COMM_OK;
}


///////////////////////////////////////////////////////////////////////////////
//! \brief This prepares the driver for communication with a selected SP board
//!
//! Since we are doing UART without the USCI, we use TimerA and it's interrupt
//! to control the baud rate. The TX and RX pins are completely controllable
//! during run time. The software UART expects 1 start bit, 8 data bits, 1
//! parity bit and 1 stop bit.
//!
//! To ensure correct operation of the software UART, the \ref SP_Pin_Definitions
//! must be set correctly.
//!   \param ucBaud The baud rate define to use, ucSPnumber The SP board in use
//!   \return None
//!   \sa vCOMM_SendByte(), TIMERA0_ISR()
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_SetState(uint16 ucBaud, uint8 ucSPNumber)
{

	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	//initialize the communication flag
	g_ucCOMM_Flags = 0x00;

	//set the pins and registers of the active SP in the S_ActiveSP structure
	if (ucSP_SetActiveBoard(ucSPNumber))
		return SP_NOT_ATTACHED;

	//Set the transmit pin
	*S_ActiveSP.m_ucActiveSP_RegDir |= g_ucaSPTxPins[ucSPNumber];

	//Set the Receive pins
	P_RX_DIR &= ~S_ActiveSP.m_ucActiveSP_BitRx;
	// Enable the falling edge interrupt on RX to see start bits
	P_RX_IES |= S_ActiveSP.m_ucActiveSP_BitRx;
	P_RX_IFG &= ~S_ActiveSP.m_ucActiveSP_BitRx;
	P_RX_IE |= S_ActiveSP.m_ucActiveSP_BitRx;

	// Set the interrupt pin low
	P_INT_DIR |= S_ActiveSP.m_ucActiveSP_BitInt;
	P_INT_OUT &= ~S_ActiveSP.m_ucActiveSP_BitInt;

	// Clear the RX buffer and reset index
	for (g_ucRXBufferIndex = 0x00; g_ucRXBufferIndex < RX_BUFFER_SIZE; g_ucRXBufferIndex++)
	{
		g_ucaRXBuffer[g_ucRXBufferIndex] = 0x00;
	}
	g_ucRXBufferIndex = 0x00;

	// Set the size of the received message to the minimum
	g_ucRXMessageSize = SP_HEADERSIZE;

	// BUG FIX: Clear TACTL in case someone was using it before us
	TA0CTL = 0x0000;
	TA0EX0 = 0x00;

	// Hold TimerA in reset
	TA0CTL &= ~(MC0 | MC1);

	// Use the SMCLK, enable CCR0 interrupt
	TA0CTL |= TASSEL_2;
	TA0CCTL0 |= CCIE;

	// The timer interrupt controls the baud rate, currently configured for a
	// 4 MHz SMCLK
	g_unCOMM_BaudRateControl = ucBaud;
	switch (g_unCOMM_BaudRateControl)
	{
		case BAUD_9600:
			g_unCOMM_BaudRateDelayControl = BAUD_9600_DELAY;
		break;

		case BAUD_19200:
			g_unCOMM_BaudRateDelayControl = BAUD_19200_DELAY;
		break;

		case BAUD_57600:
			g_unCOMM_BaudRateDelayControl = BAUD_57600_DELAY;
		break;

		case BAUD_115200:
			g_unCOMM_BaudRateDelayControl = BAUD_115200_DELAY;
		break;

		case BAUD_230400:
			g_unCOMM_BaudRateDelayControl = BAUD_230400_DELAY;
		break;

		case BAUD_345600:
			g_unCOMM_BaudRateDelayControl = BAUD_345600_DELAY;
		break;

		case BAUD_460800:
			g_unCOMM_BaudRateDelayControl = BAUD_460800_DELAY;
		break;

		default:
			g_unCOMM_BaudRateControl = BAUD_9600;
			g_unCOMM_BaudRateDelayControl = BAUD_9600_DELAY;
		break;
	}

	TA0CCR0 = g_unCOMM_BaudRateControl;

	g_ucCOMM_Flags |= COMM_RUNNING;

	return COMM_OK;
}


///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a byte via the software UART
//!
//! This function pushes \e ucChar into the global TX buffer, where the
//! TimerA ISR can access it. The system drops into LPM0, keeping the SMCLK
//! alive for TimerA. The ISR handles the start and stop bits as well as the
//! baud rate. This is a blocking call and will not return until the software
//! has sent the entire message.
//!   \param ucChar The 8-bit value to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////
static void vSP_SendByte(uint8 ucTXChar)
{

	uint8 ucParityBit;
	uint8 ucBitIdx;
	uint8 ucTXBitsLeft;

	// If we are already busy, return so as not to screw it up
	if (g_ucCOMM_Flags & COMM_TX_BUSY)
		return;

	P_RX_IE &= ~S_ActiveSP.m_ucActiveSP_BitRx;

	// Indicate in the status register that we are now busy
	g_ucCOMM_Flags |= COMM_TX_BUSY;

	// Calculate the parity bit prior to transmission
	ucParityBit = 0;
	for(ucBitIdx = 0; ucBitIdx < 8; ucBitIdx++)
	{
		ucParityBit ^= ((ucTXChar & 0x01)>>ucBitIdx);
	}

	// Reset the bit count so the ISR knows how many bits left to send
	ucTXBitsLeft = 0x0A;

	TA0CCR0 = g_unCOMM_BaudRateControl;
	// Starts the counter in 'Up-Mode'
	TA0CTL |= TACLR | MC_1;

	while (g_ucCOMM_Flags & COMM_TX_BUSY)
	{
		switch (ucTXBitsLeft)
		{
			case 0x00:
				// Last bit is stop bit, return to idle state
				*S_ActiveSP.m_ucActiveSP_RegOut |= S_ActiveSP.m_ucActiveSP_BitTx;
				__bis_SR_register(GIE + LPM0_bits);
				g_ucCOMM_Flags &= ~COMM_TX_BUSY;
			break;

			case 0x01:
				if (ucParityBit)
					*S_ActiveSP.m_ucActiveSP_RegOut |= S_ActiveSP.m_ucActiveSP_BitTx;
				else
					*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;

				__bis_SR_register(GIE + LPM0_bits);
			break;

			case 0x0A:
				if (ucTXChar & 0x01)
					*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;
				else
					*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;

				// First bit is start bit
//				*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;
				__bis_SR_register(GIE + LPM0_bits);
			break;

			default:
				// For data bits, mask to get correct value and the shift for next time
				if (ucTXChar & 0x01)
					*S_ActiveSP.m_ucActiveSP_RegOut |= S_ActiveSP.m_ucActiveSP_BitTx;
				else
					*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;
				ucTXChar >>= 1;
				__bis_SR_register(GIE + LPM0_bits);
			break;
		}

		// Decrement the total bit count
		ucTXBitsLeft--;

	}

	P_RX_IE |= S_ActiveSP.m_ucActiveSP_BitRx;

	// Stop the timer
	TA0CTL &= ~(MC0 | MC1 | TAIFG);

	P3OUT |=  BIT4;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Receives a byte via the software UART
//!
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
static uint8 ucSP_ReceiveByte(void)
{
	uint8 ucRXBitsLeft;
	uint8 ucParityBit;  // The calculated parity bit
	uint8 ucRxParityBit; // The received parity bit
	uint8 ucBitIdx;


	// Set up timer
	TA0CTL = (TASSEL_2 | TACLR);
	TA0CCTL0 &= ~CCIE;
	TA0CCR0 = 0xFFFF;

	// Wait for the port interrupt to exit LPM and continue
	TA0CCTL0 |= CCIE;
	TA0CTL |= MC_1;

	// shut off MCU
	LPM0;

	P2IE &= ~0x0F; // Disable interrupts on the RX pin
	TA0CTL = (TASSEL_2 | TACLR);
	TA0CCTL0 &= ~CCIE;

	//If a port interrupt was received then we are in the RX active state else exit
	if(	!(g_ucCOMM_Flags & COMM_RX_BUSY))
	{
		return COMM_ERROR;
	}

	ucRXBitsLeft = 0x09;

	// Start timer and delay for half a bit
	TA0CCR0 = g_unCOMM_BaudRateDelayControl;
	TA0CTL |= MC_1;
	while (!(TA0CTL & TAIFG));
	TA0CTL &= ~TAIFG;

	// Set up timer for comm. at the baud rate
	TA0CTL = (TASSEL_2 | TACLR);
	TA0CCTL0 = CCIE;
	TA0CCR0 = g_unCOMM_BaudRateControl;

	// Start the timer
	TA0CTL |= MC_1;

	while(g_ucCOMM_Flags & COMM_RX_BUSY)
	{
			switch (ucRXBitsLeft)
			{
				case 0x00:
					// There are no bits left, so lets reset all the values and stop timer
					TA0CTL = (TASSEL_2 | TACLR);
					P_RX_IFG &= ~S_ActiveSP.m_ucActiveSP_BitRx;
					P_RX_IE |= S_ActiveSP.m_ucActiveSP_BitRx;
					g_ucCOMM_Flags &= ~COMM_RX_BUSY;
					// Increment index for next byte
					g_ucRXBufferIndex++;
				break;

				// Parity Bit
				case 0x01:
					if (P_RX_IN & S_ActiveSP.m_ucActiveSP_BitRx)
						ucRxParityBit = 1;
					else
						ucRxParityBit = 0;

					// shut off MCU
					LPM0;
				break;

				// Last data bit no shift
				case 0x02:
					if (P_RX_IN & S_ActiveSP.m_ucActiveSP_BitRx)
						g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
					else
						g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;

					// shut off MCU
					LPM0;
				break;

				default:
					if (P_RX_IN & S_ActiveSP.m_ucActiveSP_BitRx)
						g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
					else
						g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;

					g_ucaRXBuffer[g_ucRXBufferIndex] >>= 1;
					LPM0;
				break;

			}
			ucRXBitsLeft--;

	}

	// Check Parity
//	ucParityBit = 0;
//	for(ucBitIdx = 0; ucBitIdx < 8; ucBitIdx++)
//	{
//		ucParityBit ^= ((g_ucaRXBuffer[g_ucRXBufferIndex] & 0x01)>>ucBitIdx);
//	}

	//Todo move the parity check to the read from buffer function to keep comm simple
//	if(ucParityBit != ucRxParityBit)
//		return COMM_PARITY_ERR;

	return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Shuts off the software modules
//!
//! This shuts down TimerA and disables all of the interrupts used. The vSP_ShutdownComm()
//! function must be used when switching between SP boards otherwise the communication
//! will fail.
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_ShutdownComm(void)
{
	// Halt timer and clear interrupt enables
	TA0CTL &= ~(MC0 | MC1 | TAIE | TAIFG);
	TA0CCTL0 &= ~CCIE;

	//Disable active SP
	S_ActiveSP.m_ucActiveSP = 0xFF;

	//Disable interrupts on the RX line
	P_RX_IE &= ~S_ActiveSP.m_ucActiveSP_BitRx;

	// Pull interrupt line low
	P_INT_OUT &= ~S_ActiveSP.m_ucActiveSP_BitInt;

	//Indicate the comm channel is inactive
	g_ucCOMM_Flags &= ~COMM_RUNNING;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Cleans 128 Bit buffer
//!
//!
//!   \param The buffer being cleaned
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_Clean128BitBuffer(union SP_DataMessage * message)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_DATAMESSAGE_SIZE; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = 0x00;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Cleans 32 Bit buffer
//!
//!
//!   \param The buffer being cleaned
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_Clean32BitBuffer(union SP_DataMessage * message)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_DATAMESSAGE_SIZE; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = 0x00;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a data message
//!
//! This function waits for a data message to be received on the serial port.
//!   \param None.
//!   \return The error code indicating the status after call
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_WaitForMessage(void)
{

  // Set the size of the received message to the minimum
	g_ucRXMessageSize = SP_HEADERSIZE;

	// Wait to receive the message
	do // saves 3 cycles as opposed to while
	{

		if(ucSP_ReceiveByte())
		{
			return COMM_ERROR;
		}

		// If we have received the header of the message, update the RX message to
		// the size of the message received
		if (g_ucRXBufferIndex == SP_HEADERSIZE)
		{
			g_ucRXMessageSize = g_ucaRXBuffer[0x01];

			// Range check the g_ucRXMessageSize variable
			if (g_ucRXMessageSize > MAX_SP_MSGSIZE || g_ucRXMessageSize < SP_HEADERSIZE)
				return 0x04;
		}


	}while (g_ucRXBufferIndex != g_ucRXMessageSize);

	// No message received
	if (g_ucRXBufferIndex == 0)
	{
		g_ucRXMessageSize = 0;
		return COMM_ERROR;
	}

//success
	return 0;
}



///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a label message
//!
//! This function waits for a label message to be received on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a label message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_WaitForLabelMessage(void)
{
	uint16 unLoopCount = 0x0000;

	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	while ((g_ucRXBufferIndex != SP_LABELMESSAGE_SIZE) && (unLoopCount != 0xFFFF))
	{
		unLoopCount++;
	}
	if (unLoopCount == 0xFFFF)
	{
		return 0;
	}

	//success
	return 1;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data message on the serial port
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vSP_Send32BitDataMessage(union SP_DataMessage * p_DataMessage, uint8 ucSPNumber)
{
	uint8 ucLoopCount;

	//Toggle the interrupt pin some SP boards require time to wake from deep sleep
	P_INT_OUT |= S_ActiveSP.m_ucActiveSP_BitInt;
	P_INT_OUT &= ~S_ActiveSP.m_ucActiveSP_BitInt;

	for (ucLoopCount = 0x00; ucLoopCount < 8; ucLoopCount++)
		vSP_SendByte(p_DataMessage->ucByteStream[ucLoopCount]);

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a data message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the data message pointed to by \e message.
//!   \param message Pointer to the message
//!   \return The error code indicating the status after call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_Grab32BitDataMessageFromBuffer(union SP_DataMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXBufferIndex < g_ucaRXBuffer[0x01])
		return COMM_ERROR;

	for (ucLoopCount = 0x00; ucLoopCount < SP_DATAMESSAGE_SIZE; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

	g_ucRXBufferIndex = 0x00;

	return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a data message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the data message pointed to by \e message.
//!   \param message Pointer to the message
//!   \return The error code indicating the status after call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_Grab128BitDataMessageFromBuffer(union SP_DataMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXMessageSize == 0 || g_ucRXMessageSize > MAX_SP_MSGSIZE)
		return 0x01;

	for (ucLoopCount = 0x00; ucLoopCount < g_ucRXMessageSize; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

	g_ucRXBufferIndex = 0x00;

	return COMM_OK;
}
///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a label message
//!
//! This function takes the characters from \e g_ucaRXBuffer and formats and
//! stores them in the label message pointed to by \e message.
//!   \param message Pointer to the label message
//!   \return The error code indicating the status of the call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_GrabLabelMessageFromBuffer(union SP_LabelMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXBufferIndex < SP_LABELMESSAGE_SIZE)
		return COMM_ERROR;

	for (ucLoopCount = 0x00; ucLoopCount < SP_LABELMESSAGE_SIZE; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

	g_ucRXBufferIndex = 0x00;

	return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Turns on and verifies the SP is the correct one.  Leaves the SP board
//!				 on but disables the timers and ISR upon exit.
//!
//!
//!   \param ucSPNumber, ucPrint
//!   \return 1 Error, 0 Success
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_Start(uint8 ucSPNumber, uint8 ucPrint)
{
	uint8 ucWakeUpCount;
	uint8 ucCounter; // Generic counter used in several loops
	uint32 ulTempSN; //Temporary serial number

	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	//Assume the SP is attached
	S_AttachedSP.m_ucAttachedSPs |= ucaSPBit[ucSPNumber];

	// Initialize the Timer and the TX and RX pins,buffer, and the RX interrupt.
	ucSP_SetState(SP_OPMODE_BAUD, ucSPNumber);

	// set the state of the SP to active in the attached SP structure
	S_AttachedSP.m_uiaSP_Status[ucSPNumber] = SP_STATE_ACTIVE;

	//Initialize the counter that keeps track of attempts to wake up the SP board
	ucWakeUpCount = 0x05;

	//Attempt to wake up the SP board a few times
	while (ucWakeUpCount != 0)
	{
		ucSP_SetState(SP_OPMODE_BAUD, ucSPNumber);
		vSP_TurnOn(ucSPNumber);
		if (!(ucSP_WaitForMessage()))
		{
			break;
		}
		vSP_TurnOff(ucSPNumber);
		ucWakeUpCount--;
	}

	//If the counter equals zero then the wake up attempt was a failure
	if (ucWakeUpCount == 0x00)
	{
		vSP_ShutdownComm(); // Shutdown communication
		vSP_TurnOff(ucSPNumber); // Power off the board

		vSP_DetachBoard(ucSPNumber); // Clear the board from the SP structure

		return 1;
	}

	//If we are here then the wake up was successful
	//Get the received data after cleaning the buffer being passed
	vSP_Clean32BitBuffer(&g_RecDataMsg);
	ucSP_Grab32BitDataMessageFromBuffer(&g_RecDataMsg);

	// Get the serial number from the received message
	ulTempSN = 0;
	for (ucCounter = 4; ucCounter < 8; ucCounter++)
	{
		ulTempSN |= g_RecDataMsg.ucByteStream[ucCounter];
		ulTempSN = (ulTempSN << 8);
	}

	// Compare SP serial number with what the driver has on record
	if (ulTempSN != S_AttachedSP.m_ulaSP_SerialNumber[ucSPNumber])
	{
		// Update the version number
		S_AttachedSP.m_ucaSP_MSG_Version[ucSPNumber] = g_RecDataMsg.fields.ucMsgVersion;

		vSP_Clean128BitBuffer(&g_RecDataMsg);

		//The serial number doesn't match so we must interrogate it
		if (ucSP_SendInterrogate(ucSPNumber))
			return 1; // Command failed

		//If we are here the interrogation was successful

		// Set the SP to inactive for the rest of the frame.
		// This is done to prevent sending commands to the SP in future slots within the current frame
		// when the scheduler is unaware that the SP has changed
		S_AttachedSP.m_uiaSP_Status[ucSPNumber] = SP_STATE_INACTIVE;

		// Get the SP boards name from the final 8 bytes in the interrogate packet
		for (ucCounter = 0; ucCounter < SP_NAME_LENGTH; ucCounter++)
		{
			S_AttachedSP.m_ucaSP_Name[ucSPNumber][ucCounter] = g_RecDataMsg.fields.ucaParameters[ucCounter + 8];
		}

		// Update driver with the current serial number
		S_AttachedSP.m_ulaSP_SerialNumber[ucSPNumber] = ulTempSN;

		// Get the number of transducers from the SP
		S_AttachedSP.m_ucaSP_NumTransducers[ucSPNumber] = g_RecDataMsg.fields.ucSensorNumber;

		// Get the transducer types (sensor or actuator)
		for (ucCounter = 0; ucCounter < NUMBER_TRANSDUCERS; ucCounter++)
		{
			S_AttachedSP.m_ucaSP_TypeTransducers[ucSPNumber][ucCounter] = g_RecDataMsg.fields.ucaParameters[ucCounter];
		}

		// Get the transducer labels
		vSP_RequestSP_TransducerLabels(ucSPNumber);
	}

	// Display the state of the driver if requested
	if (ucPrint)
	{
		vSP_Display();
	}

// Stop timer and disable interrupts
	vSP_ShutdownComm();

	return 0; //success

}


/////////////////////////////////////////////////////////////////////////////////

//Write a set of functions here that return required information to the task manager
//		without passing some complex data type structs etc..

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the number of transducers and SP board has
/////////////////////////////////////////////////////////////////////////////////
uchar ucSP_IsAttached(uchar ucSPNumber)
{
	if(S_AttachedSP.m_ucAttachedSPs & ucaSPBit[ucSPNumber])
		return 1;

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the number of transducers and SP board has
/////////////////////////////////////////////////////////////////////////////////
uchar ucSP_FetchNumTransducers(uchar ucSPNumber)
{
	return S_AttachedSP.m_ucaSP_NumTransducers[ucSPNumber];
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the type of transducer and SP board has
/////////////////////////////////////////////////////////////////////////////////
uchar ucSP_FetchTransType(uchar ucSPNumber, uchar ucTransNumber)
{
	return S_AttachedSP.m_ucaSP_TypeTransducers[ucSPNumber][ucTransNumber];
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the status of the SP board
/////////////////////////////////////////////////////////////////////////////////
uint16 uiSP_FetchSPState(uchar ucSPNumber)
{
	return S_AttachedSP.m_uiaSP_Status[ucSPNumber];
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the name of the SP board
/////////////////////////////////////////////////////////////////////////////////
void vSP_FetchTransName(uchar ucSPNumber, uchar ucTransNum, char *p_caName)
{
	uint8 ucIndex;

	//Display the characters in the cpaSPLabels array only if they are ascii characters
	for (ucIndex = 0; ucIndex < LABEL_LENGTH; ucIndex++)
	{
		*p_caName = S_AttachedSP.m_cpaSPLabels[ucSPNumber][ucTransNum][ucIndex];
		p_caName++;
	}

}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Returns the name of the SP board
/////////////////////////////////////////////////////////////////////////////////
void vSP_FetchSPName(uchar ucSPNumber, uchar *p_caName)
{
	uint8 ucIndex;

	//Display the characters in the cpaSPLabels array only if they are ascii characters
	for (ucIndex = 0; ucIndex < SP_NAME_LENGTH; ucIndex++)
	{
		*p_caName = S_AttachedSP.m_ucaSP_Name[ucSPNumber][ucIndex];
		p_caName++;
	}

}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Displays the SP information such as who is attached their state and
//!  capabilities
//!
//! I know the code looks ugly but it gets the formatting right
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////////////
void vSP_Display(void)
{
	uchar ucSP_Number;
	uchar ucCounter; // general purpose counters reused several times

	vSERIAL_rom_sout("------- SP1 ------------ SP2 ------------ SP3 ------------ SP4 ---");
	vSERIAL_crlf();
	vSERIAL_rom_sout("Name    ");

	// Display SP names
	for (ucSP_Number = 0; ucSP_Number < NUMBER_SPBOARDS; ucSP_Number++)
	{
		for (ucCounter = 0; ucCounter < SP_NAME_LENGTH; ucCounter++)
		{
			if (S_AttachedSP.m_ucaSP_Name[ucSP_Number][ucCounter] != 0x00)
			{
				vSERIAL_bout((uchar) S_AttachedSP.m_ucaSP_Name[ucSP_Number][ucCounter]);
			}
			else
			{
				vSERIAL_rom_sout(" ");
			}
		}
		vSERIAL_rom_sout("         ");
	}
	vSERIAL_crlf();

	// Display SP serial numbers
	vSERIAL_rom_sout("SN      ");
	for (ucSP_Number = 0; ucSP_Number < NUMBER_SPBOARDS; ucSP_Number++)
	{
		vSERIAL_HB32Fout(S_AttachedSP.m_ulaSP_SerialNumber[ucSP_Number]);
		vSERIAL_rom_sout("       ");
	}
	vSERIAL_crlf();

	vSERIAL_rom_sout("Labels");
	vSERIAL_crlf();

	// Display labels
	for (ucCounter = 0; ucCounter < NUM_OF_LABELS; ucCounter++)
	{
		vSERIAL_rom_sout("        ");

		for (ucSP_Number = 0; ucSP_Number < NUMBER_SPBOARDS; ucSP_Number++)
		{
			vSP_DisplaySingleLabel(ucSP_Number, ucCounter);
			vSERIAL_rom_sout(" ");
		}
		vSERIAL_crlf();
	}
} //END: vSP_Display()


///////////////////////////////////////////////////////////////////////////////
//! \brief Input the correct parameters into the packet to request data, send.
//!
//!   \param uint8 ucSPnumber, uint8LabelIndex
//!   \return none
///////////////////////////////////////////////////////////////////////////////
void vSP_RequestSingle_TransducerLabel(uint8 ucSPnumber, uint8 ucLabelIndex)
{
	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Do not proceed if there is not a functioning board present
	{
		vSP_Clean128BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgType = REQUEST_LABEL; //!< Type of the message.
		g_SendDataMsg.fields.ucSensorNumber = ucLabelIndex; //!< The index into the label message array on the SP board
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab128BitDataMessageFromBuffer(&g_RecDataMsg);
		vSP_ShutdownComm();
	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Get labels for a single SP board
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
void vSP_RequestSP_TransducerLabels(uint8 ucSPNumber)
{
	uint8 ucLabelIndex; //!< The index into the label message array on the SP board
	uint8 ucCharIndex;

	for (ucLabelIndex = 0; ucLabelIndex < 0x12; ucLabelIndex++)
	{
		vSP_RequestSingle_TransducerLabel(ucSPNumber, ucLabelIndex);
		for (ucCharIndex = 0; ucCharIndex < 0x0F; ucCharIndex++)
		{
			S_AttachedSP.m_cpaSPLabels[ucSPNumber][ucLabelIndex][ucCharIndex] = g_RecDataMsg.fields.ucaParameters[ucCharIndex];
		}

	}

} //end: vSP_RequestSPTransducerLabels

///////////////////////////////////////////////////////////////////////////////
//! \brief Loop through all possible SP boards to get the labels.
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
void vSP_RequestAll_TransducerLabels(void)
{
	uint8 ucSPNumber; //!< The SP board

	for (ucSPNumber = 0; ucSPNumber < 4; ucSPNumber++)
	{
		vSP_RequestSP_TransducerLabels(ucSPNumber);
	}
} //end: vSP_RequestAllLabels

///////////////////////////////////////////////////////////////////////////////
//! \brief Display a single transducer label
//!
//!   \param ucSPNumber, ucTransducer
//!   \return none
///////////////////////////////////////////////////////////////////////////////
void vSP_DisplaySingleLabel(uint8 ucSPNumber, uint8 ucTransducer)
{
	uint8 ucIndex;

//Display the characters in the cpaSPLabels array only if they are ascii characters
	for (ucIndex = 0; ucIndex < LABEL_LENGTH; ucIndex++)
	{
		if ((S_AttachedSP.m_cpaSPLabels[ucSPNumber][ucTransducer][ucIndex] > 0x2F)
		    && (S_AttachedSP.m_cpaSPLabels[ucSPNumber][ucTransducer][ucIndex] < 0x7B))
		{
			vSERIAL_bout(S_AttachedSP.m_cpaSPLabels[ucSPNumber][ucTransducer][ucIndex]);
		}
		else
			vSERIAL_rom_sout(" ");
	}

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Display all transducer labels for an SP
//!
//!   \param ucSPNumber,
//!   \return none
///////////////////////////////////////////////////////////////////////////////
void vSP_DisplayLabels(uint8 ucSPNumber)
{
	uint8 ucTransducer;

//Display the characters in the cpaSPLabels array only if they are ascii characters
	for (ucTransducer = 0; ucTransducer < 0x10; ucTransducer++)
	{
		vSP_DisplaySingleLabel(ucSPNumber, ucTransducer);
	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Input the correct parameters into the packet to request data, send.
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 vSP_Request128BitsData(uint8 ucSPnumber)
{
	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Do not proceed if there is not a functioning board present
	{
		vSP_Clean128BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgType = REQUEST_DATA; //!< Type of the message.
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab128BitDataMessageFromBuffer(&g_RecDataMsg);
		vSP_ShutdownComm();
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Input the correct parameters into the packet to request data, send.
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 vSP_Request32BitsData(uint8 ucSPnumber)
{
	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Do not proceed if there is not a functioning board present
	{
		vSP_Clean32BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgType = REQUEST_DATA; //!< Type of the message.
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab32BitDataMessageFromBuffer(&g_RecDataMsg);
		vSP_ShutdownComm();
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Input the correct parameters into the packet to request data, send.
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 vSP_RequestData(uint8 ucSPnumber)
{
	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Do not proceed if there is not a functioning board present
	{
		vSP_Clean32BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgType = REQUEST_DATA; //!< Type of the message.
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab32BitDataMessageFromBuffer(&g_RecDataMsg);
		if (g_RecDataMsg.fields.ucMsgType == REPORT_DATA)
			vSERIAL_rom_sout("Report\n\r");
		vSP_ShutdownComm();
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Input the correct parameters into the packet to send a command, send.
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_SendCommand(uint8 ucSPNumber, uint8 ucTransducer, uint8 ucByte1High, uint8 ucByte1Low, uint8 ucByte2High, uint8 ucByte2Low)
{

	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;


	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPNumber)) //Do not proceed if there is not a functioning board present
	{

		// Set the state of the transducer to active
		S_AttachedSP.m_uiaSP_Status[ucSPNumber] |= ucTransducer;

		vSP_Clean32BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgType = COMMAND_PKT; //!< Type of the message.
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		g_SendDataMsg.fields.ucSensorNumber = ucTransducer; //!< Sensor number.
		g_SendDataMsg.fields.ucaParameters[0] = ucByte1High; //!< High byte of the first data field
		g_SendDataMsg.fields.ucaParameters[1] = ucByte1Low; //!< Low byte of the first data field
		g_SendDataMsg.fields.ucaParameters[2] = ucByte2High; //!< High byte of the second data field
		g_SendDataMsg.fields.ucaParameters[3] = ucByte2Low; //!< Low byte of the second data field
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPNumber);

		//
		//P3OUT ^= BIT4;

		ucSP_WaitForMessage();
		ucSP_Grab32BitDataMessageFromBuffer(&g_RecDataMsg);
		if (g_RecDataMsg.fields.ucMsgType == CONFIRM_COMMAND)
			vSERIAL_rom_sout("Command Confirmed\n\r");
		vSP_ShutdownComm();
		return 0;
	}
//Return success
	return SP_NOT_ATTACHED;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Request transducer information from the SP and sets an ID in flash
//!				 memory of the SP board.
//!
//!	This function assists the task manager in determining what tasks the SP can perform.
//! The SP ID helps the CP make sure that the attached SPs have not been changed.
//! In the event that they have been changed this function is called to reconfigure
//! the SP driver to match the parameters of the new SP board
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_SendInterrogate(uint8 ucSPnumber)
{
	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Proceed only if there is a functioning board present
	{
		vSP_Clean32BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgType = INTERROGATE; //!< Type of the message.
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol
		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab128BitDataMessageFromBuffer(&g_RecDataMsg);
		if (g_RecDataMsg.fields.ucMsgType == INTERROGATE)
			vSERIAL_rom_sout("Interrogated\n\r");

		vSP_ShutdownComm();
		return COMM_OK;
	}
	return SP_NOT_ATTACHED;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Request transducer information from the SP and sets an ID in flash
//!				 memory of the SP board.
//!
//!	This function assists the task manager in determining what tasks the SP can perform.
//! The SP ID helps the CP make sure that the attached SPs have not been changed.
//! In the event that they have been changed this function is called to reconfigure
//! the SP driver to match the parameters of the new SP board
//!
//!   \param none
//!   \return none
///////////////////////////////////////////////////////////////////////////////
uint8 ucSP_SetSerialNum(uint8 ucSPnumber, uint32 ulSerialNum)
{
	if (g_eSP_DriverState == SP_DRIVER_SHUTDOWN)
		return SP_BAD_STATE;

	if (!ucSP_SetState(SP_OPMODE_BAUD, ucSPnumber)) //Do not proceed if there is not a functioning board present
	{
		vSP_Clean32BitBuffer(&g_RecDataMsg);
		g_SendDataMsg.fields.ucMsgType = SET_SERIALNUM; //!< Type of the message.
		g_SendDataMsg.fields.ucMsgSize = SP_DATAMESSAGE_SIZE; //!< Size of the message
		g_SendDataMsg.fields.ucMsgVersion = SP_DATAMESSAGE_VERSION; //!< The version number of the message protocol

		g_SendDataMsg.fields.ucaParameters[0] = (uint8) ulSerialNum;
		g_SendDataMsg.fields.ucaParameters[1] = (uint8) (ulSerialNum >> 8);
		g_SendDataMsg.fields.ucaParameters[2] = (uint8) (ulSerialNum >> 8);
		g_SendDataMsg.fields.ucaParameters[3] = (uint8) (ulSerialNum >> 8);

		vSP_Send32BitDataMessage(&g_SendDataMsg, ucSPnumber);

		ucSP_WaitForMessage();
		ucSP_Grab128BitDataMessageFromBuffer(&g_RecDataMsg);
		if (g_RecDataMsg.fields.ucMsgType == INTERROGATE)
			vSERIAL_rom_sout("Interrogated\n\r");

		vSP_ShutdownComm();
		return SP_NOT_ATTACHED;
	}
//Return success
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Wakes up all attached SP boards and determines what type they are
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////////////
void vSP_SetRole(void)
{
	ucSP_Start(SP1, NO_PRINT);
	ucSP_Start(SP2, NO_PRINT);
	ucSP_Start(SP3, NO_PRINT);
	ucSP_Start(SP4, NO_PRINT);

//Turn off the SPs
	vSP_TurnoffAll();
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Wakes up SP boards and displays message version number
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////////////
void vSP_GetSPMsgVersions(void)
{

	if (S_AttachedSP.m_ucaSP_MSG_Version[0] != 0)
	{
		vSERIAL_rom_sout("\r\nSP1_Msg-V");
		vSERIAL_UIV8out((uchar) (S_AttachedSP.m_ucaSP_MSG_Version[0])); //Version Num
	}

	if (S_AttachedSP.m_ucaSP_MSG_Version[1] != 0)
	{
		vSERIAL_rom_sout("\r\nSP2_Msg-V");
		vSERIAL_UIV8out((uchar) (S_AttachedSP.m_ucaSP_MSG_Version[1])); //Version Num
	}

	if (S_AttachedSP.m_ucaSP_MSG_Version[2] != 0)
	{
		vSERIAL_rom_sout("\r\nSP3_Msg-V");
		vSERIAL_UIV8out((uchar) (S_AttachedSP.m_ucaSP_MSG_Version[2])); //Version Num
	}

	if (S_AttachedSP.m_ucaSP_MSG_Version[3] != 0)
	{
		vSERIAL_rom_sout("\r\nSP4_Msg-V");
		vSERIAL_UIV8out((uchar) (S_AttachedSP.m_ucaSP_MSG_Version[3])); //Version Num
	}

	vSP_TurnoffAll();
	vSERIAL_crlf();
}

