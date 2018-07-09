///////////////////////////////////////////////////////////////////////////////
//! \file comm.c
//! \brief This modules implements a software UART on any two Digital I/O pins
//!
//! This module uses TimerA to implement a software UART on any two defined
//! digital I/O pin. Note that the RX pin must also be interrupt capable.
//!
//! @addtogroup core
//! @{
//!
//! @addtogroup comm Software UART
//! The software UART module allows the user to define a UART interface on
//! any two GPIO pins, provided that the RX pin is interrupt capable. The
//! module requires the use of TimerA.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wirless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//*****************************************************************************

#include <msp430x23x.h>
#include "../core.h"

//******************  Pin Configurations  ***********************************//
// Defines for the TX pin (currently 1.1)
//! @name Pin Defines
//! These defines are used so that the software has the right hardware
//! registers for the TX and RX pins.
//! @{
//! \def TX_PIN
//! \brief The pin number of the TX pin (BIT0 to BIT7)
#define TX_PIN          BIT1
//! \def P_TX_OUT
//! \brief The PxOUT register that the TX pin is on
#define P_TX_OUT        P1OUT
//! \def P_TX_DIR
//! \brief The PxDIR register that the TX pin is on
#define P_TX_DIR        P1DIR

// Defines for the RX pin (currently 2.2)
//! \def RX_PIN
//! \brief The pin number of the RX pin (BIT0 to BIT7)
#define RX_PIN          BIT2
//! \def P_RX_DIR
//! \brief The PxDIR register of the RX pin
#define P_RX_DIR        P2DIR
//! \def P_RX_IN
//! \brief The PxIN register of the RX pin
#define P_RX_IN         P2IN
//! \def P_RX_IES
//! \brief The PxIES register of the RX pin
#define P_RX_IES        P2IES
//! \def P_RX_IFG
//! \brief The PxIFG register of the RX pin
#define P_RX_IFG        P2IFG
//! \def P_RX_IE
//! \brief The PxIE register of the RX pin
#define P_RX_IE         P2IE

// Define for the Interrupt pin (currently 2.0)
//! \def INT_PIN
//! \brief The pin number of the INT pin (BIT0 to BIT7)
#define INT_PIN          BIT0
//! @}

//******************  Control and Indication Variables  *********************//
//! @name Control and Indication Variables
//! These variables are used to indicate to the system the current status
//! of the \ref comm Module and to store the baud rate timer information.
//! @{
//! \var volatile uint8 g_ucCOMM_Flags
//! \brief This 8-bit field indicates the status of the COMM module.
volatile uint8 g_ucCOMM_Flags;

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
//! \ref comm_baud_delay "Baud Rate Start Delays".
uint16 g_unCOMM_BaudRateDelayControl;
//! @}

//******************  RX Variables  *****************************************//
//! @name Receive Variables
//! These variables are used in the receiving of data on the \ref comm Module.
//! @{
//! \var volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE]
//! \brief The software UART RX Buffer
volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE];

//! \var volatile uint8 g_ucRXBufferIndex
//! \brief This index into g_ucaRXBuffer showing the current write position.
volatile uint8 g_ucRXBufferIndex;

//! \var uint8 g_ucRXBitsLeft
//! \brief The number of bits left to be received for the current byte.
uint8 g_ucRXBitsLeft;

//! \var uint8 g_ucRXParityBit
//! \brief Even Parity for bit banging uart
uint8 ucRXParityBit;
//! @}

//******************  Functions  ********************************************//
///////////////////////////////////////////////////////////////////////////////
//! \brief This sets up the hardware resources for doing software UART
//!
//! Since we are doing UART without the USCI, we use TimerA and it's interrupt
//! to control the baud rate. The TX and RX pins are completely controllable
//! at compile time. The software UART expects 1 start bit, 8 data bits and
//! 1 stop bit.
//!
//! To ensure correct operation of the software UART, the \ref comm_pins
//! "Comm Pin Defines" must be set correctly.
//!   \param ucBaud The baud rate define to use
//!   \return None
//!   \sa vCOMM_SendByte(), TIMERA0_ISR()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Init(uint16 ucBaud)
{
	// We set the directionality of the TX and RX pins based on the defines
	P_TX_DIR |= TX_PIN;
	P_RX_DIR &= ~RX_PIN;

	// Idle state for UART is high, so take the TX high
	P_TX_OUT |= TX_PIN;

	// Clear the RX buffer and reset index
	for (g_ucRXBufferIndex = 0x00; g_ucRXBufferIndex < RX_BUFFER_SIZE; g_ucRXBufferIndex++)
	{
		g_ucaRXBuffer[g_ucRXBufferIndex] = 0xFF;
	}
	g_ucRXBufferIndex = 0x00;

	// Enable the falling edge interrupt on RX to see start bits
	P_RX_IES |= RX_PIN;
	P_RX_IFG &= ~RX_PIN;
	P_RX_IE |= RX_PIN;

	// Enable interrupts on the dedicated interrupt line (same port as rx)
	P_RX_IES &= ~INT_PIN;
	P_RX_IFG &= ~INT_PIN;
	P_RX_IE |= INT_PIN;

	// BUG FIX: Clear TACTL in case someone was using it before us
	TACTL = 0x0000;

	// Hold TimerA in reset
	TACTL &= ~(MC0 | MC1);

	// Use the SMCLK, enable CCR0 interrupt
	TACTL |= TASSEL_2;
	TACCTL0 |= CCIE;

	// The timer interrupt controls the baud rate, currently configured for a
	// 4 MHz SMCLK
	g_unCOMM_BaudRateControl = ucBaud;
	switch (g_unCOMM_BaudRateControl)
	{
		case BAUD_1200:
			g_unCOMM_BaudRateDelayControl = BAUD_1200_DELAY;
		break;

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

	TACCR0 = g_unCOMM_BaudRateControl;

	g_ucCOMM_Flags = COMM_RUNNING;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a byte via the software UART
//!
//! This function pushes \e ucChar into the global TX buffer, where the
//! TimerA ISR can access it. The system drops into LPM0, keeping the SMCLK
//! alive for TimerA. The ISR handles the start and stop bits as well as the
//! baud rate. This is a blockingk call and will not return until the software
//! has sent the entire message.
//!   \param ucChar The 8-bit value to send
//!   \return None
//!   \sa TIMERA0_ISR(), vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_SendByte(uint8 ucTXChar)
{
	uint8 ucParityBit;
	uint8 ucBitIdx;
	uint8 ucTXBitsLeft;

	// If we are already busy, return so as not to screw it up
	if (g_ucCOMM_Flags & COMM_TX_BUSY)
		return;

	// Indicate in the status register that we are now busy
	g_ucCOMM_Flags |=  COMM_TX_BUSY;
	P_RX_IE &= ~RX_PIN;

	// Calculate the parity bit prior to transmission
	ucParityBit = 0;
	for (ucBitIdx = 0; ucBitIdx < 8; ucBitIdx++)
	{
		ucParityBit ^= ((ucTXChar & 0x01) >> ucBitIdx);
	}

	// Reset the bit count so the ISR knows how many bits left to send
	ucTXBitsLeft = 0x0A;

	TA0CCR0 = g_unCOMM_BaudRateControl;

	// Starts the counter in 'Up-Mode'
	TACTL |= TACLR | MC_1;

	// Transmission loop which controls the UART timing for byte transmission
	while (g_ucCOMM_Flags & COMM_TX_BUSY)
	{
		switch (ucTXBitsLeft)
		{
			case 0x00:
				// Last bit is stop bit, return to idle state
				P_TX_OUT |= TX_PIN;
				g_ucCOMM_Flags &= ~COMM_TX_BUSY;
			break;

			case 0x01:
				if (ucParityBit)
					P_TX_OUT |= TX_PIN;
				else
					P_TX_OUT &= ~TX_PIN;
			break;

			case 0x0A:
				// First bit is start bit
				P_TX_OUT &= ~TX_PIN;
			break;

			default:
				// For data bits, mask to get correct value and the shift for next time
				if (ucTXChar & 0x01)
					P_TX_OUT |= TX_PIN;
				else
					P_TX_OUT &= ~TX_PIN;
				ucTXChar >>= 1;
			break;
		}

		__bis_SR_register(GIE + LPM0_bits);

		// Decrement the total bit count
		ucTXBitsLeft--;
	}

	P_RX_IE |= RX_PIN;

	// Stop the timer and show we are done
	TACTL &= ~(MC0 | MC1 | TAIFG);
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Receives a byte via the software UART
//!
//!   \param none
//!   \return error code
//!   \sa TIMERA0_ISR(), vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_ReceiveByte(void)
{
	uint8 ucRXBitsLeft;
	uint8 ucParityBit; // The calculated parity bit
	uint8 ucRxParityBit; // The received parity bit
	uint8 ucBitIdx;

	ucRXBitsLeft = 0x09;

	LPM0; //wait for start bit

	P_RX_IE &= ~RX_PIN; // Disable interrupts on the RX line
	TACTL = (TASSEL_2 | TACLR);
	TACCTL0 &= ~CCIE;

	// Start timer and delay for half a bit
	TACCR0 = g_unCOMM_BaudRateDelayControl;
	TACTL |= MC_1;
	while (!(TACTL & TAIFG));
	TACTL &= ~TAIFG;

	// Set up timer for comm. at the baud rate
	TACTL = (TASSEL_2 | TACLR);
	TACCTL0 = CCIE;
	TACCR0 = g_unCOMM_BaudRateControl;

	// Start the timer
	TACTL |= MC_1;

	while (g_ucCOMM_Flags & COMM_RX_BUSY)
	{
		switch (ucRXBitsLeft)
		{
			case 0x00:
				// There are no bits left, so lets reset all the values and stop timer
				TACTL &= ~(MC0 | MC1);
				P_RX_IE |= RX_PIN;
				P_RX_IFG &= ~RX_PIN;
				g_ucCOMM_Flags &= ~COMM_RX_BUSY;
			break;

				// Parity Bit
			case 0x01:
				P5OUT ^= BIT2;
				if (P_RX_IN & RX_PIN)
					ucRxParityBit = 1;
				else
					ucRxParityBit = 0;
				LPM0;
			break;

				// Last data bit no shift
			case 0x02:
				P5OUT ^= BIT2;
				if (P_RX_IN & RX_PIN)
					g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
				else
					g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;
				LPM0;
			break;

			default:
				P5OUT ^= BIT2;
				if (P_RX_IN & RX_PIN)
					g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
				else
					g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;
				g_ucaRXBuffer[g_ucRXBufferIndex] >>= 1;
				LPM0;
			break;

		}
		ucRXBitsLeft--;

	}

	g_ucRXBufferIndex++; 	// Increment index for next byte

	// Check Parity
	ucParityBit = 0;
	for (ucBitIdx = 0; ucBitIdx < 8; ucBitIdx++)
	{
		ucParityBit ^= ((g_ucaRXBuffer[g_ucRXBufferIndex] & 0x01) >> ucBitIdx);
	}

	// ToDo move the parity check to the read from buffer function, leave the comm simple
//	if (ucParityBit != ucRxParityBit)
//		return COMM_PARITY_ERR;

	if (ucParityBit != ucRxParityBit)
		g_ucCOMM_Flags |= COMM_PARITY_ERR;

	return COMM_OK;

}
///////////////////////////////////////////////////////////////////////////////
//! \brief Shuts off the software modules
//!
//! This shuts down TimerA and disables all of the interrupts used
//!   \param None
//!   \return None
//!   \sa vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Shutdown(void)
{
	// Halt timer and clear interrupt enables
	TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);
	TACCTL0 &= ~CCIE;

	// Disable RX interrupt
	P_RX_IE &= ~RX_PIN;
	g_ucCOMM_Flags &= ~COMM_RUNNING;

	//Let TX drop
	P_TX_OUT &= ~TX_PIN;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a data message
//!
//! This function waits for a data message to be received on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a data message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_WaitFor32BitDataMessage(void)
{

	while (g_ucRXBufferIndex != SP_32BITDATAMESSAGE_SIZE)
	{
		ucCOMM_ReceiveByte();
	}

	// toggle trigger
	P5OUT ^= BIT2;

	// wait for a bit
	TA0CCTL0 |= CCIE;
	TA0CCR0 = g_unCOMM_BaudRateDelayControl;
	//TA0CCR0 = 0x00A0;
	TA0CTL |= TACLR | MC_1; // starts timer in up mode

	// low power mode
	LPM0;

	// Halt timer and clear interrupt enables
	TACTL &= ~(MC_0 | MC_1 | TAIE | TAIFG);

	//__delay_cycles(640);
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for reception of a data message
//!
//! This function waits for a data message to be recieved on the serial port.
//! Technically, it waits for the correct number of packets corresponding to a
//! a data message, but since we control both sides of the link and the
//! protocol flow, we can get away with it.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_WaitFor128BitDataMessage(void)
{

	while (g_ucRXBufferIndex != SP_128BITDATAMESSAGE_SIZE)
	{
		__bis_SR_register(LPM0_bits);
		//CPU asleep.
	}
/*
	// stop the timer
	TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);
	TA0CCTL0 &= ~CCIE;

	// wait for a bit
	TA0CTL |= (TASSEL_2 | TACLR);
	TA0CCTL0 |= CCIE;
	TA0CCR0 = g_unCOMM_BaudRateControl;

	// start timer
	TA0CTL |= MC_1;

	// low power mode
	__bis_SR_register(LPM3_bits);

	// Halt timer and clear interrupt enables
	TACTL &= ~(MC0 | MC1 | TAIE | TAIFG);
	TACCTL0 &= ~CCIE;*/
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
void vCOMM_WaitForLabelMessage(void)
{
	while (g_ucRXBufferIndex != SP_LABELMESSAGE_SIZE)
	{

	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data message on the serial port
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////.
void vCOMM_Send32BitDataMessage(union SP_32BitDataMessage * p_32BitDataMessage)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_32BITDATAMESSAGE_SIZE; ucLoopCount++)
		vCOMM_SendByte(p_32BitDataMessage->ucByteStream[ucLoopCount]);

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data message on the serial port
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////.
void vCOMM_Send256BitDataMessage(union SP_256BitDataMessage * p_256BitDataMessage)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_256BITDATAMESSAGE_SIZE; ucLoopCount++)
		vCOMM_SendByte(p_256BitDataMessage->ucByteStream[ucLoopCount]);

} //END: vCOMM_Send256BitDataMessage()

#if SP_PACKET_SIZE_128
///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data return message on the serial port
//! Written by -scb
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////

void vCOMM_Send128BitDataMessage(union SP_128BitDataMessage * p_128BitDataMessage)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_128BITDATAMESSAGE_SIZE; //size is 160 bit (128 + 32)
	    ucLoopCount++)
		vCOMM_SendByte(p_128BitDataMessage->ucByteStream[ucLoopCount]);

}
#endif
///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a label message on the serial port
//!
//! This function sends the label message pointed to by \e p_LabelMessage on
//! the software UART line
//!   \param p_LabelMessage Pointer to the message to send
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCOMM_SendLabelMessage(union SP_LabelMessage * p_LabelMessage)
{
	uint8 ucLoopCount;

	for (ucLoopCount = 0x00; ucLoopCount < SP_LABELMESSAGE_SIZE; ucLoopCount++)
		vCOMM_SendByte(p_LabelMessage->ucByteStream[ucLoopCount]);

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
uint8 ucCOMM_Grab32BitDataMessageFromBuffer(union SP_32BitDataMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXBufferIndex < SP_32BITDATAMESSAGE_SIZE)
		return COMM_BUFFER_UNDERFLOW;

	for (ucLoopCount = 0x00; ucLoopCount < SP_32BITDATAMESSAGE_SIZE; ucLoopCount++)
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
uint8 ucCOMM_Grab128BitDataMessageFromBuffer(union SP_128BitDataMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXBufferIndex < SP_128BITDATAMESSAGE_SIZE)
		return COMM_BUFFER_UNDERFLOW;

	for (ucLoopCount = 0x00; ucLoopCount < SP_128BITDATAMESSAGE_SIZE; ucLoopCount++)
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
//!   \return The error code indicatin the status of the call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_GrabLabelMessageFromBuffer(union SP_LabelMessage * message)
{
	uint8 ucLoopCount;

	if (g_ucRXBufferIndex < SP_LABELMESSAGE_SIZE)
		return COMM_BUFFER_UNDERFLOW;

	for (ucLoopCount = 0x00; ucLoopCount < SP_LABELMESSAGE_SIZE; ucLoopCount++)
		message->ucByteStream[ucLoopCount] = g_ucaRXBuffer[ucLoopCount];

	g_ucRXBufferIndex = 0x00;

	return COMM_OK;
}

///////////////////////////////////////////////////////////////////////////////
//! \brief TimerA0 ISR, sends and receives on the software UART lines
//!
//! TimerA has been configured by \e vCOMM_Init() to generate an interrupt
//! for the bit timing on a specific baud rate. Each time it is called, the
//! ISR checks to see which bit it is sending, control or data. Then the
//! appropriate bits are calculated and sent. If there are no more bits to
//! send, then the ISR returns the system to active mode (AM).
//!
//! This ISR handles the timing for both TX and RX. Therefore, it is best to
//! use this module in half-duplex mode only. Other wise you risk the
//! possibility of having a very long ISR, which will kill the delicate UART
//! timing.
//!
//!
//! NOTE: This interrupt is ONLY for handling CCR0, allowing the user to use
//! the TIMERA1 interrupt for other uses of TA CCR1, CCR2 and TAIFG
//!   \param None
//!   \return None
//!   \sa vCOMM_Init(), vCOMM_SendByte()
///////////////////////////////////////////////////////////////////////////////
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA0_ISR(void)
{

	if (g_ucCOMM_Flags & COMM_RUNNING)
	{
		__bic_SR_register_on_exit(LPM4_bits);
	}

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Port2 ISR, handles the start of the UART RX
//!
//! The idle state for UART is line high, thus when we get the falling edge
//! indicating the start bit, we can start the timer to handle the UART
//! sampling.
//!   \param None
//!   \return None
//!   \sa vCOMM_Init(), TIMERA0_ISR
///////////////////////////////////////////////////////////////////////////////
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	// If the source of the interrupt was the RX pin
	if (P2IFG & RX_PIN)
	{
		// If comm with the CP is running
		if (g_ucCOMM_Flags & COMM_RUNNING)
		{
			g_ucCOMM_Flags |= COMM_RX_BUSY;
			P_RX_IFG &= ~RX_PIN;
			__bic_SR_register_on_exit(LPM0_bits);
		}
	}

	// Wake up from LPM3/4 triggered by the CP to
	// allow the SP time to wake up before receiving a command
	if (P2IFG & INT_PIN)
	{
		P_RX_IFG &= ~INT_PIN;
		__bic_SR_register_on_exit(LPM4_bits);
	}


} //END __interrupt void PORT2_ISR(void)

//! @}
//! @}

