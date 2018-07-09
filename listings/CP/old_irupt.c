/**************************  IRUPT.C  ****************************************
 *
 * Interrupt routines
 *
 *
 * V1.00 01/07/2003 wzr
 *		Started
 *
 ******************************************************************************/

#include <msp430x54x.h>			//processor register description
#include "hal/config.h"			//system configuration definitions
#include "irupt.h"
#include "std.h"
#include "comm.h"
#include "drivers/adf7020.h"	//adf7020 transceiver definitions
#include "drivers/SP.h"			//Satellite Processor definitions
#include "time.h"
/*********************************************************
 * 					Flags
 ********************************************************/
extern volatile union //ucFLAG1_BYTE
{
	uchar byte;

	struct
	{
		unsigned FLG1_X_DONE_BIT :1; //bit 0
		unsigned FLG1_X_LAST_BIT_BIT :1; //bit 1
		unsigned FLG1_X_FLAG_BIT :1; //bit 2 ;1=XMIT, 0=RECEIVE
		unsigned FLG1_R_HAVE_MSG_BIT :1; //bit 3	;1=REC has a msg, 0=no msg
		unsigned FLG1_R_CODE_PHASE_BIT :1; //bit 4 ;1=MSG PHASE, 0=BARKER PHASE
		unsigned FLG1_R_ABORT_BIT :1; //bit 5
		unsigned FLG1_X_NXT_LEVEL_BIT :1; //bit 6
		unsigned FLG1_R_SAMPLE_BIT :1; //bit 7
	} FLAG1_STRUCT;

} ucFLAG1_BYTE;

extern volatile union //ucFLAG2_BYTE
{
	uchar byte;

	struct
	{
		unsigned FLG2_T3_ALARM_MCH_BIT :1; //bit 0 ;1=T3 Alarm, 0=no alarm
		unsigned FLG2_T1_ALARM_MCH_BIT :1; //bit 1 ;1=T1 Alarm, 0=no alarm
		unsigned FLG2_T2_ALARM_MCH_BIT :1; //bit 2 ;1=T2 Alarm, 0=no alarm
		unsigned FLG2_CLK_INT_BIT :1; //bit 3	;1=clk ticked, 0=not
		unsigned FLG2_X_FROM_MSG_BUFF_BIT :1; //bit 4
		unsigned FLG2_R_BUSY_BIT :1; //bit 5 ;int: 1=REC BUSY, 0=IDLE
		unsigned FLG2_R_BARKER_ODD_EVEN_BIT :1; //bit 6 ;int: 1=odd, 0=even
		unsigned FLG2_R_BITVAL_BIT :1; //bit 7 ;int:
	} FLAG2_STRUCT;

} ucFLAG2_BYTE;

//END: Flags

/*********************************************************
 *                 SP board externs
 *********************************************************/
extern struct S_Active_SP_Fields S_ActiveSP;
extern volatile uint8 g_ucRXBufferIndex;
extern uint8 g_ucRXBitsLeft;
extern volatile uint8 g_ucCOMM_Flags;
extern uint8 g_ucTXBuffer;
extern uint8 g_ucTXBitsLeft;
extern uint16 g_unCOMM_BaudRateControl;
extern uint16 g_unCOMM_BaudRateDelayControl;
extern volatile uint8 g_ucaRXBuffer[RX_BUFFER_SIZE];

/*********************************************************
 * 						Radio externs and preamble
 *********************************************************/
extern volatile uchar ucaMSG_BUFF[MAX_RESERVED_MSG_SIZE];
extern volatile ADF7020_Driver_t ADF7020_Driver;

extern const uint8 g_ucaADF7020_Preamble[ADF7020_PREAMBLE_BYTE_COUNT];
//End Radio

/*******************************************************
 * Serial Comm. externs
 *******************************************************/
extern unsigned char g_ucaUSCI_A0_RXBuffer[0x100];
extern unsigned char g_ucaUSCI_A0_RXBufferIndex;
extern unsigned char g_ucaUSCI_A1_RXBuffer[0x100];
extern unsigned char g_ucaUSCI_A1_RXBufferIndex;

/******************************************************
 *  Time keeping externs
 ******************************************************/
extern volatile ulong uslALARM_TIME;
extern volatile ulong uslCLK_TIME;
extern volatile ulong uslCLK2_TIME;
extern volatile uchar g_ucLatencyTimerState;

/////////////////////////////////////////////////////////////////////////
//! \brief This ISR handles the TACCR0 generated interrupts used for
//!  the real time clock.
//!
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////
#pragma vector=TIMER1_A0_VECTOR
__interrupt void vTIMERA1_A0_ISR(void)
{
	//Increment the primary clock time
	uslCLK_TIME++;

	//Increment the secondary clock time
	uslCLK2_TIME++;

	if (uslCLK_TIME == uslALARM_TIME)
		ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T1_ALARM_MCH_BIT = 1; //sets the alarm bit

	SUB_SEC_TIM_CTL &= ~TAIFG;
	TA1CCTL0 &= ~CCIFG;
	__bic_SR_register_on_exit(CPUOFF);
}


/////////////////////////////////////////////////////////////////////////
//! \brief ISR to handle the structure of a slot.  This allows the  
//!  WiSARD to accomplish multiple sensing actions in one slot while maintaining 
//!  the timing required for radio communication.
//!
//!
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////
#pragma vector=TIMER1_A1_VECTOR
__interrupt void vTIMER1_A1_ISR(void)
{

	switch (__even_in_range(TA1IV, 4))
	{
		case TA1IV_NONE: //no interrupt
			__no_operation();
		break;

		//Aperture buffer start
		case TA1IV_TA1CCR1:
			//Disable the interrupt and clear the interrupt flag
			TA1CCTL1 &= ~(CCIE | CCIFG);
			ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T2_ALARM_MCH_BIT = 1;
		break;

		//Aperture end
		case TA1IV_TA1CCR2:
			TA1CCTL2 &= ~(CCIE | CCIFG);
			ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T3_ALARM_MCH_BIT = 1;
			__bic_SR_register_on_exit(CPUOFF);
		break;

		case TA1IV_TA1IFG:
		break;

		default:
		break;
	}

} //END: TIMER1_A1_ISR()

/////////////////////////////////////////////////////////////////////////
//! \brief ISR
//!
//!
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
	switch (__even_in_range(TBIV, 14))
	{
		case TB0IV_NONE:
		break; // No interrupt

		case TB0IV_TB1CCR1: // CCR1

		break;

		case TB0IV_TB1CCR2: //CCR2
		break;

		case TB0IV_3: //Reserved
		break;

		case TB0IV_4://Reserved
		break;

		case TB0IV_5://Reserved
		break;

		case TB0IV_6: //Reserved
		break;

		case TB0IV_TB0IFG:
		break; // overflow

		default:
		break;
	}
//	ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T3_ALARM_MCH_BIT = 1;
//	ENDSLOT_TIM_CTL &= ~ENDSLOT_INTFLG_BIT;	//clear the interrupt
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
	ENDSLOT_TIM_CTL &= ~ENDSLOT_INTFLG_BIT; //clear the interrupt
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
//! NOTE: Traditionally an ISR is only used to changed state and not for
//! for making decisions and changing pins. However, because we are doing 
//! software UART and time was important, I wanted to avoid the latency
//! associated with moving into AND out of an ISR. For future improvements,
//! see the TODO below.
//!
//! TODO: Move the bulk of this code and decision making into the SendByte
//! routine, only have the ISR perform MCU wake up and state switching for
//! TX. The RX code has to remain as this is an asynchronus protocol and 
//! packets can come in at any time. These changes may require tweaking of 
//! the UART delay values to ensure that the space and mark timing is 
//! correct.
//!
//! NOTE: This interrupt is ONLY for handling CCR0, allowing the user to use
//! the TIMERA1 interrupt for other uses of TA CCR1, CCR2 and TAIFG
//!   \param None
//!   \return None
/////////////////////////////////////////////////////////////////////////////// 
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{

	if (g_ucCOMM_Flags & COMM_TX_BUSY)
	{
		switch (g_ucTXBitsLeft)
		{
			case 0x00:
				// If there are no bits left, then return to function call
				__bic_SR_register_on_exit(LPM0_bits);
			break;
			case 0x01:
				// Last bit is stop bit, return to idle state
				*S_ActiveSP.m_ucActiveSP_RegOut |= S_ActiveSP.m_ucActiveSP_BitTx;
				__bic_SR_register_on_exit(LPM0_bits);
			break;
			case 0x0A:
				// First bit is start bit
				*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;
			break;
			default:
				// For data bits, mask to get correct value and the shift for next time
				if (g_ucTXBuffer & 0x01)
					*S_ActiveSP.m_ucActiveSP_RegOut |= S_ActiveSP.m_ucActiveSP_BitTx;
				else
					*S_ActiveSP.m_ucActiveSP_RegOut &= ~S_ActiveSP.m_ucActiveSP_BitTx;
				g_ucTXBuffer >>= 1;
			break;
		}
		// Decrement the total bit count
		g_ucTXBitsLeft--;
	}
	if (g_ucCOMM_Flags & COMM_RX_BUSY)
	{
		switch (g_ucRXBitsLeft)
		{
			case 0x00:
				// There are no bits left, so lets reset all the values and stop timer
				TA0CTL &= ~(MC0 | MC1);
				P_RX_IE |= S_ActiveSP.m_ucActiveSP_BitRx;
				P_RX_IFG &= ~S_ActiveSP.m_ucActiveSP_BitRx;
				g_ucRXBufferIndex++;
				g_ucCOMM_Flags &= ~COMM_RX_BUSY;
				//        	__bic_SR_register_on_exit(LPM4_bits); //All Clocks and CPU etc awake now that we received a byte. (check if it's last later)
				//If it is not the last Byte, the core will put us back into LPM0, which won't stop the clocks, just the CPU
			break;
			case 0x01:
				if (P_RX_IN & S_ActiveSP.m_ucActiveSP_BitRx)
					g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
				else
					g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;
			break;
			default:
				if (P_RX_IN & S_ActiveSP.m_ucActiveSP_BitRx)
					g_ucaRXBuffer[g_ucRXBufferIndex] |= 0x80;
				else
					g_ucaRXBuffer[g_ucRXBufferIndex] &= ~0x80;
				g_ucaRXBuffer[g_ucRXBufferIndex] >>= 1;
			break;
		}
		g_ucRXBitsLeft--;
	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Timer0_A1_ISR, handles radio communication
//!
//!
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void)
{
	switch (TA0IV)
	{
		case TA0IV_NONE:
		case TA0IV_TA0CCR1:
		case TA0IV_TA0CCR2:

			// Buzzer PWM using OUTMOD not ISR
		case TA0IV_TA0CCR3:
		break;

			//Radio Comm
		case TA0IV_TA0CCR4:
			if (ADF7020_Driver.ucTxState & DATACLK_EDGE_BIT)
			{
				/* Check for end of byte */
				if (ADF7020_Driver.ucTxBitCount > 0x07)
				{
					/* Reset bit count */
					ADF7020_Driver.ucTxBitCount = 0x00;

					/* Next byte */
					ADF7020_Driver.ucTxBufferPosition++;

					/* Check for end of message */
					if (ADF7020_Driver.ucTxBufferPosition >= (ADF7020_PREAMBLE_BYTE_COUNT + ADF7020_Driver.uConfig.Registers.ulPacketSize))
					{
						/* Set the shutdown bit */
						ADF7020_Driver.ucTxState |= TX_SHUTDOWN_BIT;
					}
					else /* We are at the end of byte but not end of message */
					{
						/* Set the active byte based on position */
						if (ADF7020_Driver.ucTxBufferPosition < ADF7020_PREAMBLE_BYTE_COUNT)
						{
							ADF7020_Driver.ucTxActiveByte = g_ucaADF7020_Preamble[ADF7020_Driver.ucTxBufferPosition];
						}
						else
						{
							ADF7020_Driver.ucTxActiveByte = ADF7020_Driver.ucaTxBuffer[ADF7020_Driver.ucTxBufferPosition - ADF7020_PREAMBLE_BYTE_COUNT];
						}
					}
				}
				/* Clear the edge bit for next interrupt */
				ADF7020_Driver.ucTxState &= ~DATACLK_EDGE_BIT;
			}
			else /* We are a falling edge, not a rising */
			{
				/* Check for end of transmission */
				if (ADF7020_Driver.ucTxState & TX_SHUTDOWN_BIT)
				{
					/* Shut off the timer */
					TA0CTL &= ~(MC1 | MC0);

					/* Return to TX idle state */
					ADF7020_Driver.eRadioState = TX_IDLE;
				}

				/* Set the DATAIO line depending on the MSB in the data buffer */
				if (ADF7020_Driver.ucTxActiveByte & 0x80)
				{
					DATAIO_PIN_REG |= DATAIO_PIN_NUMBER;
				}
				else
				{
					DATAIO_PIN_REG &= ~DATAIO_PIN_NUMBER;
				}

				/* Shift the byte for next time */
				ADF7020_Driver.ucTxActiveByte <<= 1;

				/* Increment the bit count */
				ADF7020_Driver.ucTxBitCount++;

				/* The edge bit for the next interrupt */
				ADF7020_Driver.ucTxState |= DATACLK_EDGE_BIT;
			}
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Port1 ISR, handles SP board interrupts as well as user and radio
//!	interrupts
//!
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
	/* Find the source of the interrupt */
	switch (P1IV)
	{
		case P1IV_P1IFG0:
		break;

		case P1IV_P1IFG1:
		break;

		case P1IV_P1IFG2:
		break;

		case P1IV_P1IFG3:
		break;

		case P1IV_P1IFG4:
		break;

		case DATACLK_IV_NUMBER:
			/* If we get a DATACLK interrupt, it means a new bit has been output by
			 * the radio and we need to save it. This driver is configured as an MSB
			 * first driver. Thus we will always left shift the current byte FIRST
			 * and THEN write the bit to the LSB. */

			/* Increment the bit count and shit the bit */
			ADF7020_Driver.ucRxBitCount++;
			if (ADF7020_Driver.ucFirstBit)
			{
				ADF7020_Driver.ucRxBitCount--;
				ADF7020_Driver.ucFirstBit = 0;
			}

			ADF7020_Driver.ucaRxBuffer[ADF7020_Driver.ucRxBufferPosition] <<= 1;

			/* Read the incoming bit and write it to the buffer*/
			if ((DATAIO_IN_REG & DATAIO_PIN_NUMBER) == DATAIO_PIN_NUMBER)
			{
				ADF7020_Driver.ucaRxBuffer[ADF7020_Driver.ucRxBufferPosition] |= 0x01;
			}
			else
			{
				ADF7020_Driver.ucaRxBuffer[ADF7020_Driver.ucRxBufferPosition] &= ~0x01;
			}

			/* Check to see if we have received a whole byte. If we have, then
			 * increment the byte position and reset the bit count */
			if (ADF7020_Driver.ucRxBitCount >= 0x08)
			{
				ADF7020_Driver.ucRxBitCount = 0x00;
				ADF7020_Driver.ucRxBufferPosition++;
			}

			/* Check for the end of packet. If it is, then shutoff the interrupt and
			 * return to RX_IDLE */
			if (ADF7020_Driver.ucRxBufferPosition >= ADF7020_Driver.uConfig.Registers.ulPacketSize)
			{
				//Disable interrupts until ready for another message
				DATACLK_IE_REG &= ~DATACLK_PIN_NUMBER;
				INTLOCK_IE_REG &= ~INTLOCK_PIN_NUMBER;

				ADF7020_Driver.eRadioState = RX_IDLE;
				//Let the Application layer know that we have a message
				ucFLAG1_BYTE.FLAG1_STRUCT.FLG1_R_HAVE_MSG_BIT = 1;
			}

		break;

		case INTLOCK_IV_NUMBER:
			/* This means that the ADF7020 has received an incoming sync word. We
			 * need to turn on the interrupt for DATACLK to start gathering incoming
			 * data bits and turn off the INTLOCK interrupt */
			DATACLK_IE_REG |= DATACLK_PIN_NUMBER;
			INTLOCK_IE_REG &= ~INTLOCK_PIN_NUMBER;

			//To measure RX and decoding time start the timer
			LATENCY_TIMER_CTL |= g_ucLatencyTimerState;

			/* New packet, reset all write pointers and counters */
			ADF7020_Driver.ucRxBufferPosition = 0;
			ADF7020_Driver.ucRxBitCount = 0;
			ADF7020_Driver.ucFirstBit = 1;

			/* Transition the state machine */
			ADF7020_Driver.eRadioState = RX_ACTIVE;
		break;

		case P1IV_P1IFG7:
		break;

		case P1IV_NONE:
		default:
		break;
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
///////////////////////////////////////////////////////////////////////////////
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
	// If we get an interrupt from the RX pin, then we have RXed a start bit
	/* Find the source of the interrupt */
	switch (P2IV)
	{
		case P2IV_P2IFG0:
		case P2IV_P2IFG1:
		case P2IV_P2IFG2:
		case P2IV_P2IFG3:
			// Delay for half bit, this ensures we start sampling at the middle of
			// each bit
			TA0CTL &= ~(MC0 | MC1 | TAIE | TAIFG);
			TA0CTL |= TACLR;
			TA0CCTL0 &= ~CCIE;
			TA0CCR0 = g_unCOMM_BaudRateDelayControl;
			TA0CTL |= MC_1;
			while (!(TA0CTL & TAIFG))
				;
			TA0CTL &= ~TAIFG;

			// Enable timer interrupt, configure for baud rate
			TA0CTL &= ~(MC0 | MC1);
			TA0CTL |= TACLR;
			TA0CCTL0 |= CCIE;
			TA0CCR0 = g_unCOMM_BaudRateControl;

			// Disable interrupt on RX
			P_RX_IE &= ~(SP1_RX_BIT | SP2_RX_BIT | SP3_RX_BIT | SP4_RX_BIT);
			g_ucCOMM_Flags |= COMM_RX_BUSY;
			g_ucRXBitsLeft = 0x08;
			TA0CTL |= MC_1;
			P_RX_IFG &= ~(SP1_RX_BIT | SP2_RX_BIT | SP3_RX_BIT | SP4_RX_BIT);
		break;

		case P2IV_P2IFG4:
		break;

		case P2IV_P2IFG5:
		break;

		case P2IV_P2IFG6:
		break;

		case P2IV_P2IFG7:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
// vUSCI_A1_ISR()
//
// Interrupt handler for RX on USCI A1. If a character is received on
// USCI_A1, it is stored in a buffer.
//
// The buffer is only capable of holding 256 characters, after that it will
// wrap around and start overwriting at the beginning.
//
// NOTE: IT IS UP TO THE USER TO CHECK THE BUFFER FOR INPUTS, EITHER HERE OR
//       IN THE CODE ELSE WHERE
///////////////////////////////////////////////////////////////////////////////
#pragma vector=USCI_A1_VECTOR
__interrupt void vUSCI_A1_ISR(void)
{

	// Echo back to command line so user see's input
	switch (UCA1RXBUF)
	{
		case '\r':
			UCA1TXBUF = 0x0D; // \r
			UCA1TXBUF = 0x0A; // \n
			g_ucaUSCI_A1_RXBuffer[g_ucaUSCI_A1_RXBufferIndex++] = 0x00;
			__bic_SR_register_on_exit(LPM0_bits);
		break;

		default:
			UCA1TXBUF = UCA1RXBUF;
			g_ucaUSCI_A1_RXBuffer[g_ucaUSCI_A1_RXBufferIndex++] = UCA1RXBUF;
		break;
	}

	if (g_ucaUSCI_A1_RXBufferIndex > 0xFF)
		g_ucaUSCI_A1_RXBufferIndex = 0x00;
	__bic_SR_register_on_exit(CPUOFF);
	// NOTE: The USCA0RXIFG flag is automatically cleared when UCA0RXBUF is read,
	//       no need to clear in manually.
}

//////////////////  Unused ISRs ////////////////////////
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
}

#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
}

#pragma vector=USCI_B3_VECTOR
__interrupt void USCI_B3_ISR(void)
{
}

#pragma vector=USCI_A3_VECTOR
__interrupt void USCI_A3_ISR(void)
{
}

#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
}

#pragma vector=USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
}

#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
}

#pragma vector=UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
}

#pragma vector=SYSNMI_VECTOR
__interrupt void SYSNMI_ISR(void)
{
}

//! @} 
