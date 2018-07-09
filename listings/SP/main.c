//*****************************************************************************
// main.c
//
// By: Christopher Porter
//     Wireless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//
// SP-SL Main
//*****************************************************************************

#include <msp430x23x.h>
#include "core/core.h"
#include "hal/adc12.h"
#include "Light/light.h"

//! \brief Union containing the structure of the MCU hardware
//!
//! This union is used to describe the active hardware peripherals on the MCU.
volatile union
{
	unsigned char byte;
	struct
	  {
		unsigned ADC12:		1;	//!< 12-bit analog to digital converter
		unsigned ADC10:		1;  //!< 10-bit analog to digital converter
		unsigned TIMER_A:	1;  //!< Timer A
		unsigned TIMER_B:	1;  //!< Timer B
		unsigned USI:		1;  //!< Universal Serial Interface
		unsigned USCI:		1;  //!< Universal Serial Communication Interface
		unsigned OPAMP:		1;  //!< Operational Amplifier
		unsigned COMP:		1;  //!< Comparator
		unsigned DAC:		1;	//!< Digital to analog converter
		unsigned SD:		1;	//!< Sigma-Delta Converter
	  }stHardware_struct;

}uHardware_byte;

//! \brief Determines the amount of ADC readings needed before an average is taken
//!
uint16 g_uiAveCounter;

// dummy variable in light testing likely to be removed
uint16 g_uiDummDumm;

///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Test Function is called
//!
//!   Sends a UART message what the command was.
//!
//!   \param pointer to an array where data will be placed
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_Test(uint16 * arr)
{

  //Read the sensor and store the data
  *(arr) = 0x1234;

  *(arr+1) = 0x5678;

  *(arr+2) = 0xDEAD;

  *(arr+3) = 0xBEEF;

	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 1 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL1(uint16 * arr)
{

  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
  return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 2 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL2(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 3 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL12(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 4 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL3(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  *(arr) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 5 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL3_SL1(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 6 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL3_SL2(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  *(arr) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 7 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL3_SL12(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+2) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);


  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 8 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL4(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();

	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer 9 is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL4_SL1(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensors and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer A is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL4_SL2(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer B is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL4_SL12(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+2) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer C is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL43(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensors and store the data
  *(arr) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer D is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL43_SL1(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+2) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer E is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL43_SL2(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+2) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief Handle for when Transducer F is called
//!
//!
//!   \param g_unaCoreData.
//!		 A pointer at the data that came from the CP board and where the result
//!      is to be written.
//!
//!   \return 1: success, 0: failure
///////////////////////////////////////////////////////////////////////////////
uint16 main_SL43_SL12(uint16 * arr)
{
  //Initialize the light sensor hardware
  vLight_Init();

  //Read the sensor and store the data
  *(arr) = unLIGHT_ReadChannel_1(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+1) = unLIGHT_ReadChannel_2(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+2) = unLIGHT_ReadChannel_3(&g_uiAveCounter, &g_uiDummDumm);

  *(arr+3) = unLIGHT_ReadChannel_4(&g_uiAveCounter, &g_uiDummDumm);

  //Shut down the light sensor hardware
  vLight_Shutdown();
	return 1;
}


///////////////////////////////////////////////////////////////////////////////
//!   \brief The main file for the SP-CM-STM SP Board
//!
//!   Initializes Core, and flags
//!	  Then runs vCORE_Run()
//!
//!   \param none
//!
//!   \return never
///////////////////////////////////////////////////////////////////////////////
void main(void)
{

  //Initialize the core
  vCORE_Initilize();

  //Initialize the hardware flags
  uHardware_byte.byte = 0x00;

  //Set the number of readings to 16
  g_uiAveCounter = 0x10;

  P5DIR |= BIT2;
  P5OUT |= BIT2;

  // Run the device
  vCORE_Run();
}

