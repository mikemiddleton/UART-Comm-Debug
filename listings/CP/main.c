/* 1.8
 *
 * Added the finished radio driver
 * Replaced old SP communication module with updated version
 * Started work on new WiSARDnet comm protocol to be continued in next revision
 * */

/**************************  MAIN.C  ******************************************
 *
 * Main routine for wizard.
 *
 ******************************************************************************/

#define THIS_MAIN_VERSION 0x04
#define THIS_SUB_VERSION  0x00

#define THIS_VERSION_SUBCODE	' '		//debug codes
//#define THIS_VERSION_SUBCODE	'a'		//debug codes
//#define THIS_VERSION_SUBCODE	'b'		//debug codes
//#define THIS_VERSION_SUBCODE	'c'		//debug codes
//#define THIS_VERSION_SUBCODE	'd'		//debug codes

/*lint -e526 *//* function not defined */
/*lint -e563 *//* label not referencecd */
/*lint -e657 *//* Unusual (nonportable) anonymous struct or union */
/*lint -e714 *//* symbol not referenced */
/*lint -e716 *//* while(1) ... */
/*lint -e750 *//* local macro not referenced */
/*lint -e754 *//* local structure member not referenced */
/*lint -e755 *//* global macro not referenced */
/*lint -e757 *//* global declarator not referenced */
/*lint -e758 *//* global union not referenced */
/*lint -e768 *//* global struct member not referenced */

#include <msp430x54x.h>		//register and ram definition file
#include "std.h"			//std defines
#include "diag.h"			//Diagnostic package header
#include "main.h"			//main defines
#include "delay.h"			//approx delay routine
#include "misc.h"			//homeless routines
#include "rts.h"			//Real Time Sched routines
#include "rand.h"			//random number generator
#include "action.h"			//event action module
#include "key.h"			//keyboard handler
#include "serial.h"			//serial port
#include "time.h"			//System Time routines
#include "daytime.h"		//Daytime routines
#include "button.h" 		//Button routines
#include "report.h"			//Reporting routines
#include "comm.h"			//msg helper routines
#include "gid.h"			//Group ID routines
#include "stbl.h"			//Schedule table routines
#include "sensor.h"			//sensor routines
#include "pick.h"			//Trigger routines
#include "MODOPT.h"			//Modify Options routines
#include "LNKBLK.h"			//radio link handler routines
#include "hal/config.h"		//system configuration definitions
#include "drivers/SP.h"		//SP board control
#include "drivers/fram.h"	//FRAM handler routines
#include "drivers/flash.h" 	//FLASH memory handler routines
#include "drivers/buz.h"	//Buzzer routines
#include "drivers/led.h"	//on board LED definitions
#include "drivers/adf7020.h"	//ADF7020 definitions
#ifdef DEBUG_DISPATCH_TIME
#include "t0.h"			//Timer T0 routines
#endif

#include "crc.h"			//CRC calculator routine
#ifdef ESPORT_ENABLED				//defined in diag.h
#include "esport.h"			//external serial port
#endif

/****************************  DEFINES  **************************************/

//#define KILL_ALL_CHECKS TRUE
#ifndef KILL_ALL_CHECKS
#define KILL_ALL_CHECKS FALSE
#endif

//#define DEBUG_DISPATCH_TIME 1

/**********************  VOLATILE  GLOBALS  **********************************/

volatile uint8 ucaMSG_BUFF[MAX_RESERVED_MSG_SIZE];
volatile ADF7020_Driver_t ADF7020_Driver;

volatile uint8 ucaX0FLD[MSG_XFLDSIZE];
volatile uint8 ucaX1FLD[MSG_XFLDSIZE];

//Time keeping variables

volatile ulong uslALARM_TIME;
volatile ulong uslCLK_TIME;
volatile ulong uslCLK2_TIME;

volatile uint8 ucBUTTON_COUNT[4];

#ifdef INC_ESPORT						//defined on Cmd line
volatile uint8 ucESPORT_TX_Byte; //transmit byte ram
volatile uint8 ucESPORT_RX_Byte;//receive byte ram
volatile uint8 ucESPORT_BitCounter;//
volatile uint8 ucESPORT_TimeCounter;//
#endif /* END: INC_ESPORT */

volatile uint8 ucaBigMinuend[6];
volatile uint8 ucaBigSubtrahend[6];
volatile uint8 ucaBigDiff[6];

volatile uint8 ucRAND_NUM[RAND_NUM_SIZE];

volatile uint8 ucaCommQ[COMM_Q_SIZE];

volatile uint8 ucB3TEMP; //used in ONEWIREA & RANDA

volatile uint8 ucINT_TEMP1;
volatile uint8 ucINT_TEMP2;
volatile uint8 ucINT_TEMP3;
volatile uint8 ucINT_TEMP4;

volatile uint8 ucQonIdx_LUL;
volatile uint8 ucQoffIdx_LUL;
volatile uint8 ucQcount;

volatile union //ucFLAG0_BYTE
{
	uint8 byte;
	struct
	{
		unsigned FLG0_BIGSUB_CARRY_BIT :1; //bit 0 ;1=CARRY, 0=NO-CARRY
		unsigned FLG0_BIGSUB_6_BYTE_Z_BIT :1; //bit 1 ;1=all diff 0, 0=otherwise
		unsigned FLG0_BIGSUB_TOP_4_BYTE_Z_BIT :1; //bit 2 ;1=top 4 bytes 0, 0=otherwise
		unsigned FLG0_REDIRECT_COMM_TO_ESPORT_BIT :1; //bit 3 ;1=REDIRECT, 0=COMM1
		unsigned FLG0_RESET_ALL_TIME_BIT :1; //bit 4 ;1=do time  reset, 0=dont
		//SET:	when RDC4 gets finds first
		//		SOM2.
		//		or
		//		In a Hub when it is reset.
		//
		//CLR: 	when vMAIN_computeDispatchTiming()
		//		runs next.
		unsigned FLG0_SERIAL_BINARY_MODE_BIT :1; //bit 5 1=binary mode, 0=text mode
		unsigned FLG0_HAVE_WIZ_GROUP_TIME_BIT :1; //bit 6 1=Wizard group time has
		//        been aquired from a DC4
		//      0=We are using startup time
		unsigned FLG0_ECLK_OFFLINE_BIT :1; //bit 7 1=ECLK is not being used
	//      0=ECLK is being used
	} FLAG0_STRUCT;
} ucFLAG0_BYTE;

#define FLAG0_INIT_VAL	    0x00		//0000 0000
volatile union //ucFLAG1_BYTE
{
	uint8 byte;
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

#define FLAG1_INIT_VAL	  0x00  	//0000 0000
//
//  	 volatile union							//ucFLAG2_BYTE
//  	  	{
//  	  	uint8 byte;
//  	  	struct
//  	  	 {
//  	  	 unsigned FLG2_T3_ALARM_MCH_BIT:1;		//bit 0 ;1=T3 Alarm, 0=no alarm
//  	  	 unsigned FLG2_T1_ALARM_MCH_BIT:1;		//bit 1 ;1=T1 Alarm, 0=no alarm
//  	  	 unsigned FLG2_BUTTON_INT_BIT:1;		//bit 2 ;1=XMIT, 0=RECEIVE
//  	  	 unsigned FLG2_CLK_INT_BIT:1;			//bit 3	;1=clk ticked, 0=not
//  	  	 unsigned FLG2_X_FROM_MSG_BUFF_BIT:1;	//bit 4
//  	  	 unsigned FLG2_R_BUSY_BIT:1;			//bit 5 ;int: 1=REC BUSY, 0=IDLE
//  	  	 unsigned FLG2_R_BARKER_ODD_EVEN_BIT:1;	//bit 6 ;int: 1=odd, 0=even
//  	  	 unsigned FLG2_R_BITVAL_BIT:1;			//bit 7 ;int:
//  	  	 }FLAG2_STRUCT;
//
//  	  	}ucFLAG2_BYTE;
//
//  	#define FLAG2_INIT_VAL	0x00	//00000000
//
//
//  	 volatile union
//  	  	{
//  	  	  uint8 byte;
//  	  	 struct
//  	  	  {
//  	  	    unsigned FLG3_RADIO_ON_BIT:1;
//  	  	    unsigned FLG3_RADIO_MODE_BIT:1;
//  	  	    unsigned FLG3_RADIO_PROGRAMMED:1;
//  	  	    unsigned FLG3_UNUSED_BIT3:1;
//  	  	    unsigned FLG3_UNUSED_BIT4:1;
//  	  	    unsigned FLG3_UNUSED_BIT5:1;
//  	  	    unsigned FLG3_UNUSED_BIT6:1;
//  	  	    unsigned FLG3_UNUSED_BIT7:1;
//  	  	  }FLAG3_STRUCT;
//  	  	}ucFLAG3_BYTE;
//

volatile union //ucFLAG2_BYTE
{
	uint8 byte;
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

#define FLAG2_INIT_VAL	0x00	//00000000
volatile union
{
	uint8 byte;
	struct
	{
		unsigned FLG3_RADIO_ON_BIT :1;
		unsigned FLG3_RADIO_MODE_BIT :1;
		unsigned FLG3_RADIO_PROGRAMMED :1;
		unsigned FLG2_BUTTON_INT_BIT :1; //bit 2 ;1=XMIT, 0=RECEIVE
		unsigned FLG3_UNUSED_BIT4 :1;
		unsigned FLG3_UNUSED_BIT5 :1;
		unsigned FLG3_UNUSED_BIT6 :1;
		unsigned FLG3_UNUSED_BIT7 :1;
	} FLAG3_STRUCT;
} ucFLAG3_BYTE;

volatile union //ucGLOB_diagFlagByte1
{
	uint8 byte;
	struct
	{
		unsigned DIAG_mute_bit :1; //bit 0 ;1=MUTE, 0=SOUND
		unsigned DIAG_test_repeat_looping_bit :1; //bit 1 ;1=LOOPING, 0=NOT LOOPING
		unsigned DIAG_only_a_single_test_bit :1; //bit 2	;1=ONLY A SINGLE, 0=ALL
		unsigned DIAG_only_a_single_group_bit :1; //bit 3 ;1=ONLY A GROUP, 0=ALL
		unsigned DIAG_exit_out_to_top_bit :1; //bit 4 ;1=EXIT OUT, 0=NO EXIT OUT
		unsigned DIAG_exit_out_to_prev_test_bit :1; //bit 5 ;1=GOTO PREV, 0=NO GOTO PREV
		unsigned DIAG_exit_out_to_next_test_bit :1; //bit 6 ;1=GOTO NEXT, 0=NO GOTO NEXT
		unsigned DIAG_quit_out_to_return_bit :1; //bit 7 ;1=TOTO RETURN, 0=NO QUIT
	} diagFlagByte1_STRUCT;
} ucGLOB_diagFlagByte1;

#define DIAG_FLAG_BYTE_1_INIT_VAL 0x00	//00000000
volatile union //ucGLOB_diagFlagByte2
{
	uint8 byte;
	struct
	{
		unsigned DIAG_tell_whats_running_bit :1; //bit 0 ;1=TELL, 0=NO TELL
		unsigned DIAG_change_looping_sts_bit :1; //bit 1 ;1=CHANGE LOOPING, 0=NO CHANGE
		unsigned DIAG_halt_on_error_bit :1; //bit 2 ;1=halt, 0=no halt
		unsigned DIAG_partially_automated_run_bit :1; //bit 3	;1=an 'A' was hit, 0=no 'A' hit
		unsigned DIAG_not_used_4_bit :1; //bit 4 ;
		unsigned DIAG_not_used_5_bit :1; //bit 5 ;
		unsigned DIAG_not_used_6_bit :1; //bit 6 ;
		unsigned DIAG_not_used_7_bit :1; //bit 7 ;
	} diagFlagByte2_STRUCT;
} ucGLOB_diagFlagByte2;

#define DIAG_FLAG_BYTE_2_INIT_VAL 0x00	//00000000
/*---------  END of VOLATILES  -----------------------------*/

/***********************  NON-VOLATILE GLOBALS  *****************************/

uint8 ucGLOB_myLevel; //senders level +1

long lGLOB_initialStartupTime; //Time used to compute uptime

long lGLOB_lastAwakeTime; //Nearest thing to slot time
long lGLOB_opUpTimeInSec; //Nearest thing to cur operational up time
long lGLOB_lastAwakeLinearSlot; //Nearest thing to cur linear slot a
long lGLOB_lastAwakeFrame; //Nearest thing to cur frame
uint8 ucGLOB_lastAwakeSlot; //Nearest thing to cur slot
uint8 ucGLOB_lastAwakeNSTtblNum; //Nearest thing to cur NST tbl
uint8 ucGLOB_lastAwakeStblIdx; //Nearest thing to cur sched idx

long lGLOB_lastScheduledFrame; //last scheduled frame number
uint8 ucGLOB_lastScheduledSchedSlot; //slot for sched in last scheduled frame

uint8 ucGLOB_SDC4StblIdx; //Sched tbl idx for SDC4  function
uint8 ucGLOB_RDC4StblIdx; //Sched tbl idx for RDC4  function
uint8 ucGLOB_TC12StblIdx; //DEBUG: Sched tbl idx for TC12
uint8 ucGLOB_TC34StblIdx; //DEBUG: Sched tbl idx for TC34
uint8 ucGLOB_LT12StblIdx; //DEBUG: Sched tbl idx for LT12
uint8 ucGLOB_LT34StblIdx; //DEBUG: Sched tbl idx for LT34

long lGLOB_OpMode0_inSec; //Start of Opmode

// ulong ulGLOB_sramTblPtr_LUL;		//grows up,  init = end to sched tbl section
// uint  uiGLOB_sramMsgCount;

usl uslGLOB_sramQon_NFL;
usl uslGLOB_sramQoff;
uint uiGLOB_sramQcnt;

uint8 ucGLOB_curMsgSeqNum;

int iGLOB_Hr0_to_SysTim0_inSec; //dist from SysTim0 to Hr0
//uint uiGLOB_WrldStartHr;			//Starting hour in WorldTime

uint uiGLOB_grpID; //group ID for this group
uint8 ucGLOB_StblIdx_NFL; //next free loc in the sched tables

uint8 ucGLOB_lineCharPosition; //line position for computing tabulation

int iGLOB_completeSysLFactor; //entire Signed LFactor quantity.

// uint8 ucGLOB_msgSysLFactor;		//Message (byte size) unsigned Load Factor
uint8 ucGLOB_msgSysLnkReq; //Msg Linkup Req

uint8 ucGLOB_radioChannel; //Current radio channel number (0 - 127)

uint8 ucSP01_IDx;
uint8 ucSP23_IDx;

/*----------  DEBUG RAM LOCATIONS  ---------*/
uint8 ucGLOB_testByte; //counts thermocouple onewire dropouts
uint8 ucGLOB_testByte2; //counts button return type 2 errors

uint uiGLOB_bad_flash_CRC_count; //count of bad CRC's on flash msgs

uint uiGLOB_lostROM2connections; //total lost ROM2's, Zro on startup
uint uiGLOB_lostSOM2connections; //total lost SOM2's, Zro on startup

uint uiGLOB_ROM2attempts; //total ROM2 attempts, Zro on startup
uint uiGLOB_SOM2attempts; //total SOM2 attempts, Zro on startup

uint uiGLOB_TotalSDC4trys; //total SDC4 attempts, Zro on startup
uint uiGLOB_TotalRTJ_attempts; //total RDC4 attempts, Zro on startup

union //ucGLOB_debugBits1
{
	uint8 byte;
	struct
	{
		unsigned DBG_MaxIdxWriteToNST :1; //bit 0 ;set if err, clr'd after reporting
		unsigned DBG_MaxIdxReadFromNST :1; //bit 1 ;;set if err, clr'd after reporting
		unsigned DBG_notUsed2 :1; //bit 2 ;
		unsigned DBG_notUsed3 :1; //bit 3	;
		unsigned DBG_notUsed4 :1; //bit 4 ;
		unsigned DBG_notUsed5 :1; //bit 5 ;
		unsigned DBG_notUsed6 :1; //bit 6 ;
		unsigned DBG_notUsed7 :1; //bit 7 ;
	} debugBits1_STRUCT;
} ucGLOB_debugBits1;

#define DEBUG_BITS_1_INIT_VAL 0x00	//00000000
//Structure that defines the role and the identity of the WiSARD
struct Role G3Role;

/* RAM COPY OF FRAM OPTION BIT ARRAY */
uint8 ucaGLOB_optionBytes[OPTION_BYTE_COUNT];

/******************************  DECLARATIONS  *******************************/

void vMAIN_computeDispatchTiming(void);

#ifdef RUN_NOTHING
static void vMAIN_run_nothing(
		void
);
#endif

void vMAIN_showCompilerAnomalies(const char *cMsg);

/*******************************  CODE  **************************************/
uint uiTime;
/*******************************  MAIN  *************************************
 *
 *
 *
 *
 ******************************************************************************/

void main(void)

{
	//Halt the dog
	WDTCTL = WDTPW + WDTHOLD;

	/**************************************************************************
	 *
	 * Initialize the Ports, Clock, Serial Communication, and GIE
	 *
	 *************************************************************************/

	// SET INITIAL VALUE ON PORTS
	vConfig_InitializePorts();

	//Initialize clocks MCLK=16MHz SMCLK=4MHz
	vUCS_InitializeClock();

	// enable global interrupts
	__bis_SR_register(GIE);


	/* *****************************************************************************
	 *
	 * Initialize the SP boards and set the role accordingly
	 *
	 * ****************************************************************************/
		vSERIAL_init();
		vSERIAL_rom_sout(" SPstart\r\n");

//	while(1);

		//Start up the SP driver
		ucSP_Init();

//		vSP_SetRole();

		// configure trigger
		P3DIR |= BIT4;

		//Wakes up an SP board and allows printing status to the terminal
		//ucSP_WakeUp(uint8 ucSPNumber, uint8 ucPrint);
		ucSP_Start(0x02, 0x42);

		// Program the SP in location 1
		P_SP1_DIR &= ~(SP1_RST_BIT | SP1_TCK_BIT);
		P_SP1_OUT |= (SP1_EN_BIT | SP1_TX_BIT);

		while(TRUE)
		{
			vSP_Request128BitsData(0x02);
			vSERIAL_rom_sout(" Sending Command\r\n");
			ucSP_SendCommand(0x02, TRANSDUCER_0, 0xA, 0xA, 0x0, 0x0);
			vSERIAL_rom_sout(" Waiting\r\n\n");
			__delay_cycles(30000000);
			vSP_Request128BitsData(0x02);

		}

} // End main

/***********************  vMAIN_showVersionNum()  *****************************
 *
 *
 *
 *
 ******************************************************************************/

void vMAIN_showVersionNum(void)
{

	vSERIAL_bout('V');
	vSERIAL_HB4out(THIS_MAIN_VERSION);
	vSERIAL_bout('.');
	vSERIAL_HB8out(THIS_SUB_VERSION);
#if (THIS_VERSION_SUBCODE != ' ')
	vSERIAL_bout(THIS_VERSION_SUBCODE);
#endif

	return;

}/* END: vMAIN_showVersionNum() */

/************************  vMAIN_printIntro()  ************************************
 *
 * Print out our introduction message
 *
 ******************************************************************************/

void vMAIN_printIntro(void)
{
	uint8 ucECLKsts;

	/* ISSUE THE INTRO MESSAGE */
	vSERIAL_crlf();
	vDAYTIME_convertSysTimeToShowDateAndTime(TEXT_FORM);

	vSERIAL_rom_sout("\r\nWiz ");
	vSERIAL_rom_sout("("RUN_CODE_NAME") ");
	vSERIAL_bout(':');
	vL2FRAM_showSysID();
	vSERIAL_bout(' ');
	vMAIN_showVersionNum();
	vSERIAL_bout(' ');

	vMODOPT_showCurIdentity(); //shows role and SP types attached
	if (ucMODOPT_readSingleRamOptionBit(OPTPAIR_SPS_ARE_ATTCHD))
	{
		vSP_GetSPMsgVersions();
	}/* END: if(OPTPAIR_SPS_ARE_ATTCHD) */
	else
	{
		vSERIAL_rom_sout("SP-OFF)");
	}/* END: else (OPTPAIR_SPS_ARE_ATTCHD) */

	vSERIAL_crlf();
	/* TELL ABOUT THE FRAM */
	vSERIAL_rom_sout("(FRAM-");

	if (ucMODOPT_readSingleRamOptionBit(OPTPAIR_CHK_FOR_FRAM_ON_STUP))
	{

		uint uiVal;

		uiVal = uiL2FRAM_get_version_num();
		vSERIAL_rom_sout("V");
		vSERIAL_UIV8out((uint8) (uiVal >> 8)); //Version Num
		vSERIAL_bout('.');
		vSERIAL_HB8out((uint8) (uiVal & 0xFF)); //Sub Version Num
		vSERIAL_bout(')');
	}
	else
	{
		vSERIAL_rom_sout("OFF)");

	}/* END: else(OPTPAIR_CHK_FOR_FRAM_ON_STUP) */

	vSERIAL_rom_sout("(ECLK-");
	ucECLKsts = ucTIME_getECLKsts(YES_SKP_ECLK_RAM_FLAG);
	switch (ucECLKsts)
	{
		case 0:
		case 1:
			vSERIAL_rom_sout("NONE)");
		break;

		case 2:
			vSERIAL_rom_sout("NoTick)");
		break;

		case 3:
			vSERIAL_rom_sout("OK)");
		break;

		default:
			vSERIAL_rom_sout("???");
			vSERIAL_HB8out(ucECLKsts);
			vSERIAL_rom_sout(")");
		break;

	}/* END: switch() */

	vSERIAL_crlf();

	/* SHOW MESSAGE COUNT */
	vSERIAL_rom_sout("M"); //SRAM msg count
	vSERIAL_UIV16out(uiL2SRAM_getMsgCount());

	/* SHOW FLASH MESSAGE COUNT */
	vSERIAL_rom_sout(" F");
	vSERIAL_UIV24out((usl) lL2FRAM_getFlashUnreadMsgCount()); //lint !e13

	vSERIAL_crlf();
	vSERIAL_crlf();
	return;

} /* END: vMAIN_printIntro() */

/******************  vMAIN_computeDispatchTiming()  **************************
 *
 * This routine computes the dispatcher timing and then returns.
 *
 * Many of the time keeping responsibilities have been moved to the dispatcher
 * due to the structure of the slots in the GIII WiSARD.  Therefore some of
 * the alarm setting functions have been removed from this function.  Additionally,
 * due to the fact that the slots are now 1 second long the lThisSlotRemainder is
 * no longer kept in seconds it is kept in ticks from TA1R.
 *
 ******************************************************************************/

void vMAIN_computeDispatchTiming(void)
{
	long lThisTime;
	long lThisSlotEndTime;
	long lOpUpTimeInSec;
	long lThisLinearSlot;
	uint uiThisSlotRemainder;
	long lThisFrameNum;
	uint8 ucThisSlotNum;
	long lii;

#if 0
	vSERIAL_rom_sout("D\r\n");
#endif

	/* CHECK IF WE HAVE HAD A TIME RESET */
	if (ucFLAG0_BYTE.FLAG0_STRUCT.FLG0_RESET_ALL_TIME_BIT)
	{
#if 1
		vSERIAL_rom_sout("TimSet\r\n");
#endif

		lGLOB_lastAwakeTime = lTIME_getSysTimeAsLong();
		lOpUpTimeInSec = lGLOB_lastAwakeTime - lGLOB_OpMode0_inSec;
		lGLOB_lastAwakeLinearSlot = lOpUpTimeInSec / SECS_PER_SLOT_L;
		lThisSlotEndTime = ((lGLOB_lastAwakeLinearSlot + 1) * SECS_PER_SLOT_L) + lGLOB_OpMode0_inSec;
		uiThisSlotRemainder = uiTIME_getSubSecAsUint();
		lGLOB_lastAwakeFrame = lGLOB_lastAwakeLinearSlot / SLOTS_PER_FRAME_I;
		ucGLOB_lastAwakeSlot = (uint8) (lGLOB_lastAwakeLinearSlot % SLOTS_PER_FRAME_I);
		ucGLOB_lastAwakeNSTtblNum = (uint8) (lGLOB_lastAwakeFrame % 2);
		ucGLOB_lastAwakeStblIdx = ucRTS_getNSTentry(ucGLOB_lastAwakeNSTtblNum, ucGLOB_lastAwakeSlot);
		lGLOB_lastScheduledFrame = lGLOB_lastAwakeFrame; //= this frame

#if 0
		vSERIAL_rom_sout("C:FindNSTslot\r\n");
#endif

		ucGLOB_lastScheduledSchedSlot = 0x3B;
//		ucGLOB_lastScheduledSchedSlot = ucRTS_findNSTslotNumOfSched(ucGLOB_lastAwakeNSTtblNum);

#if 0
		vSERIAL_rom_sout("R:FindNSTslot\r\n");
#endif

		/* IF THE SCHED HAS NOT ALREADY RUN IN THIS TIME THEN FORCE IT TO BE RUN */
		if (ucGLOB_lastAwakeSlot > ucGLOB_lastScheduledSchedSlot)
		{
#if 1
			vSERIAL_rom_sout("C:ForceNxtNSTsched\r\n");
#endif

			vRTS_scheduleNSTtbl(lGLOB_lastAwakeFrame + 1L); //sched next NST

#if 1
			vSERIAL_rom_sout("R:ForceNxtNSTsched\r\n");
#endif
		}

		/* CLR THE RESET TIME BIT */
		ucFLAG0_BYTE.FLAG0_STRUCT.FLG0_RESET_ALL_TIME_BIT = 0; //shut flag off

#if 0
		vSERIAL_rom_sout("TimSet\r\n");
#endif

	} //END: IF THERE WAS A RESET

	/* WAIT TIL TIME IS POSITIVE */
	if (lGLOB_lastAwakeTime > lTIME_getSysTimeAsLong())
	{
		vSERIAL_rom_sout("WaitingForTimePositive\r\n");
		while (lGLOB_lastAwakeTime > lTIME_getSysTimeAsLong())
			; //lint !e722
	} //END: WAIT TIL TIME IS POSITIVE

	/* COMPUTE THE OPERATIONAL TIME IN LINEAR SLOTS */
	lThisTime = lTIME_getSysTimeAsLong();
	lOpUpTimeInSec = lThisTime - lGLOB_OpMode0_inSec;
	lThisLinearSlot = lOpUpTimeInSec / SECS_PER_SLOT_L;
	lThisSlotEndTime = ((lThisLinearSlot + 1) * SECS_PER_SLOT_L) + lGLOB_OpMode0_inSec;
	uiThisSlotRemainder = uiTIME_getSubSecAsUint();
	lThisFrameNum = lThisLinearSlot / SLOTS_PER_FRAME_I; //nxt frame num
	ucThisSlotNum = (uint8) (lThisLinearSlot % SLOTS_PER_FRAME_I);

	/* IF STILL IN THE LAST SLOT -- WAIT FOR NEXT SLOT */
	if (lThisLinearSlot == lGLOB_lastAwakeLinearSlot)
	{
		/* SLEEP IF POSSIBLE */
		if (uiThisSlotRemainder > 0x4000)
		{
			if (ucTIME_setT3AlarmToSecMinus200ms(lThisSlotEndTime) == 0)
			{
#if 0
				vSERIAL_rom_sout("E:ShortSlp\r\n");
#endif

				ucMISC_sleep_until_button_or_clk(SLEEP_MODE); //lint !e534

#if 0
				vSERIAL_rom_sout("X:ShortSlp\r\n");
#endif

			}
		}
		else
		{

			/* WAIT OUT THE REST HERE */
#if 0
			vSERIAL_rom_sout("E:WhileWait\r\n");
#endif

			while (lThisSlotEndTime > lTIME_getSysTimeAsLong())
				; //lint !e722

#if 0
			vSERIAL_rom_sout("X:WhileWait\r\n");
#endif

		}

	}/* END: if(lThisLinearSlot == lGLOB_lastAwakeLinearSlot) */

	/* IF ITS IN THE LAST_SLOT+1 -- GO START IT */
	if (lThisLinearSlot == lGLOB_lastAwakeLinearSlot + 1L)
		goto Update_and_leave;

	/*------ IF WE ARE HERE THEN TIME SKIPPED FORWARD MORE THAN A SLOT -------*/

	/* IF WE SKIPPED FRAMES -- CATCH UP THE SCHEDULED FRAMES */
	if (lThisFrameNum > lGLOB_lastScheduledFrame)
	{
#if 1
		vSERIAL_rom_sout("FRAMEskpFrom ");
		vSERIAL_HBV32out((ulong) lGLOB_lastScheduledFrame);
		vSERIAL_rom_sout(" to ");
		vSERIAL_HBV32out((ulong) lThisFrameNum);
		vSERIAL_crlf();
#endif

		/* SCHEDULER DID NOT RUN FOR THIS FRAME -- CATCH IT UP */
		for (lii = lGLOB_lastScheduledFrame + 1; lii <= lThisFrameNum; lii++)
		{
			vRTS_scheduleNSTtbl(lii);

		}/* END: for() */

		/* CHECK IF THE SCHED SLOT FOR THIS FRAME HAS BEEN PASSED */
		if (ucThisSlotNum > ucGLOB_lastScheduledSchedSlot)
		{
#if 1
			vSERIAL_rom_sout("Frame#");
			vSERIAL_HBV32out((ulong) lThisFrameNum);
			vSERIAL_rom_sout(" Frame&SlotSkpOverSchedFrom ");
			vSERIAL_HB8out(ucGLOB_lastScheduledSchedSlot);
			vSERIAL_rom_sout(" to ");
			vSERIAL_HB8out(ucThisSlotNum);
			vSERIAL_crlf();
#endif
			/* YES--ROLL ONE MORE */
			vRTS_scheduleNSTtbl(lThisFrameNum + 1L);
		}
	}
	else
	{
		/* CHECK IF THE SCHED SLOT FOR THIS FRAME HAS BEEN PASSED */
		if ((ucGLOB_lastAwakeSlot < ucGLOB_lastScheduledSchedSlot) && (ucThisSlotNum > ucGLOB_lastScheduledSchedSlot))
		{
#if 1
			vSERIAL_rom_sout("Frame#");
			vSERIAL_HBV32out((ulong) lThisFrameNum);
			vSERIAL_rom_sout(" SlotOnlySkpFrm ");
			vSERIAL_HB8out(ucGLOB_lastAwakeSlot);
			vSERIAL_rom_sout(" to ");
			vSERIAL_HB8out(ucThisSlotNum);
			vSERIAL_rom_sout(" skpingSchedSlotAt ");
			vSERIAL_HB8out(ucGLOB_lastScheduledSchedSlot);
			vSERIAL_crlf();
#endif
			/* YES--ROLL ONE MORE */
			vRTS_scheduleNSTtbl(lThisFrameNum + 1L);
		}
	}

	/* WAIT FOR A SLOT START */
	/* SLEEP IF POSSIBLE */
	if (uiThisSlotRemainder > 0)
	{
		if (!ucTIME_setT3AlarmToSecMinus200ms(lThisSlotEndTime))
		{
			ucMISC_sleep_until_button_or_clk(SLEEP_MODE); //lint !e534
		}
	}

	/* FINISH WAITING HERE */
	while (lThisSlotEndTime > lTIME_getSysTimeAsLong())
		; //lint !e722

	Update_and_leave:

	/* WE ARE CAUGHT UP -- SO UPDATE THE CURRENT FRAME AND SLOT */
	lGLOB_lastAwakeTime = lTIME_getSysTimeAsLong();
	lOpUpTimeInSec = lGLOB_lastAwakeTime - lGLOB_OpMode0_inSec;
	lGLOB_lastAwakeLinearSlot = lOpUpTimeInSec / SECS_PER_SLOT_L;
	lThisSlotEndTime = ((lGLOB_lastAwakeLinearSlot + 1) * SECS_PER_SLOT_L) + lGLOB_OpMode0_inSec;
	uiThisSlotRemainder = uiTIME_getSubSecAsUint();
	lGLOB_lastAwakeFrame = lGLOB_lastAwakeLinearSlot / SLOTS_PER_FRAME_I;
	ucGLOB_lastAwakeSlot = (uint8) (lGLOB_lastAwakeLinearSlot % SLOTS_PER_FRAME_I);
	ucGLOB_lastAwakeNSTtblNum = (uint8) (lGLOB_lastAwakeFrame % 2);

	/* SET SYSTIME ALARM TO NEXT SLOT START */
	vTIME_setAlarmFromLong(lThisSlotEndTime);

#if 0
	vSERIAL_rom_sout("X\r\n");
#endif

	vSERIAL_crlf();

	return;

}/* END: vMAIN_computeDispatchTiming() */

/********************  vMAIN_showCompilerAnomalies()  *************************
 *
 * Run nothing in the brain so the SD can perform.
 *
 *
 *******************************************************************************/

void vMAIN_showCompilerAnomalies(const char *cMsg)
{
//buzzer was annoying
	//vBUZ_blink_buzzer(5);

	vSERIAL_crlf();
	vSERIAL_dash(20);
	vSERIAL_rom_sout(cMsg);
	vSERIAL_dash(20);
	vSERIAL_crlf();

	//vBUZ_blink_buzzer(5);

	return;

}/* END: vMAIN_showCompilerAnomalies() */

#ifdef RUN_NOTHING
/**************************  vMAIN_run_nothing()  ****************************
 *
 * Run nothing in the brain so the SD can perform.
 *
 *
 *****************************************************************************/

void vMAIN_run_nothing(
		void
)
{
	ucSDCTL_start_SD_and_specify_boot(SD_BOOT_LOADER_ENABLED); //lint !e534

	vSERIAL_init();
	vSERIAL_rom_sout("NOTHING:\r\n");

	vBUZ_blink_buzzer(1);

	HANG:
	goto HANG;

}/* END: vMAIN_run_nothing() */
#endif

/* -----------------------  END OF MODULE  ------------------------------- */
