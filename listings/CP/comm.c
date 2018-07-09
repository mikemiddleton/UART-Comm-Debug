/**************************  comm.C  *****************************************
 *
 * Routines to perform MSGs during events
 *
 *
 * V1.00 10/04/2003 wzr
 *		Started
 *
 * V2.00 10/15/2013 cp
 *
 *
 ******************************************************************************/
//! \file comm.c
//! \brief File handles all communications
//! \addtogroup Communications
//! @{
/*lint -e526 *//* function not defined */
/*lint -e657 *//* Unusual (nonportable) anonymous struct or union */
/*lint -e714 *//* symbol not referenced */
/*lint -e716 *//* while(1)... */
/*lint -e750 *//* local macro not referenced */
/*lint -e754 *//* local structure member not referenced */
/*lint -e755 *//* global macro not referenced */
/*lint -e757 *//* global declarator not referenced */
/*lint -e752 *//* local declarator not referenced */
/*lint -e758 *//* global union not referenced */
/*lint -e768 *//* global struct member not referenced */

#include <msp430x54x.h>		//processor reg description */
#include "diag.h"			//Diagnostic package
#include "std.h"			//standard defines
#include "misc.h"			//homeless functions
#include "crc.h"			//CRC calculation module
#include "stbl.h"			//Schedule table routines
#include "serial.h"			//serial IO port stuff
#include "comm.h"    		//event MSG module
#include "time.h"			//Time routines
#include "gid.h"			//group ID routines
#include "rad40.h"			//Radix 40 routines
#include "rand.h"			//random number generator
#include "sensor.h"			//sensor name routines
#include "task.h"
#include "action.h"
#include "rts.h" //scheduling functions
#include "discover.h"
#include "hal/config.h" 	//system configuration description file
#include "drivers/buz.h"
#include "drivers/adf7020.h"//radio driver
#include "mem_mod/l2fram.h" 		//Level 2 Fram routines
#include "mem_mod/l2sram.h"  		//disk storage module

#define MAX_LINKS_PER_SLOT		3

//! \var union DE_Command UCommandDE
//! \brief This union contains the fields required for a command data element
union DE_Command UCommandDE;
//! \var union DE_Report UReportDE
//! \brief This union contains the fields required for a report data element
union DE_Report UReportDE;
//! \var union DE_Code UCodeDE
//! \brief This union contains the fields required for a program code update data element
union DE_Code UCodeDE;

//! \var ucaLinkSlotTimes[MAX_LINKS_PER_HALFSLOT]
//! \brief This lookup table to provides the reply time based on the random linkslot selected
uint ucaLinkSlotTimes[MAX_LINKS_PER_SLOT] =
{ 0x0000, 0x200, 0x400 };

union Message_Advertisement UMsg_Advertisement;

union Message_Operational UMsg_Operational;

union Message_OperationalSynch UMsg_OperationalSynch;

union Message_RequestToJoin UMsg_RequestToJoin;

extern union SP_DataMessage g_RecDataMsg; //from sp.c this will change to a variable length data type

extern volatile uint8 ucaMSG_BUFF[MAX_RESERVED_MSG_SIZE];

extern uint8 ucGLOB_curMsgSeqNum;

extern uchar ucGLOB_myLevel; //senders level +1

extern uint uiGLOB_TotalSDC4trys; //counts number of SDC4 attempts

extern uint uiGLOB_TotalRTJ_attempts; //counts number of Request to Join attempts

extern long lGLOB_lastAwakeLinearSlot; //Nearest thing to cur linear slot

extern long lGLOB_OpMode0_inSec; //Time when OP mode started

extern unsigned int uiGLOB_curTripleOffset;

extern uchar ucGLOB_lastAwakeStblIdx; //Nearest thing to cur sched idx

extern uint uiGLOB_TotalRDC4trys; //counts number of RDC4 attempts

extern uchar ucGLOB_StblIdx_NFL; //Next free location pointer in the scheduler table

extern volatile union //ucFLAG0_BYTE
{
	uchar byte;
	struct
	{
		unsigned FLG0_BIGSUB_CARRY_BIT :1; //bit 0 ;1=CARRY, 0=NO-CARRY
		unsigned FLG0_BIGSUB_6_BYTE_Z_BIT :1; //bit 1 ;1=all diff 0, 0=otherwise
		unsigned FLG0_BIGSUB_TOP_4_BYTE_Z_BIT :1; //bit 2 ;1=top 4 bytes 0, 0=otherwise
		unsigned FLG0_NOTUSED_3_BIT :1; //bit 3 ;1=SOM2 link exists, 0=none
		//SET:	when any SOM2 links exist
		//CLR: 	when the SOM2 link is lost
		unsigned FLG0_RESET_ALL_TIME_BIT :1; //bit 4 ;1=do time  reset, 0=dont
		//SET:	when RDC4 gets finds first
		//		SOM2.
		//		or
		//		In a hub when it is reset.
		//
		//CLR: 	when vMAIN_computeDispatchTiming()
		//		runs next.
		unsigned FLG0_SERIAL_BINARY_MODE_BIT :1; //bit 5 1=binary mode, 0=text mode
		unsigned FLG0_HAVE_WIZ_GROUP_TIME_BIT :1; //bit 6 1=Wizard group time has
		//        been aquired from a DC4
		//      0=We are using startup time
		unsigned FLG0_NOTUSED7_BIT :1; //bit 7
	} FLAG0_STRUCT;
} ucFLAG0_BYTE;

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

const char *cpaMsgName[MSG_TYPE_MAX_COUNT] =
{ "NONE", //0 reserved
    "BCN", //1 beacon message
    "RTJ", //2 request to join
    "OPL", //3 operational message
    "TST1", //4
    "TST2" //5
    };

/*****************************  CODE STARTS HERE  ****************************/

/*WiSARDNet discovery routines*/

void vComm_PackReportDE(uchar ucSourceID)
{
	ulong ulTime;
	uchar ucIndex;

	//generating fake data
	g_RecDataMsg.fields.ucMsgSize = 13;
	g_RecDataMsg.fields.ucMsgVersion = 0x66;
	g_RecDataMsg.fields.ucSensorNumber = 0x03;
	g_RecDataMsg.fields.ucaParameters[0] = 0x12;
	g_RecDataMsg.fields.ucaParameters[1] = 0x34;
	g_RecDataMsg.fields.ucaParameters[2] = 0x56;
	g_RecDataMsg.fields.ucaParameters[3] = 0x78;

	//get the system time
	ulTime = lTIME_getSysTimeAsLong();

	//stuff the received fields into the data element structure
	ucaMSG_BUFF[0x00] = REPORT_DATA;
	ucaMSG_BUFF[0x01] = g_RecDataMsg.fields.ucMsgSize;
	ucaMSG_BUFF[0x02] = g_RecDataMsg.fields.ucMsgVersion;
	ucaMSG_BUFF[0x03] = ucSourceID;
	ucaMSG_BUFF[0x04] = g_RecDataMsg.fields.ucSensorNumber;
	ucaMSG_BUFF[0x05] = (uchar) ((ulTime >> 24) & 0xFF);
	ucaMSG_BUFF[0x06] = (uchar) ((ulTime >> 16) & 0xFF);
	ucaMSG_BUFF[0x07] = (uchar) ((ulTime >> 8) & 0xFF);
	ucaMSG_BUFF[0x08] = (uchar) (ulTime & 0xFF);

	//stuff the report parameters into the data element parameter array
	for (ucIndex = 0; ucIndex < (g_RecDataMsg.fields.ucMsgSize - 8); ucIndex++)
	{
		ucaMSG_BUFF[0x09 + ucIndex] = g_RecDataMsg.fields.ucaParameters[ucIndex];
	}

	// we need to add the SSP check as well as the storage to the SD card here

	//Store the DE in SRAM
	vL2SRAM_storeDEToSramIfAllowed();

}

///////////////////////////////////////////////////////////////////////////////
//! \brief Prepends the destination and source addresses to the packet
//!
//! Handles network layer responsibilities of communication protocol by building
//! the Net Package in front of a message
//!
//! \param none
//! \return none
///////////////////////////////////////////////////////////////////////////////
void vComm_NetPkg_buildHdr(uint uiDest)
{
	//Stuff the destination address
	ucaMSG_BUFF[NET_IDX_DEST_HI] = (uchar) (uiDest >> 8);
	ucaMSG_BUFF[NET_IDX_DEST_LO] = (uchar) (uiDest);

	// Stuff the source address
	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[NET_IDX_SRC_HI]);

} //END: vComm_NetPkg_buildHdr()

///////////////////////////////////////////////////////////////////////////////
//! \brief Builds the request to join message
//!
//! This is the message sent by a child node after it receives a beacon message.
//! It contains the random seed which is a deterministic random number
//! generated to coordinate communication in future slots.
//!
//! \param ulRandomSeed
//! \return none
///////////////////////////////////////////////////////////////////////////////
void vComm_Msg_buildRequest_to_Join(ulong ulRandomSeed)
{
	/* STUFF MSG TYPE */
	ucaMSG_BUFF[MSG_IDX_ID] = MSG_ID_REQUEST_TO_JOIN;

	// Stuff the message flags
	ucaMSG_BUFF[MSG_IDX_FLG] = 0x20;

	// Stuff the message number
	ucaMSG_BUFF[MSG_IDX_NUM_HI] = 0x00;
	ucaMSG_BUFF[MSG_IDX_NUM_LO] = 0x00;

	//Stuff the source address
	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_ADDR_HI]);

	/* STUFF HEADER SIZE+2 ONLY */
	ucaMSG_BUFF[MSG_IDX_LEN] = 0x10;

	// Stuff the random seed to schedule future communication
	vMISC_copyUlongIntoBytes(ulRandomSeed, (uchar *) &ucaMSG_BUFF[MSG_IDX_RANDSEED_XI], NO_NOINT);

	/* COMPUTE THE CRC */
	ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_SEND); //lint !e534 //compute the CRC

	//Show the message
#if 0
	uint8 ucCounter;

	vSERIAL_rom_sout("RTJ:\r\n");
	for (ucCounter = 0; ucCounter < ucaMSG_BUFF[MSG_IDX_LEN]; ucCounter++)
	{
		vSERIAL_HB8out(ucaMSG_BUFF[ucCounter]);
		vSERIAL_crlf();
	}
#endif

} //END: vComm_Msg_buildRequest_to_Join()

///////////////////////////////////////////////////////////////////////////////
//! \brief Builds the beacon message
//!
//! This message is used by parent nodes, including the hub, to advertise
//! possible network connectivity to any listening children.  It contains the
//! time for synchronization as well as group ID and distance in hops from the
//! hub (sourceIDlevel)
//!
///////////////////////////////////////////////////////////////////////////////
static void vComm_Msg_buildBeacon(long lSyncTimeSec)
{

	/* STUFF MSG TYPE */
	ucaMSG_BUFF[MSG_IDX_ID] = MSG_ID_BEACON;

	// Stuff the message flags
	ucaMSG_BUFF[MSG_IDX_FLG] = 0x20;

	// Stuff the message number
	ucaMSG_BUFF[MSG_IDX_NUM_HI] = 0x00;
	ucaMSG_BUFF[MSG_IDX_NUM_LO] = 0x00;

	//Stuff the address fields with an invalid address
	ucaMSG_BUFF[MSG_IDX_ADDR_HI] = 0xFF;
	ucaMSG_BUFF[MSG_IDX_ADDR_LO] = 0xFF;

	/* STUFF HEADER SIZE+2 ONLY */
	ucaMSG_BUFF[MSG_IDX_LEN] = 0x15;

	//Stuff group ID
	/* STUFF THE GROUP ID NUMBER */
	ucaMSG_BUFF[MSG_IDX_GID_HI] = ucGID_getWholeSysGidLoByte();
	ucaMSG_BUFF[MSG_IDX_GID_LO] = ucGID_getWholeSysGidHiByte();

	//Stuff the distance (in hops) from the hub
	ucaMSG_BUFF[MSG_IDX_SRC_LEVEL] = ucGLOB_myLevel;

	//Stuff the synch time in seconds
	vMISC_copyUlongIntoBytes((ulong) lSyncTimeSec, (uchar *) &ucaMSG_BUFF[BCNMSG_IDX_TIME_SEC_XI], NO_NOINT);

	vMISC_copyUintIntoBytes(uiTIME_getSubSecAsUint(), (uchar *) &ucaMSG_BUFF[BCNMSG_IDX_TIME_SUBSEC_HI], NO_NOINT);

	/* COMPUTE THE CRC */
	ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_SEND); //lint !e534 //compute the CRC

// Debug - Show the beacon message
#if 0
	uint8 ucCounter;

	vSERIAL_rom_sout("Beacon:\r\n");
	for(ucCounter=0;ucCounter<ucaMSG_BUFF[MSG_IDX_LEN];ucCounter++)
	{
		vSERIAL_HB8out(ucaMSG_BUFF[ucCounter]);
		vSERIAL_crlf();
	}
#endif

} //END: vComm_Msg_buildBeacon()

///////////////////////////////////////////////////////////////////////////////
//! \brief Builds the operational message
//!
//! This message contain can contain reports, commands, and code.  In the
//! event that it is the first message sent in a slot from a parent node
//! to a child then the first DE is an update time command and is explicitly packed
//! here
//!
///////////////////////////////////////////////////////////////////////////////
void vComm_Msg_buildOperational(uchar ucFlags, uint uiMsgNum, uint uiDest, uchar ucSendSynch)
{
	uint uiRetCode; //!< Return code from after reading the DEs from SRAM
	uchar ucDE_Length; //!< Length of the DE retrieved from SRAM
	uchar ucMsgPtr; //!< The pointer to available space in the message buffer

	/* STUFF MSG TYPE */
	ucaMSG_BUFF[MSG_IDX_ID] = MSG_ID_OPERATIONAL;

	// Stuff the message flags
	ucaMSG_BUFF[MSG_IDX_FLG] = ucFlags;

	// Stuff the message number
	ucaMSG_BUFF[MSG_IDX_NUM_HI] = (uchar) (uiMsgNum >> 8);
	ucaMSG_BUFF[MSG_IDX_NUM_LO] = (uchar) uiMsgNum;

	//Stuff the address fields with address
	ucaMSG_BUFF[MSG_IDX_ADDR_HI] = (uchar) (uiDest >> 8);
	ucaMSG_BUFF[MSG_IDX_ADDR_LO] = (uchar) uiDest;

	ucMsgPtr = 12;

	//if the send synch byte is true then point to the next DE location
	if (ucSendSynch)
	{
		ucMsgPtr = 17;

	}

	//Initialize the data element length variable
	ucDE_Length = 0x00;

	//Fetch messages from the queue
//	uiRetCode = ucL2SRAM_getCopyOfCurDE(ucMsgPtr);

	//ucDE_Length = (uchar) ((uiRetCode >> 8) & 0xFF);

	//Set the message length here
	//ucaMSG_BUFF[MSG_IDX_LEN] = MSG_IDX_LEN + ucDE_Length;
  ucaMSG_BUFF[MSG_IDX_LEN] = 18;

            //if the send synch byte is true then the first DE sent will be a command to the CP board to update it's time
	if (ucSendSynch)
	{

		//Stuff the synch time in seconds
		vMISC_copyUlongIntoBytes(lTIME_getSysTimeAsLong(), (uchar *) &ucaMSG_BUFF[OPMSG_IDX_TIME_SEC_XI], NO_NOINT);

		//Stuff the time in sub-seconds
		vMISC_copyUintIntoBytes(uiTIME_getSubSecAsUint(), (uchar *) &ucaMSG_BUFF[OPMSG_IDX_TIME_SUBSEC_HI], NO_NOINT);
	}

	/* COMPUTE THE CRC */
	ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_SEND); //lint !e534 //compute the CRC

} //END: vComm_Msg_buildOperational()

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits for the request to join message
//!
//! \param none
//! \return 0 for success or 1 for failure
///////////////////////////////////////////////////////////////////////////////
static uchar ucComm_Wait_for_RequesttoJoin(void)
{
	uchar ucIntegrityRetVal;
	uint uiOtherGuysSN;
	ulong uslRandNum;
	uint uiaLinkSN[MAX_LINKS_PER_SLOT];
	uchar ucLinkSNidx;

	//Init the SN array index
	ucLinkSNidx = 0;

	//Wait for replies
	while (TRUE)
	{
		if (!ucMSG_waitForMsgOrTimeout())
		{
			//No replies so shutdown radio and exit
			vADF7020_Quit();

			return 1;
		}

		//Something has been received

		//Check the message integrity
		//RET: Bit Err Mask, 0 if OK
		ucIntegrityRetVal = ucComm_chkMsgIntegrity(CHKBIT_CRC + CHKBIT_MSG_TYPE + CHKBIT_DEST_SN, CHKBIT_CRC + CHKBIT_MSG_TYPE + CHKBIT_DEST_SN,
		    MSG_ID_REQUEST_TO_JOIN, //msg type
		    0, //src SN
		    uiL2FRAM_getSnumLo16AsUint() //Dst SN
		    );

		if (!ucIntegrityRetVal)
			break; //found one break out now

#if 0
		vSERIAL_rom_sout("Rjt\r\n");
#endif

		//otherwise the message was rejected, restart the receiver and wait for another
		vADF7020_StartReceiver();

	} // End: while(true)

#if 0
	vSERIAL_rom_sout("RTJ good\r\n");
#endif

// Debug - Show the RTJ message
#if 0
	uint8 ucCounter;

	vSERIAL_rom_sout("RTJ Message:\r\n");
	for (ucCounter = 0; ucCounter < ucaMSG_BUFF[MSG_IDX_LEN]; ucCounter++)
	{
		vSERIAL_HB8out(ucaMSG_BUFF[ucCounter]);
		vSERIAL_crlf();
	}
#endif

	//Save message data
	uiOtherGuysSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_ADDR_HI], NO_NOINT);

	uslRandNum = ulMISC_buildUlongFromBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_RANDSEED_XI], NO_NOINT);

	/* STASH THE LINKUP SN */
	uiaLinkSN[ucLinkSNidx++] = uiOtherGuysSN;

	/* ENTER THIS ONE IN THE SCHED TABLE */
	vSTBL_putROM2inStbl(uiOtherGuysSN, //SN
	    uslRandNum //Random seed
	    );

#if 0
	/* REPORT TO CONSOLE */
	vSERIAL_rom_sout("ROM2<");
	vRAD40_showRad40(uiOtherGuysSN);
	vSERIAL_rom_sout(" Rnd=");
	vSERIAL_HB24out(uslRandNum);
	vSERIAL_crlf();
#endif

#if 1
	vSTBL_showSOM2andROM2counts(NO_CRLF);
	vSERIAL_rom_sout(" ");
	vSTBL_showRDC4andSDC4counts(YES_CRLF);
#endif

	return ucLinkSNidx;
} //END: ucComm_BeaconReply()

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends the beacon message
//!
//! @param none
//! @return none
///////////////////////////////////////////////////////////////////////////////
void vComm_SendBeacon(void)
{
	long lCurSec;
	uchar ucReply;
	uchar ucc;

	//Power up and initialize the radio
	vADF7020_WakeUp();

	//Set the channel
	unADF7020_SetChannel(TEST_CHANNEL);

	//Get the current time
	lCurSec = lTIME_getSysTimeAsLong();

	//Prepend network layer with an illegal destination address
	vComm_NetPkg_buildHdr(0xFFFF);

	//Build the Beacon message
	vComm_Msg_buildBeacon(lCurSec);

	//Load message into TX buffer
	unADF7020_LoadTXBuffer((uint8*) &ucaMSG_BUFF, ucaMSG_BUFF[MSG_IDX_LEN]);

	//Send the Message
	vADF7020_SendMsg();

	//Start the receiver
	vADF7020_StartReceiver();

	//Wait for replies from nodes
	ucReply = ucComm_Wait_for_RequesttoJoin();

	//Shutdown the radio
	vADF7020_Quit();

	//Build a link report

	//Log the report in memory

	//If there was a timeout or no nodes replied then exit
//  if(ucReply == 0) return;
//
//
//  /* BUILD UP TO 3 OM2 REPORTS */
//  for(ucc=0;  ucc<ucReply;  ucc++)
//  {
//    /* START A NEW OM2 */
//    if((ucc % 2) == 0)
//    {
//      /* START A NEW REPORT */
//      vREPORT_buildEmptyReportHull();
//    }
//
//    /* ADD AN ENTRY TO THE REPORT */
//    ucREPORT_addSensorDataToExistingReport(
//            SENSOR_ROM2_LINK_INFO,		//Sensor num 0
//            uiaLinkSN[ucc],			//Sensor Data 0
//            SENSOR_ROM2_INFO_REASON,	        //Sensor num 1
//            ROM2_LINK_ESTABLISHED		//Sensor Data 2
//				); //lint !e534 !e676
//
//    /* FINISH AN OLD OM2 */
//    if(((ucc % 2) == 1) || (ucc+1 == ucReply)) //if full or at lp end
//    {
//      /* WRITE OUT THE REPORT */
//      vREPORT_logReport(OPTPAIR_RPT_ROM2_LNKS_TO_RDIO,
//		  OPTPAIR_RPT_ROM2_LNKS_TO_FLSH
//		  );
//    }

} //END: vComm_SendBeacon()

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits to receive the Beacon message
//!
//!
//! @param none
//! @return ucReturnCode: 1 if failed, 0 for success
///////////////////////////////////////////////////////////////////////////////
static uchar ucComm_Wait_for_Beacon(void)
{
	uchar ucReturnCode; //!< Return code
	uchar ucSrcLevel; //!< Source level in network
	uint uiSrcSN; //!< Serial number of source
	uchar ucFoundStblIdx; //!<Index of source in scheduler table if it exists

	//Assume success
	ucReturnCode = 0x00;

	if (ucMSG_waitForMsgOrTimeout())
	{
		/* GOT A MSG -- CHK FOR: CRC, MSGTYPE, GROUPID, DEST_SN */
		if (!(ucComm_chkMsgIntegrity( //RET: Bit Err Mask, 0 if OK
		CHKBIT_CRC + CHKBIT_MSG_TYPE, //chk flags
		    CHKBIT_CRC + CHKBIT_MSG_TYPE, //report flags
		    MSG_ID_BEACON, //msg type
		    0, //src SN
		    0 //Dst SN
		    )))
		{
			//Message is good
			vSERIAL_rom_sout("Beacon Good\r\n");

			//check the link level to make sure that the network maintains the child to parent tree structure
			ucSrcLevel = ucaMSG_BUFF[MSG_IDX_SRC_LEVEL];
			if (ucSrcLevel > ucGLOB_myLevel)
			{
#if 1
				/* POST A REJECT MSG */
				vSERIAL_rom_sout("Beacon Rejected:Lvl, My=");
				vSERIAL_HB8out(ucGLOB_myLevel);
				vSERIAL_rom_sout(" Beacon=");
				vSERIAL_HB8out(ucSrcLevel);
				vSERIAL_crlf();
#endif
				//set the return variable to indicate an error
				ucReturnCode |= SRCLVLERR; //set the source level error flag
			}

			//Check for a pre-existing link to avoid multiple links between the same nodes
			uiSrcSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_ADDR_HI], NO_NOINT);
			ucFoundStblIdx = ucSTBL_searchStblsForMatch(uiSrcSN, E_ACTN_SOM2);

			/* IF PRE-EXISTING -- DON'T RECONNECT */
			if (ucFoundStblIdx < ucGLOB_StblIdx_NFL)
			{
#if 1
				/* REPORT TO CONSOLE THAT WE ALREADY HAVE THIS LINK */
				vSERIAL_rom_sout("Beacon Rejected:PrevLk=");
				vRAD40_showRad40(uiSrcSN);
				vSERIAL_crlf();
#endif
				ucReturnCode |= PREXISTLNKERR; //set the pre-existing link error flag
			}
		}

		else
		{
			//If we are here then the message failed the integrity check
			ucReturnCode |= MSGINTGRTYERR; //set the message integrity error flag
		}
	} //End: if(ucMSG_waitForMsgOrTimeout)

	else
	{
		//The slot timed out before a message was received
		ucReturnCode |= TIMEOUTERR;
	}

	return ucReturnCode;
} //END: ucComm_Wait_for_Beacon()

///////////////////////////////////////////////////////////////////////////////
//! \brief Waits to receive the Beacon message and responds with a Request to Join
//! message
//!
//! The discovery slot is divided into sub slots to allow multiple children to
//! respond to a single beacon message.  Without this feature several children
//! would request to join at the same time causing packet collisions.
//!
//! @param none
//! @return none
///////////////////////////////////////////////////////////////////////////////
void vComm_Request_to_Join(void)
{
	ulong uslRandSeed;
	ulong ulRandLinkSlot;
	ulong ulSlotStartTime_sec;
	uchar ucMsgXmitRetVal;
	uint uiMsgXmitOffset_clks;
	uint uiDest;
	uchar ucaSndTime[6];
	uint uiSubSecLatency;
	uint uiSubSecTemp;

	/* INC THE RTJ COUNTER */
	uiGLOB_TotalRTJ_attempts++;

	//Configure the timer to measure latency
	vTime_LatencyTimer(ON);

	//Power up and initialize the radio
	vADF7020_WakeUp();

	//set the channel
	unADF7020_SetChannel(TEST_CHANNEL);

	//Set the radio state to RX mode, enabling interrupts
	vADF7020_StartReceiver();

	if (!ucComm_Wait_for_Beacon()) //if we have received a beacon message
	{
		//Stop the latency timer
		vTime_LatencyTimer(OFF);
		//read time from the timer register
		uiSubSecLatency = LATENCY_TIMER;

#if 0
		vSERIAL_rom_sout("Latency Time = ");
		vSERIAL_HB16out(uiSubSecLatency);
		vSERIAL_crlf();
#endif

		// Save the new time in Clk2, therefore Clk1 and alarms are still good
		vTIME_setWholeClk2FromBytes((uchar *) &ucaMSG_BUFF[BCNMSG_IDX_TIME_SEC_XI]);

		// Determine the new timer value from the current value + measured value + constant
		uiSubSecTemp = (uiTIME_getSubSecAsUint() + uiSubSecLatency + 0x1AA);
		// Update the timer
		vTIME_setSubSecFromUint(uiSubSecTemp);

#if 0
		vSERIAL_rom_sout("Updated Time = ");
		vSERIAL_HB16out(uiSubSecTemp);
		vSERIAL_crlf();
#endif

		//Update the system flags to show time has been reset to group time
		if (ucFLAG0_BYTE.FLAG0_STRUCT.FLG0_HAVE_WIZ_GROUP_TIME_BIT == 0)
		{
			ucFLAG0_BYTE.FLAG0_STRUCT.FLG0_HAVE_WIZ_GROUP_TIME_BIT = 1; //we have group time
			ucFLAG0_BYTE.FLAG0_STRUCT.FLG0_RESET_ALL_TIME_BIT = 1;
		}

		/* SAVE THE LEVEL */
		ucGLOB_myLevel = ucaMSG_BUFF[MSG_IDX_SRC_LEVEL] + 1;

		/* SAVE THE GROUP ID */
		vGID_setWholeSysGidFromBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_GID_HI]);

		/*-----------------  COMPUTE THE RTJ REPLY TIME  ------------------------*/

		// choose a random link slot to respond in
		ulRandLinkSlot = (ulong) (ucRAND_getRolledMidSysSeed() % MAX_LINKS_PER_SLOT);

#if 0
		vSERIAL_rom_sout("ulRandLinkSlot= ");
		vSERIAL_HBV32out(ulRandLinkSlot);
		vSERIAL_crlf();
#endif

		/* COMPUTE THE SLOT START TIME */
		ulSlotStartTime_sec = (ulong) lTIME_getClk2AsLong();

		/* COMPUTE THE XMIT OFFSET FOR REPLY */
		uiMsgXmitOffset_clks = 0x2000 + ucaLinkSlotTimes[ulRandLinkSlot]; //for now I have a educated guess as to ta1r value at this point 0x2000

#if 0
		/* REPORT THE OFFSET TIME AND SLOT NUMBER */
		vSERIAL_rom_sout("RTJOffst=");
		vSERIAL_HB16out(uiMsgXmitOffset_clks);
		vSERIAL_crlf();
#endif

		/* PACK THE WHOLE TIME  */
		vMISC_copyUlongIntoBytes( //pack FULL sec part
		    ulSlotStartTime_sec, &ucaSndTime[0], NO_NOINT);
		vMISC_copyUintIntoBytes(uiMsgXmitOffset_clks, //pack SUBSEC part
		    &ucaSndTime[4], NO_NOINT);

#if 0
		{
			/* REPORT TO CONSOLE TRANSMIT TIMES */
			uchar ucjj;
			vSERIAL_rom_sout("XmtTm=");
			for (ucjj = 0; ucjj < 6; ucjj++)
			{
				vSERIAL_HB8out(ucaSndTime[ucjj]);
			}/* END: for() */

			vSERIAL_rom_sout(" =");
			vTIME_showWholeTimeInDuS(&ucaSndTime[0], YES_CRLF);
		}
#endif

		//Get a random seed to coordinate the next slot for communication
		uslRandSeed = uslRAND_getRolledFullSysSeed(); //get a new rand seed

		//copy the destination address into
		uiDest = ((ucaMSG_BUFF[NET_IDX_SRC_HI] << 8) | ucaMSG_BUFF[NET_IDX_SRC_LO]);

		//Build the network layer header
		vComm_NetPkg_buildHdr(uiDest);

		//Build the request to join message
		vComm_Msg_buildRequest_to_Join(uslRandSeed);

//		P2OUT &= ~BIT7;

		//Send the Message
		ucMsgXmitRetVal = ucMSG_doSubSecXmit(&ucaSndTime[0], USE_CLK2, NO_RECEIVER_START);

		//Shutdown the radio once the RTJ is sent
		vADF7020_Quit();

		//Synchronize the system time with the parent node
		vTIME_setSysTimeFromClk2();

		//Clear the RTJ slots
		vRTS_convertAllRTJslotsToSleep();

		//Indicate to listeners that the link is established
//		vBUZ_tune_TaDah_TaDah();

		//Add the new link to the scheduler tables
		vSTBL_putSOM2inStbl(uiDest, //dest serial num
		    uslRandSeed //Random seed
		    );

	} //End: if(!ucComm_Wait_for_Beacon())
	else
	{
		//Shutdown the radio, no beacon received
		vADF7020_Quit();
	}

} //End: vComm_Request_to_Join()

/////////////////////////////////////////////////////////////////////////////
//! \brief This function handles operational communication
//!
//!
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////////
void vComm_Send_OpMSG(void)
{
	uint uiOtherGuysSN; //!< Serial number of destination

	// Power up and initialize the radio
	vADF7020_WakeUp();

	// Set the channel
	unADF7020_SetChannel(DATA_CHANNEL);

	// GET THE OTHER LINK'S SERIAL NUM
	uiOtherGuysSN = (uint) ulL2SRAM_getStblEntry(SCHED_SN_TBL_NUM, //Tbl num
	    ucGLOB_lastAwakeStblIdx //Tbl Idx
	    );

	// Prepend network layer with an illegal destination address
	vComm_NetPkg_buildHdr(uiOtherGuysSN);

	// Build operational message: single message no Ack
	vComm_Msg_buildOperational(MSG_FLG_SINGLE, 1, uiOtherGuysSN, YES_SYNCH	);

	// Load message into TX buffer
	unADF7020_LoadTXBuffer((uint8*) &ucaMSG_BUFF, ucaMSG_BUFF[MSG_IDX_LEN]);

	// Send the Message
	vADF7020_SendMsg();

#if 0
		uchar ucjj;
		vSERIAL_rom_sout("Tx=");
		for (ucjj = 0; ucjj < 19; ucjj++)
		{
			vSERIAL_HB8out(ucaMSG_BUFF[ucjj]);
			vSERIAL_crlf();
		}
#endif

	// Start the receiver
	//vADF7020_StartReceiver();

}

/////////////////////////////////////////////////////////////////////////////
//! \brief This function handles operational communication
//!
//!
//!
//! \param none
//! \return none
/////////////////////////////////////////////////////////////////////////////
void vComm_Receive_OpMSG(void)
{
	uint uiSubSecLatency;
	uint uiSubSecTemp;

	//Configure the timer to measure latency
	vTime_LatencyTimer(ON);

	// Power up and initialize the radio
	vADF7020_WakeUp();

	// Set the channel
	unADF7020_SetChannel(DATA_CHANNEL);

	//Set the radio state to RX mode, enabling interrupts
	vADF7020_StartReceiver();

	if (ucMSG_waitForMsgOrTimeout())
	{
		//Stop the latency timer
		vTime_LatencyTimer(OFF);
		//read time from the timer register
		uiSubSecLatency = LATENCY_TIMER;

#if 0
		vSERIAL_rom_sout("Latency Time = ");
		vSERIAL_HB16out(uiSubSecLatency);
		vSERIAL_crlf();
#endif

		// Save the new time in Clk2, therefore Clk1 and alarms are still good
		vTIME_setWholeClk2FromBytes((uchar *) &ucaMSG_BUFF[OPMSG_IDX_TIME_SEC_XI]);

		// Determine the new timer value from the current value + measured value + constant
		uiSubSecTemp = (uiTIME_getSubSecAsUint() + uiSubSecLatency + 0x1AA);

		// Update the timer
		vTIME_setSubSecFromUint(uiSubSecTemp);

#if 0
		vSERIAL_rom_sout("Updated Time = ");
		vSERIAL_HB16out(uiSubSecTemp);
		vSERIAL_crlf();
#endif


#if 0
		uchar ucjj;
		vSERIAL_rom_sout("Rx=");
		for (ucjj = 0; ucjj < 19; ucjj++)
		{
			vSERIAL_HB8out(ucaMSG_BUFF[ucjj]);
			vSERIAL_crlf();
		}
#endif
	}
}
///////////////////////////////////////////////////////////////////////////////
//!
//! This routine checks the message header in this order:
//! 1. CRC
//! 2. Message Type
//! 3. Group ID
//! 4. Dest SN
//! 5. Src SN
//!
//!
//! \param ucChkByteBits
//!	<ul>
//!   <li>BIT7 Check CRC
//!   <li>BIT6 Check MSG ID
//!   <li>BIT5 Check group select
//!   <li>BIT4 Check group ID
//!   <li>BIT3 Check destination
//!   <li>BIT2 Check source
//!	  <li>BIT1	unused
//!	  <li>BIT0 unused
//!	</ul>
//!
//! \param ucReportByteBits
//! <ul>
//!   <li>BIT7 Report CRC error
//!   <li>BIT6 Report MSG ID error
//!   <li>BIT5 Report group select error
//!   <li>BIT4 Report group ID error
//!   <li>BIT3 Report destination error
//!   <li>BIT2 Report source error
//!	  <li>BIT1 unused
//!	  <li>BIT0 unused
//!	</ul>
//!
//! \return ucErrRetVal
//!	<ul>
//!   <li>BIT7 CRC error
//!   <li>BIT6	MSG ID error
//!   <li>BIT5 Group select error
//!   <li>BIT4 Group ID error
//!   <li>BIT3	Destination error
//!   <li>BIT2 Source error
//!	  <li>BIT1	unused
//!	  <li>BIT0 unused
//!	</ul>
///////////////////////////////////////////////////////////////////////////////
uchar ucComm_chkMsgIntegrity(
//RET: BitMask if BAD,  0 if OK
    uchar ucChkByteBits, uchar ucReportByteBits, uchar ucMsgType, uint uiExpectedSrcSN, uint uiExpectedDestSN)
{
	uchar ucErrRetVal;
	uint uiMsgDestSN;
	uint uiMsgSrcSN;

	ucErrRetVal = 0; //assume no errors

	/* CHECK THE CRC -- IF ITS BAD -- LOOP BACK */
	if ((ucChkByteBits & CHKBIT_CRC) && (!ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_REC)))
	{
		if (ucReportByteBits & CHKBIT_CRC)
		{
			vSERIAL_rom_sout("MSG:BdCRC\r\n");
		}
		ucErrRetVal |= CHKBIT_CRC;
	}

	/* CHECK FOR PROPER MSG TYPE */
	if ((ucChkByteBits & CHKBIT_MSG_TYPE) && (ucaMSG_BUFF[MSG_IDX_ID] != ucMsgType))
	{
		if (ucReportByteBits & CHKBIT_MSG_TYPE)
		{
			uchar ucGotMsgType;
			ucGotMsgType = ucaMSG_BUFF[MSG_IDX_ID];

			vSERIAL_rom_sout("MSG:MsgJammedExp ");
			//vSERIAL_UIV8out(ucMsgType);
			vSERIAL_rom_sout("(");
			if (ucMsgType < MSG_TYPE_MAX_COUNT)
				vSERIAL_rom_sout(cpaMsgName[ucMsgType]);
			vSERIAL_rom_sout(") got ");

			//vSERIAL_UIV8out(ucGotMsgType);
			vSERIAL_rom_sout("(");
			if (ucGotMsgType < MSG_TYPE_MAX_COUNT)
				vSERIAL_rom_sout(cpaMsgName[ucGotMsgType]);
			vSERIAL_rom_sout(")\r\n");
		}
		ucErrRetVal |= CHKBIT_MSG_TYPE;
	}

	/* CHECK FOR A GROUP SELECTOR MATCH */
	if ((ucChkByteBits & CHKBIT_GRP_SEL)
	    && (!ucGID_compareOnlySysGrpSelectToBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_GID_HI], ucReportByteBits & CHKBIT_GRP_SEL, //report flag
	        YES_CRLF)))
	{
		ucErrRetVal |= CHKBIT_GRP_SEL;
	}

	/* CHECK FOR GROUP ID */
	if ((ucChkByteBits & CHKBIT_GID) && (ucGID_getSysGrpSelectAsByte() == 0))
	{
		if (!ucGID_compareOnlySysGidToBytes((uchar *) &ucaMSG_BUFF[MSG_IDX_GID_HI], ucReportByteBits & CHKBIT_GID, YES_CRLF))
		{
			ucErrRetVal |= CHKBIT_GID;
		}
	}

	/* CHECK DEST ID */
	if (ucChkByteBits & CHKBIT_DEST_SN)
	{
		uiMsgDestSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[NET_IDX_DEST_HI], NO_NOINT);
		if (uiMsgDestSN != uiExpectedDestSN)
		{
			if (ucReportByteBits & CHKBIT_DEST_SN)
			{
				vSERIAL_rom_sout("MSG:BdDstSN ");
				vComm_showSNmismatch(uiExpectedDestSN, uiMsgDestSN, YES_CRLF);
			}
			ucErrRetVal |= CHKBIT_DEST_SN;
		}

	}/* END: if() */

	/* CHECK THE SOURCE SN */
	if (ucChkByteBits & CHKBIT_SRC_SN)
	{
		uiMsgSrcSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[NET_IDX_SRC_HI], NO_NOINT);
		if (uiMsgSrcSN != uiExpectedSrcSN)
		{
			if (ucReportByteBits & CHKBIT_SRC_SN)
			{
				vSERIAL_rom_sout("MSG:BDSrcSN ");
				vComm_showSNmismatch(uiExpectedSrcSN, uiMsgSrcSN, YES_CRLF);
			}
			ucErrRetVal |= CHKBIT_SRC_SN;
		}

	}/* END: if() */

	return (ucErrRetVal);

}/* END: ucComm_chkMsgIntegrity() */

/*********************** ucMSG_chkMsgIntegrity()  *************************
 *
 * This routine checks the message header in this order:
 *		1. CRC
 *		2. Message Type
 *		3. Group ID
 *		4. Dest SN
 *		5. Src SN
 *
 *
 *
 * CheckByteBits:
 *
 *    7        6        5        4        3         2       1        0
 *ÚÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄ¿
 *³chk crc ³chk msg ³  chk   ³  chk   ³  chk   ³  chk   ³ unused ³ unused ³
 *³        ³  type  ³ group  ³ group  ³dest SN ³src  SN ³        ³        ³
 *³        ³        ³selector³  ID    ³        ³        ³        ³        ³
 *ÀÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÙ
 *
 * ReportByteBits:
 *
 *    7        6        5        4        3         2       1        0
 *ÚÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄ¿
 *³ report ³ report ³ report ³ report ³ report ³ report ³ unused ³ unused ³
 *³  CRC   ³msg type³grp  sel³group ID³dest SN ³src  SN ³        ³        ³
 *³  err   ³  err   ³  err   ³  err   ³  err   ³  err   ³        ³        ³
 *ÀÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÙ
 *
 * RET value:
 *
 *    7        6        5        4        3         2       1        0
 *ÚÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄ¿
 *³ found  ³ found  ³ found  ³ found  ³ found  ³ found  ³ unused ³ unused ³
 *³  CRC   ³msg type³grp  sel³group ID³dest SN ³src  SN ³        ³        ³
 *³  err   ³  err   ³  err   ³  err   ³  err   ³  err   ³        ³        ³
 *ÀÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÙ
 *
 *
 *****************************************************************************/

uchar ucMSG_chkMsgIntegrity(
//RET: BitMask if BAD,  0 if OK
    uchar ucChkByteBits, uchar ucReportByteBits, uchar ucMsgType, uint uiExpectedSrcSN, uint uiExpectedDestSN)
{
	uchar ucErrRetVal;
	uint uiMsgDestSN;
	uint uiMsgSrcSN;

	ucErrRetVal = 0; //assume no errors

	/* CHECK THE CRC -- IF ITS BAD -- LOOP BACK */
	if ((ucChkByteBits & CHKBIT_CRC) && (!ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_REC)))
	{
		if (ucReportByteBits & CHKBIT_CRC)
		{
			vSERIAL_rom_sout("MSG:BdCRC\r\n");
		}
		ucErrRetVal |= CHKBIT_CRC;
	}

	/* CHECK FOR PROPER MSG TYPE */
	if ((ucChkByteBits & CHKBIT_MSG_TYPE) && ((ucaMSG_BUFF[GMH_IDX_MSG_TYPE] & GMH_MSG_TYPE_MASK) != ucMsgType))
	{
		if (ucReportByteBits & CHKBIT_MSG_TYPE)
		{
			uchar ucGotMsgType;
			ucGotMsgType = ucaMSG_BUFF[GMH_IDX_MSG_TYPE] & GMH_MSG_TYPE_MASK;

			vSERIAL_rom_sout("MSG:MsgJammedExp ");
			//vSERIAL_UIV8out(ucMsgType);
			vSERIAL_rom_sout("(");
			if (ucMsgType < MSG_TYPE_MAX_COUNT)
				vSERIAL_rom_sout(cpaMsgName[ucMsgType]);
			vSERIAL_rom_sout(") got ");

			//vSERIAL_UIV8out(ucGotMsgType);
			vSERIAL_rom_sout("(");
			if (ucGotMsgType < MSG_TYPE_MAX_COUNT)
				vSERIAL_rom_sout(cpaMsgName[ucGotMsgType]);
			vSERIAL_rom_sout(")\r\n");
		}
		ucErrRetVal |= CHKBIT_MSG_TYPE;
	}

	/* CHECK FOR A GROUP SELECTOR MATCH */
	if ((ucChkByteBits & CHKBIT_GRP_SEL)
	    && (!ucGID_compareOnlySysGrpSelectToBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_GID_HI], ucReportByteBits & CHKBIT_GRP_SEL, //report flag
	        YES_CRLF)))
	{
		ucErrRetVal |= CHKBIT_GRP_SEL;
	}

	/* CHECK FOR GROUP ID */
	if ((ucChkByteBits & CHKBIT_GID) && (ucGID_getSysGrpSelectAsByte() == 0))
	{
		if (!ucGID_compareOnlySysGidToBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_GID_HI], ucReportByteBits & CHKBIT_GID, YES_CRLF))
		{
			ucErrRetVal |= CHKBIT_GID;
		}
	}

	/* CHECK DEST ID */
	if (ucChkByteBits & CHKBIT_DEST_SN)
	{
		uiMsgDestSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_DEST_SN_HI], NO_NOINT);
		if (uiMsgDestSN != uiExpectedDestSN)
		{
			if (ucReportByteBits & CHKBIT_DEST_SN)
			{
				vSERIAL_rom_sout("MSG:BdDstSN ");
				vComm_showSNmismatch(uiExpectedDestSN, uiMsgDestSN, YES_CRLF);
			}
			ucErrRetVal |= CHKBIT_DEST_SN;
		}

	}/* END: if() */

	/* CHECK THE SOURCE SN */
	if (ucChkByteBits & CHKBIT_SRC_SN)
	{
		uiMsgSrcSN = uiMISC_buildUintFromBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_SRC_SN_HI], NO_NOINT);
		if (uiMsgSrcSN != uiExpectedSrcSN)
		{
			if (ucReportByteBits & CHKBIT_SRC_SN)
			{
				vSERIAL_rom_sout("MSG:BDSrcSN ");
				vComm_showSNmismatch(uiExpectedSrcSN, uiMsgSrcSN, YES_CRLF);
			}
			ucErrRetVal |= CHKBIT_SRC_SN;
		}

	}/* END: if() */

	return (ucErrRetVal);

}/* END: ucMSG_chkMsgIntegrity() */

/**********************  vComm_showSNmismatch()  ***********************
 *
 * Show the values of a serial number mismatch
 *
 *
 *****************************************************************************/
void vComm_showSNmismatch(uint uiExpectedVal, uint uiGotVal, uchar ucCRLF_flag //YES_CRLF, NO_CRLF
    )
{
	vSERIAL_rom_sout("EXP= #");
	vSERIAL_UIV16out(uiExpectedVal);
	vSERIAL_rom_sout(" GOT= #");
	vSERIAL_UIV16out(uiGotVal);
	if (ucCRLF_flag)
		vSERIAL_crlf();

	return;

}/* END: vComm_showSNmismatch() */

/*******************  vMSG_showStorageErr()  ********************************
 *
 *
 *
 *****************************************************************************/

void vMSG_showStorageErr(const char *cpLeadinMsg, unsigned long ulFailAddr, unsigned long ulWroteVal, unsigned long ulReadVal)
{

	vSERIAL_rom_sout(cpLeadinMsg);
	vSERIAL_rom_sout(" at ");
	vSERIAL_HB32out(ulFailAddr);
//	vSERIAL_crlf();
	vSERIAL_rom_sout(" Wrote ");
	vSERIAL_HB32out(ulWroteVal);
	vSERIAL_rom_sout(" read ");
	vSERIAL_HB32out(ulReadVal);
	vSERIAL_rom_sout(" xor= ");
	vSERIAL_HB32out(ulWroteVal ^ ulReadVal);
	vSERIAL_crlf();

}/* END: vMSG_showStorageErr() */

///////////////////////////////////////////////////////////////////////////
//! \brief Packs the message buffer header
//!
//! This routine builds an GENERIC msg header but has the following fixups:
//!	1. ucaMSG_BUFF[OM2_IDX_EOM_IDX] only has the header size+2 in it
//!	3. CRC bytes are not stuffed. (don't yet know where they are).
//!
///////////////////////////////////////////////////////////////////////////

void vMSG_buildMsgHdr_GENERIC( //MSG=Len,Type,Dest,Flags
    uchar ucMsgLen, uchar ucMsgID, uint uiDestSN, uchar ucFlags)
{
	// Stuff the destination address
	vMISC_copyUintIntoBytes(uiDestSN, (uchar *) &ucaMSG_BUFF[NET_IDX_DEST_HI], NO_NOINT);

	// Stuff the source address
	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[NET_IDX_SRC_HI]);

	/* STUFF HEADER SIZE+2 ONLY */
	ucaMSG_BUFF[MSG_IDX_LEN] = ucMsgLen;

	// Stuff the message flags
	ucaMSG_BUFF[MSG_IDX_FLG] = ucFlags;
	/* STUFF MSG TYPE */
	ucaMSG_BUFF[MSG_IDX_ID] = ucMsgID;

	/* STUFF THE GROUP ID NUMBER */
	ucaMSG_BUFF[MSG_IDX_GID_HI] = ucGID_getWholeSysGidHiByte();
	ucaMSG_BUFF[MSG_IDX_GID_LO] = ucGID_getWholeSysGidLoByte();

	/* STUFF THE SOURCE NUMBER */
	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_SRC_SN_HI]);

	/* STUFF THE DESTINATION */
	vMISC_copyUintIntoBytes(uiDestSN, (uchar *) &ucaMSG_BUFF[GMH_IDX_DEST_SN_HI], NO_NOINT);

	return;

}/* END: vMSG_buildMsgHdr_GENERIC() */

/**********************  ucMSG_waitForMsgOrTimeout() *****************************
 *
 * This routine assumes a clk alarm has already been setup.
 *
 * RET:	1 = GOT A MSG
 *		0 = Timed out
 *
 ******************************************************************************/

uchar ucMSG_waitForMsgOrTimeout(void)
{

	/* WAIT FOR MESSAGE COMPLETE */
	while (TRUE) //lint !e774
	{

		//End of slot
		if (ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T1_ALARM_MCH_BIT)
			break;

		//Aperture Buffer Started
		if (ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T2_ALARM_MCH_BIT)
			break;

		//Aperture Ended
		if (ucFLAG2_BYTE.FLAG2_STRUCT.FLG2_T3_ALARM_MCH_BIT)
			break;

		if (ucFLAG1_BYTE.FLAG1_STRUCT.FLG1_R_HAVE_MSG_BIT)
		{
			unADF7020_SetRadioState(RADIO_OFF);
			//Assumed that the message buffer is safe to pass and cast as
			//uint8* as opposed to volatile uint8*
			unADF7020_ReadRXBuffer((uint8*) &ucaMSG_BUFF[0x00]);

			ucFLAG1_BYTE.FLAG1_STRUCT.FLG1_R_HAVE_MSG_BIT = 0;
			return (1);
		}

	}/* END: while() */

	return (0);

}/* END: ucMSG_waitForMsgOrTimeout()*/

/***************************  vMSG_rec_obnd_msgs()  *******************************
 *
 * Receive the Outbound Messages.
 *
 *
 ******************************************************************************/

void vMSG_rec_obnd_msgs(void)
{

	return;

}/* END: vMSG_rec_obnd_msgs() */

/***********************  vMSG_showMsgBuffer() *********************************
 *
 * Print out the message buffer
 *
 ******************************************************************************/

void vMSG_showMsgBuffer(uchar ucCRLF_termFlag, //YES_CRLF, NO_CRLF
    uchar ucShowTypeFlag //SHOW_MSG_RAW, SHOW_MSG_COOKED
    )
{
	uchar ucii;
	uchar ucjj;
	uchar ucMsgSize;
	uchar ucByteCount;
	char *cpLeadStrPtr;
	char *cpTrailStrPtr;

	vSERIAL_rom_sout("MSG: ");

//	ucMsgSize = MAX_MSG_SIZE;
	ucMsgSize = ucaMSG_BUFF[0] & MAX_MSG_SIZE_MASK;
	ucMsgSize++;
	if (ucMsgSize < 16)
		ucMsgSize = 16;

	if (ucShowTypeFlag == SHOW_MSG_RAW)
		ucMsgSize = MAX_MSG_SIZE;

	for (ucii = 0; ucii < ucMsgSize; ucii++) //loop for all bytes
	{
		/* SETUP DEFAULTS TO SAVE CODE */
		ucByteCount = 1;
		cpLeadStrPtr = "";
		cpTrailStrPtr = ",";

		if (ucii != ucMsgSize - 2) //if CRC skip usual stuff
		{
			switch (ucii)
			{
				case 0:
					cpLeadStrPtr = "SZ=";
					//DEFAULT: ucByteCount = 1;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 1:
					cpLeadStrPtr = "ID=";
					//DEFAULT: ucByteCount = 1;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 2:
					cpLeadStrPtr = "GP=";
					ucByteCount = 2;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 4:
					cpLeadStrPtr = "SRC=";
					ucByteCount = 2;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 6:
					cpLeadStrPtr = "DST=";
					ucByteCount = 2;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 8:
					cpLeadStrPtr = "SEQ=";
					//DEFAULT: ucByteCount = 1;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 9:
					cpLeadStrPtr = "LD=";
					//DEFAULT: ucByteCount = 1;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 10:
					cpLeadStrPtr = "AGNT=";
					ucByteCount = 2;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

				case 12:
					cpLeadStrPtr = "T="; //time
					ucByteCount = 4;
					cpTrailStrPtr = ",\r\n";
				break;

				case 16:
				case 19:
				case 22:
				case 25:
				case 28:
					/* ASSUME RAW SETUP */
					//DEFAULT: ucByteCount = 1;
					cpLeadStrPtr = "  ";
					cpTrailStrPtr = "";

					/* IF COOKED CHANGE THE SETUP */
					if (ucShowTypeFlag == SHOW_MSG_COOKED)
					{
						vSENSOR_showSensorName(ucaMSG_BUFF[ucii], R_JUSTIFY);
						cpLeadStrPtr = "(";
						cpTrailStrPtr = ")";
					}

				break;

				case 17: //Sensor Data value
				case 20:
				case 23:
				case 26:
				case 29:
					cpLeadStrPtr = "=>";
					//DEFAULT: cpTrailStrPtr = ",";
					//DEFAULT: ucByteCount = 1;
					if (ucii != 29)
						ucByteCount = 2;
					if (ucShowTypeFlag == SHOW_MSG_COOKED)
						cpTrailStrPtr = ",";
				break;

				default:
					//DEFAULT: cpLeadStrPtr = "";
					//DEFAULT: ucByteCount = 1;
					//DEFAULT: cpTrailStrPtr = ",";
				break;

			}/* END: switch(ucii) */

			/* SHOW THE LEAD STRING PTR */
			vSERIAL_rom_sout(cpLeadStrPtr);

			/* SHOW THE BYTES IN HEX */
			for (ucjj = 0; ucjj < ucByteCount; ucjj++)
			{
				vSERIAL_HB8out(ucaMSG_BUFF[ucii + ucjj]);
			}

			/* BUMP THE COUNT OVER THE SHOW BYTES */
			ucii += (ucByteCount - 1);

			/* SHOW THE TRAILING STRING PTR */
			vSERIAL_rom_sout(cpTrailStrPtr);

			continue;

		}/* END: if() */

		/* SHOW THE FINAL CRC WHERE EVER IT IS */
		vSERIAL_rom_sout("  CRC=");
		vSERIAL_HB8out(ucaMSG_BUFF[ucii]);
		vSERIAL_HB8out(ucaMSG_BUFF[++ucii]);
		vSERIAL_crlf();

	}/* END: for(ucii) */

	if (ucCRLF_termFlag)
		vSERIAL_crlf();

}/* END: vMSG_showMsgBuffer() */

/**********************  ucMSG_doSubSecXmit()  *******************************
 *
 * RET:	1 if msg sent on time
 *		0 if msg not send (too late to send it)
 *
 *****************************************************************************/

uchar ucMSG_doSubSecXmit( //RET: 1-sent, 0-too late
    uchar ucpSendTime[6], //full start time (IN CLKS)
    uchar ucClkChoiceFlag, //USE_CLK2, USE_CLK1
    uchar ucStartRecFlag //YES_RECEIVER_START, NO_RECEIVER_START
    )
{
	long lCurSec;
	long lSendTimeSec;
	uint uiSubSecSendTime;

	/* CHECK THE START SEC AGAINST CUR TIME */
	if (ucClkChoiceFlag == USE_CLK1)
	{
		lCurSec = lTIME_getSysTimeAsLong(); //wups was SysTime clk
	}
	else
	{
		lCurSec = lTIME_getClk2AsLong(); //clk2
	}

	lSendTimeSec = (long) ulMISC_buildUlongFromBytes(&ucpSendTime[0], NO_NOINT);

	if (lCurSec > lSendTimeSec)
	{
#if 1
		vSERIAL_rom_sout("MSG:MissdMsgTmWas ");
		vSERIAL_HBV32out((ulong) lCurSec);
		vSERIAL_rom_sout("  wanted ");
		vSERIAL_HBV32out((ulong) lSendTimeSec);
		vSERIAL_crlf();
#endif
		return (0);
	}

	/* WAIT FOR FULL SEC TIC TO COME UP */
	if (ucClkChoiceFlag == USE_CLK2)
	{
		while (lSendTimeSec > lTIME_getClk2AsLong())
			; //lint !e722
	}
	else
	{
		while (lSendTimeSec > lTIME_getSysTimeAsLong())
			; //lint !e722
	}

	uiSubSecSendTime = ((uint) (ucpSendTime[4] << 8) | ucpSendTime[5]);

	/* WAIT FOR SUB SECOND TIC TO COME UP */
	while (uiSubSecSendTime > SUB_SEC_TIM)
		; //SUB_SEC

#if 0
	vSERIAL_rom_sout("MSG:BeforRadioXmt= ");
	vTIME_ShowWholeSysTimeInDuS(YES_CRLF);
#endif

	unADF7020_LoadTXBuffer((uint8*) &ucaMSG_BUFF, ucaMSG_BUFF[0]);
	/* SEND THE MSG */
	vADF7020_SendMsg();

#if 0
	vSERIAL_rom_sout("MSG:AfterRadioXmt= ");
	vTIME_ShowWholeSysTimeInDuS(YES_CRLF);
#endif

	/* CHECK FOR IMMEDIATE START OF RECEIVER */
	if (ucStartRecFlag)
		vADF7020_StartReceiver();

#if 0
	vSERIAL_rom_sout("MSG:AfterRadioRec= ");
	vTIME_ShowWholeSysTimeInDuS(YES_CRLF);
#endif

	return (1);

}/* vMSG_doSubSecXmit() */

/************************  vMSG_stuffFakeMsgToSRAM()  ***************************
 *
 * Builds a fake OM2 message and stuffs it into the SRAM
 *
 *
 ******************************************************************************/
void vMSG_stuffFakeMsgToSRAM(void)
{

	ucaMSG_BUFF[OM2_IDX_EOM_IDX] = OM2_MSG_LAST_BYTE_NUM_UC;
	ucaMSG_BUFF[OM2_IDX_MSG_TYPE] = MSG_ID_OPERATIONAL;

	vGID_copyWholeSysGidToBytes((uchar *) &ucaMSG_BUFF[GMH_IDX_GID_HI]);

	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[OM2_IDX_SRC_SN_HI]);

	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[OM2_IDX_DEST_SN_HI]);

	ucaMSG_BUFF[OM2_IDX_MSG_SEQ_NUM] = 0x50;

	/* STUFF THE LINK BYTE */
	ucaMSG_BUFF[OM2_IDX_GENERIC_LINK_BYTE] = 0;

	vL2FRAM_copySnumLo16ToBytes((uchar *) &ucaMSG_BUFF[OM2_IDX_AGENT_NUM_HI]);

	ucaMSG_BUFF[OM2_IDX_COLLECTION_TIME_XI] = 0;
	ucaMSG_BUFF[OM2_IDX_COLLECTION_TIME_HI] = 0;
	ucaMSG_BUFF[OM2_IDX_COLLECTION_TIME_MD] = 0;
	ucaMSG_BUFF[OM2_IDX_COLLECTION_TIME_LO] = 99;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 0] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 0] = 0x50;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 2] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 2] = 0x51;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 4] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 4] = 0x52;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 4] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 4] = 0x53;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 6] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 6] = 0x54;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 8] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 8] = 0x55;

	ucaMSG_BUFF[OM2_IDX_DATA_0_HI + 10] = 0x1;
	ucaMSG_BUFF[OM2_IDX_DATA_0_LO + 10] = 0x56;

	/* COMPUTE THE CRC */
	ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_SEND); //lint !e534 //compute CRC

	/* STUFF THE MSG TO THE DISK */
	vL2SRAM_storeMsgToSram();

#if 0
	vSERIAL_rom_sout("FakeMsgCnt=");
	vSERIAL_HB16out(uiL2SRAM_getMsgCount());
	vSERIAL_crlf();

	vSERIAL_rom_sout("Vacancy=");
	vSERIAL_HB16out(uiL2SRAM_getVacantMsgCount());
	vSERIAL_crlf();

	vSERIAL_rom_sout("FakeMsgBeforeStore\r\n");
	vMSG_showMsgBuffer(YES_CRLF,SHOW_MSG_COOKED);

	ucL2SRAM_getCopyOfCurMsg(); //lint !e534

	vSERIAL_rom_sout("FakeMsgAfterRetrieve\r\n");
	vMSG_showMsgBuffer(YES_CRLF,SHOW_MSG_COOKED);

#endif

	return;

}/* vMSG_stuffFakeMsgToSRAM() */

/************************  vMSG_checkSingleMsgBuffEntry()  **********************************
 *
 * Report an error in the msg buffer
 *
 *
 ******************************************************************************/

static void vMSG_checkSingleMsgBuffEntry(uint uiMsgIdx, uchar ucCorrectValue)
{

	if (ucaMSG_BUFF[uiMsgIdx] != ucCorrectValue)
	{
		vSERIAL_rom_sout("Byt ");
		vSERIAL_HB8out((uchar) uiMsgIdx);
		vSERIAL_rom_sout(" bd,needed ");
		vSERIAL_HB8out(ucCorrectValue);
		vSERIAL_rom_sout(" got ");
		vSERIAL_HB8out(ucaMSG_BUFF[uiMsgIdx]);
		vSERIAL_crlf();
	}

	return;

}/* END: vMSG_checkSingleMsgBuffEntry() */

/************************  vMSG_checkFakeMsg()  **********************************
 *
 * Check a fake OM2 message
 *
 *
 ******************************************************************************/

void vMSG_checkFakeMsg(void)
{
	vSERIAL_rom_sout("vChkMsg \r\n");

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_EOM_IDX, OM2_MSG_LAST_BYTE_NUM_UC);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_MSG_TYPE, MSG_ID_OPERATIONAL);

	vMSG_checkSingleMsgBuffEntry(GMH_IDX_GID_HI, ucGID_getWholeSysGidHiByte());
	vMSG_checkSingleMsgBuffEntry(GMH_IDX_GID_LO, ucGID_getWholeSysGidLoByte());

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_SRC_SN_HI, ucL2FRAM_getSnumMd8());
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_SRC_SN_LO, ucL2FRAM_getSnumLo8());

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DEST_SN_HI, ucL2FRAM_getSnumMd8());
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DEST_SN_LO, ucL2FRAM_getSnumLo8());

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_MSG_SEQ_NUM, 0x50);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_GENERIC_LINK_BYTE, 0x00);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_AGENT_NUM_HI, ucL2FRAM_getSnumMd8());
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_AGENT_NUM_LO, ucL2FRAM_getSnumLo8());

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_COLLECTION_TIME_XI, 0);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_COLLECTION_TIME_HI, 0);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_COLLECTION_TIME_MD, 0);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_COLLECTION_TIME_LO, 99);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 0, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 0, 0x50);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 2, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 2, 0x51);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 4, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 4, 0x52);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 6, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 6, 0x53);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 8, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 8, 0x54);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 10, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 10, 0x55);

	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_HI + 12, 0x1);
	vMSG_checkSingleMsgBuffEntry(OM2_IDX_DATA_0_LO + 12, 0x56);

	return;

}/* END: vMSG_checkFakeMsg() */

/************************  vMSG_incMsgSeqNum()  ******************************
 *
 * Increment the message seq number
 *
 * NOTE: The Seq number cannot be 0 or 255
 *
 * RET: Incremented Msg Seq Num
 *
 ******************************************************************************/

uchar ucMSG_incMsgSeqNum( //RET: Incremented Msg Seq Num (not 0 or 255)
    void)
{
	ucGLOB_curMsgSeqNum++;

	/* NOT 0 AND NOT 255 */
	if (ucGLOB_curMsgSeqNum >= 255)
		ucGLOB_curMsgSeqNum = 1;

	return (ucGLOB_curMsgSeqNum);

}/* END: vMSG_incMsgSeqNum() */

/********************  ucMSG_getLastFilledEntryInOM2()  **********************
 *
 * RET:	 0 = illegal
 *		## = index for Last entry in the msg (pts to sensor ID)
 *
 ******************************************************************************/
uchar ucMSG_getLastFilledEntryInOM2( //RET: 0=none, ##=idx of last entry in OM2
    void)
{
	uchar ucMsgLastDataStart;

	/* GET AN INDEX FOR THE LAST BEGINNING OF THE LAST ENTRY IN THE MSG */
	ucMsgLastDataStart = (ucaMSG_BUFF[0] & MAX_MSG_SIZE_MASK) - 4;

	/* NOW CHECK THAT END COUNT TO MAKE SURE ITS OK */
	switch (ucMsgLastDataStart)
	{
		case 16: //normal msg lengths
		case 19:
		case 22:
		case 25:
		break;

		case 27:
			ucMsgLastDataStart = 25; //had a short msg also
		break;

		default:
			ucMsgLastDataStart = 0; //illegal length
		break;

	}/* END: switch() */

	return (ucMsgLastDataStart);

}/* END: ucMSG_getLastFilledEntryInOM2() */

/*-------------------------------  MODULE END  ------------------------------*/
//! @}
