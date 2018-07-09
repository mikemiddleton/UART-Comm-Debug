//! \file comm.h
//! \brief The file contains the Messages module
//!
//!
//! @addtogroup Communications
//! The Communication Module (comm.h) contains all of the data structures and defines
//! to communicate back and forth between the SP Board boards, the radio, the
//! garden server and the brain board
//! @{
///////////////////////////////////////////////////////////////////////////////
#ifndef COMM_H_INCLUDED
#define COMM_H_INCLUDED

#include "std.h"

//! @name Data Element definitions
//! {

//! \def MAX_NUM_TRANSDUCERS
//! \brief This sets the maximum number of transducers the core can handle
#define MAX_NUM_TRANSDUCERS 0x10
//! \def TRANSDUCER_LABEL_LEN
//! \brief The fixed length of the transducer labels
#define TRANSDUCER_LABEL_LEN 0x10
//! \def VERSION_LABEL_LEN
//! \brief The fixed length of the version labels
#define VERSION_LABEL_LEN    0x10
//! \def SP_CORE_VERSION
//! \brief This sensor number is for requesting the version string
//!
//! This sensor number should only EVER be used with REQUEST_LABEL packet.
//! Otherwise the system will try to access an array index that does
//! not exist, which will fault the hardware and lock up the software.
#define SP_CORE_VERSION 0x10
//! \def WRAPPER_VERSION
//! \brief This sensor number is for requesting the wrapper version string
//!
//! This sensor number should only EVER be used with REQEUST_LABEL packet.
//! Otherwise the system will try to access an array index that does not
//! exist, which will fault the hardware and lock up the software.
#define WRAPPER_VERSION 0x11

//! \def SW_VERSION
//! \brief This is the version of the software running on the device generating the message
//!
//! For the release of the generation 3 WiSARD we are starting the software version at 4.0
//! as the latest release of the generation 2 software was 3.62a
#define SW_VERSION	0x04

//****************  SP Board Data Messages  *********************************//
//! @defgroup msg_data SP Board Data Message
//! The SP Board Data Message is used to sent data back and forth between the
//! SP Board and the CP Board. The request message is ALWAYS a data message, the
//! return message can be a data message or a label message.
//! @{
#define SP_DATAMESSAGE_VERSION 110     //!< Version 1.10
// Message Types
//! @name Data Message Types
//! These are the possible data message types.
//! @{
//! \def HAND_SHK
//! \brief This is used to confirm that CP and SP communicate both ways.
//!
//! This packet is sent when the SP Board board sent the ID_PKT and the brain
//! wants to make sure communication is both ways.
#define HAND_SHK        0x00

//! \def COMMAND_PKT
//! \brief This packet contains the command from the CP Board to the SP Board on
//! what functions to execute.
//!
//! The SP Board replies with a CONFIRM_COMMAND if the command is valid
#define COMMAND_PKT   		0x01

//! \def REPORT_DATA
//! \brief This packet reports a transducer measurement to the CP Board
//!
//! This packet is only sent to the CP Board. The
//! transducer measurement is contained in data1 through data 8.
//! The SP sends a
#define REPORT_DATA     	0x02

//! \def PROGRAM_CODE
//! \brief This packet contains program code
//!
//! This packet is only sent from the real-time-data-center to the target. The
//! packet contains a portion of a programming update.
#define PROGRAM_CODE     0x03

//! \def REQUEST_DATA
//! \brief This packet requests data from the SP Board board
//!
//! This packet is only sent from the CP Board to the SP Board.
//! The SP will send a REPORT_DATA packet as a reply, either 32 bit or 128 bit
//! packet depending on the SP function.
#define REQUEST_DATA    	0x04

//! \def REQUEST_LABEL
//! \brief This packet asks the SP Board board to send a label message back
//!
//! The packet is sent from the CP Board to the SP Board and the CP Board will
//! expect a SP_LabelMessage in return. At this time the data1 and
//! data2 fields in the message contain do-not-care values.
#define REQUEST_LABEL   	0x05

//! \def ID_PKT
//! \brief This packet identifies the SP Board board to the CP Board
//!
//! This packet is only ever sent when the SP Board starts up and should
//! never be sent by the CP Board.
#define ID_PKT          	0x06

//! \def CONFIRM_COMMAND
//! \brief This packet is used by the SP Board to confirm that the COMMAND_PKT
//! sent by the CP Board is valid and that the commands will be executed.
//!
#define CONFIRM_COMMAND 	0x07

//! \def SP_ERROR
//! \brief This packet is used by the SP Board to inform the CP board
//! that there was a transducer error
//!
#define REPORT_ERROR 			0x08

//! \def REQUEST_BSL_PW
//! \brief This packet is used by the CP board to request the bootstrap loader password
//!
//! The SP will look into flash address 0xFFE0 to 0xFFFF and send the contents to the
//! CP board.
//!
#define REQUEST_BSL_PW   	0x09

//! \def INTERROGATE
//! \brief This packet is used by the CP board to request the transducer information
//!
//!
#define INTERROGATE   0x0A

//! \def SET_SERIALNUM
//! \brief Used to set the serial number on the SP board from the CP.
//! This is done only during diagnostics mode
#define SET_SERIALNUM		0x0B

//! \def REPORT_LABEL
//! \brief This packet contains a label indicated by the sensor number.
//!
//! This type of label packet is only sent by the SP Board board in response
//! to a LABEL_REQUEST Data Message. It has a different packet length and
//! is longer than a Data Message and therefore can be received as a
//! Data Message in error.
#define REPORT_LABEL             0x01
//! @}

// Sensor Numbers
//! @name Data Message Sensor Numbers
//! These are the possible sensor numbers
//! @{
#define TRANSDUCER_0    0x00
#define TRANSDUCER_1    0x01
#define TRANSDUCER_2    0x02
#define TRANSDUCER_3    0x03
#define TRANSDUCER_4    0x04
#define TRANSDUCER_5    0x05
#define TRANSDUCER_6    0x06
#define TRANSDUCER_7    0x07
#define TRANSDUCER_8    0x08
#define TRANSDUCER_9    0x09
#define TRANSDUCER_A    0x0A
#define TRANSDUCER_B    0x0B
#define TRANSDUCER_C    0x0C
#define TRANSDUCER_D    0x0D
#define TRANSDUCER_E    0x0E
#define TRANSDUCER_F    0x0F
//! @}

//! \def MAX_PARAMETERS
//! \brief The limit on the length of the parameter and data fields of application layer messages
#define MAX_SP_PARAMETERS 0x20

//! \def MAX_OP_MSG_LENGTH
//! \brief The limit on the length of the parameter and data fields of transport layer messages
#define MAX_OP_MSG_LENGTH 0x40

//! @name Data Element structures
//! @{
//! \brief Structure of a command Data Element(DE)
//!
//! This structure is used to define the command DE.
struct DE_Command_feilds
{
		uint8 ucDEID; //!< Type of the message.
		uint8 ucDE_Length; //!< Size of the message
		uint8 ucVersion; //!< The version number of the sources software
		uint8 ucCommandID_HI; //!< High byte of the command ID, the address of the processor receiving the command
		uint8 ucCommandID_LO; //!< Low byte of the command ID, the command to be executed
		uint8 ucaParameters[MAX_SP_PARAMETERS];
};

//! \def  DE_COMMAND_SIZE
//! \brief The size of a data message (in bytes).
#define DE_COMMAND_SIZE sizeof(struct DE_Command_feilds)

//! \brief This is the union used to work with data messages
//!
//! This union allows us to work with the various fields in the data message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union DE_Command
{
		uint8 ucByteStream[DE_COMMAND_SIZE];
		struct DE_Command_feilds fields;
};

//! \brief Structure of a report Data Element(DE)
//!
//! This structure is used to define the report DE.
struct DE_Report_fields
{
		uint8 ucDEID; //!< Type of the message.
		uint8 ucDE_Length; //!< Size of the message
		uint8 ucVersion; //!< The version number of the sources software
		uint8 ucReportID_HI; //!< High byte of the report ID, the address of the processor sending the report
		uint8 ucReportID_LO; //!< Low byte of the report ID, the transducer ID of the report
		uint8 ucTimestamp_XI; //!< The fourth byte of the timestamp (highest)
		uint8 ucTimestamp_HI; //!< The third byte of the timestamp
		uint8 ucTimestamp_MD; //!< The second byte of the timestamp
		uint8 ucTimestamp_LO; //!< The first byte of the timestamp (lowest)
		uint8 ucaData[MAX_SP_PARAMETERS]; //!< The sensor data
};

//! \def  DE_COMMAND_SIZE
//! \brief The size of a data message (in bytes).
#define DE_REPORT_SIZE sizeof(struct DE_Report_fields)

//! \brief This is the union used to work with data messages
//!
//! This union allows us to work with the various fields in the data message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union DE_Report
{
		uint8 ucByteStream[DE_REPORT_SIZE];
		struct DE_Report_fields fields;
};

//! \brief Structure of a code Data Element(DE)
//!
//! This structure is used to define the code DE.
struct DE_Code_feilds
{
		uint8 ucDEID; //!< Type of the message.
		uint8 ucDE_Length; //!< Size of the message
		uint8 ucVersion; //!< The version number of the software
		uint8 ucDestID_HI; //!< The high byte of the destination ID, the processor to be reprogrammed
		uint8 ucDestID_LO; //!< The low byte of the destination ID, the board type being reprogrammed
		uint8 ucProgID; //!< The unique ID of the programming update
		uint8 ucComponentNum_HI; //!< The high byte of the component number
		uint8 ucComponentNum_LO; //!< The low byte of the component number
		uint8 ucStartAddr_XI; //!< The fourth byte of the start address of the programming update (highest)
		uint8 ucStartAddr_HI; //!< The third byte of the start address of the programming update
		uint8 ucStartAddr_MD; //!< The second byte of the start address of the programming update
		uint8 ucStartAddr_LO; //!< The first byte of the start address of the programming update (lowest)
};

//! \def  DE_COMMAND_SIZE
//! \brief The size of a data message (in bytes).
#define DE_CODE_SIZE sizeof(struct DE_Code_feilds)

//! \brief This is the union used to work with data messages
//!
//! This union allows us to work with the various fields in the data message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union DE_Code
{
		uint8 ucByteStream[DE_CODE_SIZE];
		struct DE_Code_feilds fields;
};
//! @}

//! @name SP data messages
//! \brief Structure of a SP Data Message
//!
//! This structure is only used to define the fields in SP_DataMessage.
struct SP_DataMessage_Fields
{
		uint8 ucMsgType; //!< Type of the message.
		uint8 ucMsgSize; //!< Size of the message
		uint8 ucMsgVersion; //!< The version number of the message protocol
		uint8 ucSensorNumber; //!< Sensor number.
		uint8 ucaParameters[MAX_SP_PARAMETERS]; //!< Sensor readings, command arguments
};

//! \def SP_DATAMESSAGE_SIZE
//! \brief The size of a data message (in bytes).
#define SP_DATAMESSAGE_SIZE sizeof(struct SP_DataMessage_Fields)

//! \brief This is the union used to work with data messages
//!
//! This union allows us to work with the various fields in the data message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union SP_DataMessage
{
		uint8 ucByteStream[SP_DATAMESSAGE_SIZE];
		struct SP_DataMessage_Fields fields;
};
//! @}

//! @{name SP board label message
//! \brief Structure of a SP Board Label Message
//!
//! This structure is only used inside of the SP_LabelMessage. It
//! should never be allocated on it's own.
struct SP_LabelMessage_Fields
{
		uint8 ucMsgType; //!< Type of the message.
		uint8 ucMsgSize; //!< Size of the message
		uint8 ucMsgVersion; //!< The version number of the message protocol
		uint8 ucSensorNumber; //!< Sensor Number
		uint8 ucaDescription[TRANSDUCER_LABEL_LEN]; //!< 16-byte char array for the label
};

//! \def SP_LABELMESSAGE_SIZE
//! \brief The size of the SP Board Label Message (in bytes).
#define SP_LABELMESSAGE_SIZE sizeof(struct SP_LabelMessage_Fields)

//! \brief The union used to work with label messages
//!
//! This union is what should be allocated to work with label messages. It
//! allows easy access into the various fields of the message and makes it
//! easy to send and receive using byte oriented communication methods.
union SP_LabelMessage
{
		uint8 ucByteStream[SP_LABELMESSAGE_SIZE];
		struct SP_LabelMessage_Fields fields;
};

//! \def SP_HEADERSIZE
//! \brief The size of the SP message header Type, Size, Version fields
#define SP_HEADERSIZE		0x03
//! @}

//****************  SP Board Label Message  *********************************//
//! @defgroup msg_label SP Board Label Message
//! The SP Board Label Message is used to send labels (byte strings) to the
//! CP board. The label message is only ever sent in response to a
//! LABEL_REQUEST Data Message. (See \ref msg_data).
//! @{
#define SP_LABELMESSAGE_VERSION 102        //!< v1.02
// Sensor Types
//! \name Label Message Sensor Numbers
//! These are the possible sensor numbers for Label Messages. While these
//! do overlap with the sensor numbers in \ref msg_data it is important
//! to use the right sensor number with the right packet in case one of the
//! message types changes.
//! @{
#define TRANSDUCER_0_LABEL       0x00
#define TRANSDUCER_1_LABEL       0x01
#define TRANSDUCER_2_LABEL       0x02
#define TRANSDUCER_3_LABEL       0x03
#define TRANSDUCER_4_LABEL       0x04
#define TRANSDUCER_5_LABEL       0x05
#define TRANSDUCER_6_LABEL       0x06
#define TRANSDUCER_7_LABEL       0x07
#define TRANSDUCER_8_LABEL       0x08
#define TRANSDUCER_9_LABEL       0x09
#define TRANSDUCER_A_LABEL       0x0A
#define TRANSDUCER_B_LABEL       0x0B
#define TRANSDUCER_C_LABEL       0x0C
#define TRANSDUCER_D_LABEL       0x0D
#define TRANSDUCER_E_LABEL       0x0E
#define TRANSDUCER_F_LABEL       0x0F
#define SP_CORE_VERSION_LABEL    0x10
#define WRAPPER_VERSION_LABEL    0x11
//! @}

//! \brief Structure of an advertisement message
//!
//! This structure is used to define the advertisement message.
struct Message_Advert_fields
{
		uint8 ucID; //!< Type of the message.
		uint8 ucFlags; //!< Message Flags
		uint8 ucNumber_HI; //!< The high byte of the message number
		uint8 ucNumber_LO; //!< The low byte of the message number
		uint8 ucAddress_HI; //!< The high byte of the message address
		uint8 ucAddress_LO; //!< The low byte of the message address
		uint8 ucLength; //!< The length of the message
		uint8 ucGroupID_HI; //!< The ID of the local network
		uint8 ucGroupID_LO; //!< The ID of the local network
		uint8 ucTime_Sec_XI; //!< The fourth byte of the time in seconds (highest)
		uint8 ucTime_Sec_HI; //!< The third byte of the time in seconds
		uint8 ucTime_Sec_MD; //!< The second byte of the time in seconds
		uint8 ucTime_Sec_LO; //!< The first byte of the time in seconds (lowest)
		uint8 ucTime_SubSec_HI; //!< The high byte of the time in sub-seconds
		uint8 ucTime_SubSec_LO; //!< The low byte of the time in sub-seconds
		uint8 ucSourceIDLevel; //!< The distance (in hops) from the Hub
};

//! \def  MESSAGE_Advert_SIZE
//! \brief The size of an advertisement message (in bytes).
#define MESSAGE_ADVERT_SIZE sizeof(struct Message_Advert_fields)

//! \brief This is the union used to work with messages
//!
//! This union allows us to work with the various fields in the message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union Message_Advertisement
{
		uint8 ucByteStream[MESSAGE_ADVERT_SIZE];
		struct Message_Advert_fields fields;
};

//! \brief Structure of a request to join message
//!
//! This structure is used to define the request to join message.
struct Message_RequestToJoin_fields
{
		uint8 ucID; //!< Type of the message.
		uint8 ucFlags; //!< Message Flags
		uint8 ucNumber_HI; //!< The high byte of the message number
		uint8 ucNumber_LO; //!< The low byte of the message number
		uint8 ucAddress_HI; //!< The high byte of the message address
		uint8 ucAddress_LO; //!< The low byte of the message address
		uint8 ucLength; //!< The length of the message
		uint8 ucGroupID_HI; //!< The ID of the local network
		uint8 ucGroupID_LO; //!< The ID of the local network
		uint8 ucRand_Seed_XI; //!< The fourth byte of the random seed (highest) used to coordinate future communication
		uint8 ucRand_Seed_HI; //!< The third byte of the random seed
		uint8 ucRand_Seed_MD; //!< The second byte of the random seed
		uint8 ucRand_Seed_LO; //!< The first byte of the random seed (lowest)
};

//! \def  MESSAGE_RequestToJoin_SIZE
//! \brief The size of an advertisement message (in bytes).
#define MESSAGE_REQUESTTOJOIN_SIZE sizeof(struct Message_RequestToJoin_fields)

//! \brief This is the union used to work with messages
//!
//! This union allows us to work with the various fields in the message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union Message_RequestToJoin
{
		uint8 ucByteStream[MESSAGE_REQUESTTOJOIN_SIZE];
		struct Message_RequestToJoin_fields fields;
};

//! \brief Structure of an operational message with synchronization time
//!
//! This structure is used to define the operational message.
struct Message_SlotStartOp_fields
{
		uint8 ucID; //!< Type of the message.
		uint8 ucFlags; //!< Message Flags
		uint8 ucNumber_HI; //!< The high byte of the message number
		uint8 ucNumber_LO; //!< The low byte of the message number
		uint8 ucAddress_HI; //!< The high byte of the message address
		uint8 ucAddress_LO; //!< The low byte of the message address
		uint8 ucLength; //!< The length of the message
		uint8 ucTime_Sec_XI; //!< The fourth byte of the time in seconds (highest)
		uint8 ucTime_Sec_HI; //!< The third byte of the time in seconds
		uint8 ucTime_Sec_MD; //!< The second byte of the time in seconds
		uint8 ucTime_Sec_LO; //!< The first byte of the time in seconds (lowest)
		uint8 ucaPayload[MAX_OP_MSG_LENGTH]; //!< The payload of this message consists of data elements
};

//! \def  MESSAGE_OP_SIZE
//! \brief The size of a message (in bytes).
#define MESSAGE_SLOTSTARTOP_SIZE sizeof(struct Message_SlotStartOp_fields)

//! \brief This is the union used to work with messages
//!
//! This union allows us to work with the various fields in the message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union Message_OperationalSynch
{
		uint8 ucByteStream[MESSAGE_SLOTSTARTOP_SIZE];
		struct Message_SlotStartOp_fields fields;
};

//! \brief Structure of an operational message
//!
//! This structure is used to define the operational message.
struct Message_Op_fields
{
		uint8 ucID; //!< Type of the message.
		uint8 ucFlags; //!< Message Flags
		uint8 ucNumber_HI; //!< The high byte of the message number
		uint8 ucNumber_LO; //!< The low byte of the message number
		uint8 ucAddress_HI; //!< The high byte of the message address
		uint8 ucAddress_LO; //!< The low byte of the message address
		uint8 ucLength; //!< The length of the message
		uint8 ucaPayload[MAX_OP_MSG_LENGTH]; //!< The payload of this message consists of data elements
};

//! \def  MESSAGE_OP_SIZE
//! \brief The size of a message (in bytes).
#define MESSAGE_OP_SIZE sizeof(struct Message_Op_fields)

//! \brief This is the union used to work with messages
//!
//! This union allows us to work with the various fields in the message
//! easily while also making it easy send and receive using byte oriented
//! communication methods
union Message_Operational
{
		uint8 ucByteStream[MESSAGE_OP_SIZE];
		struct Message_Op_fields fields;
};

//! \def MAX_SP_MSGSIZE
//! \brief The max limit on messages received from SP boards
#define MAX_SP_MSGSIZE		0x40

#define YES_RECEIVER_START 1
#define  NO_RECEIVER_START 0

#define USE_CLK2 1
#define USE_CLK1 0

#define SHOW_MSG_RAW		1
#define SHOW_MSG_COOKED		2

#define MAX_RESERVED_MSG_SIZE 70

#define MAX_MSG_SIZE 32							//radio msg pkt max size
//	#define MAX_MSG_SIZE 64							//radio msg pkt max size
#define MAX_MSG_SIZE_L ((long) MAX_MSG_SIZE)
#define MAX_MSG_SIZE_UL ((ulong) MAX_MSG_SIZE)

#define MAX_MSG_SIZE_MASK  (MAX_MSG_SIZE -1)
#define MAX_MSG_SIZE_MASK_L ((long)MAX_MSG_SIZE_MASK)

/* CHECK_BYTE_BIT DEFINITIONS */
#define CHKBIT_CRC			0x80	//10000000
#define CHKBIT_MSG_TYPE		0x40	//01000000
#define CHKBIT_GRP_SEL		0x20	//00100000
#define CHKBIT_GID			0x10	//00010000
#define CHKBIT_DEST_SN		0x08	//00001000
#define CHKBIT_SRC_SN		0x04	//00000100
#define CHKBIT_UNUSED_1		0x02	//00000010
#define CHKBIT_UNUSED_2		0x01	//00000001
/* SLOT AND FRAME DEFINITIONS */
#define SLOTS_PER_FRAME_I  60
#define SLOTS_PER_FRAME_L  ((long) SLOTS_PER_FRAME_I)

#define SECS_PER_SLOT_I 1
#define SECS_PER_SLOT_L ((long) SECS_PER_SLOT_I)

#define SECS_PER_FRAME_L (SLOTS_PER_FRAME_L * SECS_PER_SLOT_L)

/* GENERAL RADIO DEFINES */

#define MSG_ST_DELAY_IN_nS_UL ((ulong) (300 * 1000000)) //300ms
#define MSG_ST_DELAY_IN_TICS_UL (MSG_ST_DELAY_IN_nS_UL / CLK_nS_PER_LTIC_UL)
#define MSG_ST_DELAY_IN_TICS_UI ((uint) MSG_ST_DELAY_IN_TICS_UL)

#define MSG_ST_DELAY_IN_CLKS_UI (MSG_ST_DELAY_IN_TICS_UI | 0x8000)

#define RADIO_BIT_IN_nS_UL (800UL * 1000UL)
#define RADIO_BIT_IN_TICS_X_10000_UL ((ulong) 262144)

#define RADIO_WARMUP_IN_nS_UL   (20UL * 1000000UL)			//20ms
#define RADIO_WARMUP_IN_100us_UL (RADIO_WARMUP_IN_nS_UL/100000)
#define RADIO_WARMUP_IN_100us_UI ((uint) RADIO_WARMUP_IN_100us_UL)
#define RADIO_WARMUP_IN_TICS_UL (RADIO_WARMUP_IN_nS_UL / CLK_nS_PER_LTIC_UL)
#define RADIO_WARMUP_IN_TICS_UI ((uint) RADIO_WARMUP_IN_TICS_UL)

#define RECEIVER_DIGEST_IN_nS_L  ((2L * 1000000L) - 64000L) //2ms - 64us
#define RECEIVER_DIGEST_IN_TICS_UL (RECEIVER_DIGEST_IN_nS_L / CLK_nS_PER_LTIC_UL)
#define RECEIVER_DIGEST_IN_TICS_UI ((uint) RECEIVER_DIGEST_IN_TICS_UL)

#define IDLE_IN_nS_UL (2UL * 1000000UL)	//2ms
#define SUB_MINUS_PKT_IN_nS_UL (RADIO_WARMUP_IN_nS_UL + RECEIVER_DIGEST_IN_nS_UL + IDLE_IN_nS_UL)

#define RADIO_LEADER_SIZE_IN_BYTES_UL 6UL //leader = preamble + barker
#define RADIO_LEADER_SIZE_IN_BITS_UL (RADIO_LEADER_SIZE_IN_BYTES_UL * 8UL)
#define RADIO_LEADER_IN_nS_UL (RADIO_LEADER_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

#define MAX_VYING_XMITTERS_UL  8UL

#define REPEATED_XMISSIONS_UC  8
#define REPEATED_XMISSIONS_UI  ((uint)  REPEATED_XMISSIONS_UC)
#define REPEATED_XMISSIONS_UL  ((ulong) REPEATED_XMISSIONS_UI)

#define MEAN_FREE_CAPACITY_UL  (8UL/4UL)

#define MSG_XFLDSIZE 6

/* --------------------  MESSAGE DEFINES ------------------------------- */

/********  MESSAGE TYPES  ***********/

//#define MSG_TYPE_STARTUP		1
#define MSG_ID_BEACON						1
#define MSG_ID_REQUEST_TO_JOIN	2
#define MSG_ID_OPERATIONAL			3

//TODO: to be removed
// {
#define MSG_TYPE_DC1			2
#define MSG_TYPE_DC2			3
#define MSG_TYPE_DC3		  4

#define MSG_TYPE_OM1			5
#define MSG_TYPE_OM2			6
#define MSG_TYPE_OM3			7

#define MSG_TYPE_DC4			8
#define MSG_TYPE_DC5			9
#define MSG_TYPE_DC6		   10
// }

#define MSG_TYPE_TS1		   11  //TEST 1
#define MSG_TYPE_TS2		   12  //TEST 2
#define MSG_TYPE_MAX_COUNT	 6

#define MAX_MSG_TYPE 16						//limit on msg type count
#define MAX_MSG_TYPE_MASK 		0x0F			//00001111		//mask
/* GENERIC MESSAGE HEADER */

//! \def NET_IDX_DEST_HI
//! \brief Network layer destination address index (high byte)
#define NET_IDX_DEST_HI					0

//! \def NET_IDX_DEST_LO
//! \brief Network layer destination address index (low byte)
#define NET_IDX_DEST_LO					1

//! \def NET_IDX_SRC_HI
//! \brief Network layer source address index (high byte)
#define NET_IDX_SRC_HI		  		2

//! \def NET_IDX_SRC_LO
//! \brief Network layer source address index (low byte)
#define NET_IDX_SRC_LO		  		3

//! \def MSG_IDX_ID
//! \brief Transport layer message identification index (8-bits)
#define MSG_IDX_ID							4

//! \def MSG_IDX_FLG
//! \brief Transport layer message flags index (8-bits)
#define MSG_IDX_FLG							5

//! \def MSG_IDX_NUM_HI
//! \brief Transport layer message number index (high byte)
#define MSG_IDX_NUM_HI					6

//! \def MSG_IDX_NUM_LO
//! \brief Transport layer message number index (low byte)
#define MSG_IDX_NUM_LO					7

//! \def MSG_IDX_ADDR_HI
//! \brief Transport layer message address index (high byte)
#define MSG_IDX_ADDR_HI					8

//! \def MSG_IDX_ADDR_LO
//! \brief Transport layer message address index (low byte)
#define MSG_IDX_ADDR_LO					9

//! \def MSG_IDX_LEN
//! \brief Transport layer message length index (8-bits)
#define MSG_IDX_LEN							10

//! \def MSG_IDX_PAYLD
//! \brief Transport layer message payload start index (8-bits)
#define MSG_IDX_PAYLD						11

//! \def MSG_IDX_GID_HI
//! \brief Transport layer beacon message group ID index (high byte)
#define MSG_IDX_GID_HI					11

//! \def MSG_IDX_GID_LO
//! \brief Transport layer beacon message group ID index (low byte)
#define MSG_IDX_GID_LO					12

//! \def MSG_IDX_RANDSEED_XI
//! \brief Transport layer request to join message random seed index (extra high byte)
#define MSG_IDX_RANDSEED_XI			11

//! \def MSG_IDX_RANDSEED_HI
//! \brief Transport layer request to join message random seed index (high byte)
#define MSG_IDX_RANDSEED_HI			12

//! \def MSG_IDX_RANDSEED_MD
//! \brief Transport layer request to join message random seed index (mid byte)
#define MSG_IDX_RANDSEED_MD			13

//! \def MSG_IDX_RANDSEED_LO
//! \brief Transport layer request to join message random seed index (low high byte)
#define MSG_IDX_RANDSEED_LO			14

//! \def BCNMSG_IDX_TIME_SEC_XI
//! \brief Transport layer beacon message time in seconds index (extra high byte)
#define BCNMSG_IDX_TIME_SEC_XI			13

//! \def BCNMSG_IDX_TIME_SEC_HI
//! \brief Transport layer beacon message time in seconds index (high byte)
#define BCNMSG_IDX_TIME_SEC_HI			14

//! \def BCNMSG_IDX_TIME_SEC_MD
//! \brief Transport layer beacon message time in seconds index (mid byte)
#define BCNMSG_IDX_TIME_SEC_MD			15

//! \def BCNMSG_IDX_TIME_SEC_LO
//! \brief Transport layer beacon message time in seconds index (low byte)
#define BCNMSG_IDX_TIME_SEC_LO			16

//! \def BCNMSG_IDX_TIME_SUBSEC_HI
//! \brief Transport layer beacon message time in sub-seconds index (high byte)
#define BCNMSG_IDX_TIME_SUBSEC_HI	17

//! \def BCNMSG_IDX_TIME_SUBSEC
//! \brief Transport layer beacon message time in sub-seconds index (low byte)
#define BCNMSG_IDX_TIME_SUBSEC_LO 	18

//! \def MSG_IDX_SRC_LEVEL
//! \brief Transport layer beacon message source level index (8-bits)
#define MSG_IDX_SRC_LEVEL		19

//! \def OPMSG_IDX_TIME_SEC_XI
//! \brief Transport layer operational message time in seconds index (extra high byte)
#define OPMSG_IDX_TIME_SEC_XI			11

//! \def OPMSG_IDX_TIME_SEC_HI
//! \brief Transport layer operational message time in seconds index (high byte)
#define OPMSG_IDX_TIME_SEC_HI			12

//! \def OPMSG_IDX_TIME_SEC_MD
//! \brief Transport layer operational message time in seconds index (mid byte)
#define OPMSG_IDX_TIME_SEC_MD			13

//! \def OPMSG_IDX_TIME_SEC_LO
//! \brief Transport layer operational message time in seconds index (low byte)
#define OPMSG_IDX_TIME_SEC_LO			14

//! \def OPMSG_IDX_TIME_SUBSEC_HI
//! \brief Transport layer operational message time in sub-seconds index (high byte)
#define OPMSG_IDX_TIME_SUBSEC_HI	15

//! \def OPMSG_IDX_TIME_SUBSEC
//! \brief Transport layer operational message time in sub-seconds index (low byte)
#define OPMSG_IDX_TIME_SUBSEC_LO 	16

//! @defgroup Messages Flags
//! @{
//! \def MSG_FLG_ACKRQST
//! \brief Message is requesting an acknowledgment
#define MSG_FLG_ACKRQST			0x08

//! \def MSG_FLG_ACK
//! \brief Message is an acknowledgment
#define MSG_FLG_ACK					0x10

//! \def MSG_FLG_SINGLE
//! \brief Message is not part of a sequence of messages
#define MSG_FLG_SINGLE			0x20

//! \def MSG_FLG_END
//! \brief Message is the end of a sequence of messages
#define MSG_FLG_END					0x40

//! \def MSG_FLG_BEG
//! \brief Message is the beginning of a sequence of messages
#define MSG_FLG_BEG					0x80
//! @}

//! @defgroup Error Messages
//! @{
//! \def SRCLVLERR
//! \brief Source level error flag
#define SRCLVLERR						0x01

//! \def PREXISTLNKERR
//! \brief Pre-existing link error flag
#define PREXISTLNKERR				0x02

//! \def MSGINTGRTYERR
//! \brief Message integrity error flag
#define MSGINTGRTYERR				0x04

//! \def TIMEOUTERR
//! \brief Time out error
#define TIMEOUTERR					0x08
//! @}

//! \def YES_SYNCH
//! \brief Used to tell operational message to add time synch to message
#define YES_SYNCH		1

//! \def NO_SYNCH
//! \brief Used to tell operational message not to add time synch to message
#define NO_SYNCH		0

#define GMH_IDX_EOM_IDX			0

#define GMH_IDX_MSG_TYPE		1
#define GMH_MSG_TYPE_MASK	MAX_MSG_TYPE_MASK

#define GMH_IDX_GID_HI		2
#define GMH_IDX_GID_LO		3

#define GMH_IDX_SRC_SN_HI	4
#define GMH_IDX_SRC_SN_LO	5

#define GMH_IDX_DEST_SN_HI	6
#define GMH_IDX_DEST_SN_LO	7

/* STARTUP MESSAGE TYPE 1 (STARTUP MSG) DEFINES */

#define ST1_IDX_EOM_IDX		0
#define ST1_IDX_MSG_TYPE	1

#define ST1_IDX_GID_HI		2
#define ST1_IDX_GID_LO		3

#define ST1_IDX_SRC_SN_HI	4
#define ST1_IDX_SRC_SN_LO	5

#define ST1_IDX_DEST_SN_HI	6
#define ST1_IDX_DEST_SN_LO	7

#define ST1_IDX_CRC_HI		8
#define ST1_IDX_CRC_LO		9

#define ST1_MSG_LAST_BYTE_NUM_UC   ST1_IDX_CRC_LO
#define ST1_MSG_LAST_BYTE_NUM_UL ((ulong) ST1_MSG_LAST_BYTE_NUM_UC)
#define ST1_MSG_SIZE_IN_BYTES_UL (ST1_MSG_LAST_BYTE_IDX_UL + 1UL)
#define ST1_MSG_SIZE_IN_BITS_UL (ST1_MSG_SIZE_IN_BYTES_UL * 8UL)
#define ST1_MSG_IN_nS_UL (ST1_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

/* DISCOVERY 4 MSG INDEXES */

#define DC4_IDX_EOM_IDX	0
#define DC4_IDX_MSG_TYPE			1

#define DC4_IDX_GID_HI				2
#define DC4_IDX_GID_LO				3

#define DC4_IDX_SRC_SN_HI			4
#define DC4_IDX_SRC_SN_LO			5

#define DC4_IDX_DEST_SN_HI			6
#define DC4_IDX_DEST_SN_LO			7

#define DC4_IDX_SYNC_TIME_XI		8
#define DC4_IDX_SYNC_TIME_HI		9
#define DC4_IDX_SYNC_TIME_MD		10
#define DC4_IDX_SYNC_TIME_LO		11
#define DC4_IDX_SYNC_TIME_SUB_HI	12
#define DC4_IDX_SYNC_TIME_SUB_LO	13

#define DC4_IDX_SRC_LEVEL			14

#define DC4_IDX_HR0_TO_SYSTIM0_IN_SEC_B16	15
#define DC4_IDX_HR0_TO_SYSTIM0_IN_SEC_HI	15
#define DC4_IDX_HR0_TO_SYSTIM0_IN_SEC_LO	16

#define DC4_IDX_CRC_HI				17
#define DC4_IDX_CRC_LO				18

#define DC4_MSG_LAST_BYTE_NUM_UC   DC4_IDX_CRC_LO
#define DC4_MSG_LAST_BYTE_NUM_UL ((ulong) DC4_MSG_LAST_BYTE_NUM_UC)

#define DC4_MSG_SIZE_IN_BYTES_UL (DC4_MSG_LAST_BYTE_NUM_UL + 1UL)
#define DC4_MSG_SIZE_IN_BITS_UL (DC4_MSG_SIZE_IN_BYTES_UL * 8UL)

#define DC4_MSG_IN_nS_UL (DC4_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

#define DC4_PKT_SIZE_IN_BITS_UL (RADIO_LEADER_SIZE_IN_BITS_UL + \
									DC4_MSG_SIZE_IN_BITS_UL)

#define DC4_PKT_IN_TICS_UL ((DC4_PKT_SIZE_IN_BITS_UL * \
								RADIO_BIT_IN_TICS_X_10000_UL) / 10000UL)
#define DC4_PKT_IN_TICS_UI ((uint) DC4_PKT_IN_TICS_UL)

#define DC4_SYNC_IN_TICS_UI  (MSG_ST_DELAY_IN_TICS_UI + \
		RADIO_WARMUP_IN_TICS_UI + DC4_PKT_IN_TICS_UI + RECEIVER_DIGEST_IN_TICS_UI)

#define DC4_SYNC_IN_CLKS_UI      (DC4_SYNC_IN_TICS_UI | 0x8000)

/* DISCOVERY 5 MSG INDEXES */

#define DC5_IDX_EOM_IDX		0
#define DC5_IDX_MSG_TYPE				1

#define DC5_IDX_GID_HI					2
#define DC5_IDX_GID_LO					3

#define DC5_IDX_SRC_SN_HI				4
#define DC5_IDX_SRC_SN_LO				5

#define DC5_IDX_DEST_SN_HI				6
#define DC5_IDX_DEST_SN_LO				7

#define DC5_IDX_SEED_HI					8
#define DC5_IDX_SEED_MD					9
#define DC5_IDX_SEED_LO	   			   10

#define DC5_IDX_CRC_HI				   12
#define DC5_IDX_CRC_LO				   13

#define DC5_MSG_LAST_BYTE_NUM_UC   DC5_IDX_CRC_LO

#define DC5_MSG_LAST_BYTE_IDX_UL ((ulong) DC5_MSG_LAST_BYTE_NUMBER_UC)
#define DC5_MSG_SIZE_IN_BYTES_UL (DC5_MSG_LAST_BYTE_IDX_UL + 1UL)
#define DC5_MSG_SIZE_IN_BITS_UL (DC5_MSG_SIZE_IN_BYTES_UL * 8UL)
#define DC5_MSG_IN_nS_UL (DC5_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

#define DC5_PKT_SIZE_IN_BITS_UL ((RADIO_LEADER_SIZE_IN_BITS_UL + DC5_MSG_SIZE_IN_BITS_UL)
#define DC5_PKT_IN_TICS_UL ((DC5_PKT_SIZE_IN_BITS_UL * RADIO_BIT_IN_TICS_X_10000_UL) / 10000UL)
#define DC5_PKT_IN_TICS_UI ((uint) DC5_PKT_IN_TICS_UL)

/* DISCOVERY  5  CALCULATIONS */

#define DC5_PKT_IN_nS_UL (RADIO_LEADER_IN_nS_UL + DC5_MSG_IN_nS_UL)
#define DC5_SUB_IN_nS_UL (DC5_PKT_IN_nS_UL + SUB_MINUS_PKT_IN_nS_UL)

#define DC5_MAX_SUBS_PER_SEC_UL (1000000000UL / DC5_SUB_IN_nS_UL)

#define DC5_SUBS_PER_SEC_UC 7
#define DC5_SUBS_PER_SEC_UI ((uint)  DC5_SUBS_PER_SEC_UC)
#define DC5_SUBS_PER_SEC_UL ((ulong) DC5_SUBS_PER_SEC_UC)

#define DC5_TOT_SUB_COUNT_UL (MAX_VYING_XMITTERS_UL * REPEATED_XMISSIONS_UL * MEAN_FREE_CAPACITY_UL)
#define DC5_TOT_SUB_COUNT_UI ((uint) DC5_TOT_SUB_COUNT_UL)
#define DC5_TOT_SUB_COUNT_UC ((uchar) DC5_TOT_SUB_COUNT_UI)

#define DC5_SLOT_IN_SEC_UL ((DC5_TOT_SUB_COUNT_UL / DC5_SUBS_PER_SEC_UL) + 1UL)
#define DC5_SLOT_IN_SEC_L  ((long) DC5_SLOT_IN_SEC_UL)
#define DC5_SLOT_IN_SEC_UI ((uint)  DC5_SLOT_IN_SEC_UL)
#define DC5_SLOT_IN_SEC_UC ((uchar) DC5_SLOT_IN_SEC_UI)

#define DC5_SUB_IN_TICS_UI ((uint)(0x8000/DC5_SUBS_PER_SEC_UI))

#define DC5_SUBSLOT_COUNT			16
#define DC5_SUBSLOT_COUNT_MASK		(DC5_SUBSLOT_COUNT-1)

#define DC5_SUBSLOT_WIDTH_IN_ns_UL		(200UL * 1000000UL) //200ms
#define DC5_SUBSLOT_ST_OFFSET_IN_ns_UL	(600UL * 1000000UL) //600ms
/* OPERATIONAL MODE DEFINES */

/* OPERATIONAL MODE 1 INDEXES */

#define OM1_IDX_EOM_IDX		0
#define OM1_MSG_OUTBOUND_MSG_PENDING_BIT 0x80

#define OM1_IDX_MSG_TYPE	1

#define OM1_IDX_GID_HI		2
#define OM1_IDX_GID_LO		3

#define OM1_IDX_SRC_SN_HI	4
#define OM1_IDX_SRC_SN_LO	5

#define OM1_IDX_DEST_SN_HI	6
#define OM1_IDX_DEST_SN_LO	7

#define OM1_IDX_SYNC_TIME_XI		8
#define OM1_IDX_SYNC_TIME_HI		9
#define OM1_IDX_SYNC_TIME_MD	   10
#define OM1_IDX_SYNC_TIME_LO	   11
#define OM1_IDX_SYNC_TIME_SUB_HI   12
#define OM1_IDX_SYNC_TIME_SUB_LO   13

#define OM1_IDX_DELTA_SEC_TO_SLOT_END	14

#define OM1_IDX_AVAIL_MSG_SPACE		15

#define OM1_IDX_CRC_HI				16
#define OM1_IDX_CRC_LO				17

#define OM1_MSG_LAST_BYTE_NUM_UC   OM1_IDX_CRC_LO
#define OM1_MSG_LAST_BYTE_NUM_UL ((ulong) OM1_MSG_LAST_BYTE_NUM_UC)

#define OM1_MSG_SIZE_IN_BYTES_UL (OM1_MSG_LAST_BYTE_NUM_UL + 1UL)

#define OM1_MSG_SIZE_IN_BITS_UL (OM1_MSG_SIZE_IN_BYTES_UL * 8UL)

#define OM1_MSG_IN_nS_UL (OM1_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

#define OM1_PKT_SIZE_IN_BITS_UL (RADIO_LEADER_SIZE_IN_BITS_UL + OM1_MSG_SIZE_IN_BITS_UL)

#define OM1_PKT_IN_TICS_UL ((OM1_PKT_SIZE_IN_BITS_UL * RADIO_BIT_IN_TICS_X_10000_UL) / 10000UL)
#define OM1_PKT_IN_TICS_UI ((uint) OM1_PKT_IN_TICS_UL)

#define OM1_SYNC_IN_TICS_UI  (MSG_ST_DELAY_IN_TICS_UI + \
		RADIO_WARMUP_IN_TICS_UI + OM1_PKT_IN_TICS_UI + RECEIVER_DIGEST_IN_TICS_UI)

#define OM1_SYNC_IN_CLKS_UI      (OM1_SYNC_IN_TICS_UI | 0x8000)

/* OM2 INDEXES */

#define OM2_IDX_EOM_IDX		0
#define OM2_LAST_PKT_BIT	0x80
#define OM2_NODATA_BIT	0x40

#define OM2_IDX_MSG_TYPE	1
#define OM2_MSG_TYPE_MASK	MAX_MSG_TYPE_MASK

#define OM2_IDX_GID_HI		2
#define OM2_IDX_GID_LO		3

#define OM2_IDX_SRC_SN_HI	4
#define OM2_IDX_SRC_SN_LO	5

#define OM2_IDX_DEST_SN_HI	6
#define OM2_IDX_DEST_SN_LO	7

#define OM2_IDX_MSG_SEQ_NUM			 8

//	#define OM2_IDX_LFACTOR				 9	//Dcnt Radio Link Algorithm
//	#define OM2_IDX_LNKREQ				 9	//LnkBlk Radio Link Algorithm
#define OM2_IDX_GENERIC_LINK_BYTE	 9  //A generic name here
#define OM2_IDX_AGENT_NUM_HI		10
#define OM2_IDX_AGENT_NUM_LO		11

#define OM2_IDX_COLLECTION_TIME_XI	12
#define OM2_IDX_COLLECTION_TIME_HI	13
#define OM2_IDX_COLLECTION_TIME_MD	14
#define OM2_IDX_COLLECTION_TIME_LO	15

#define OM2_IDX_DATA_0_SENSOR_NUM	16
#define OM2_IDX_DATA_0_HI			17
#define OM2_IDX_DATA_0_LO			18

#define OM2_IDX_DATA_1_SENSOR_NUM	19
#define OM2_IDX_DATA_1_HI			20
#define OM2_IDX_DATA_1_LO			21

#define OM2_IDX_DATA_2_SENSOR_NUM	22
#define OM2_IDX_DATA_2_HI			23
#define OM2_IDX_DATA_2_LO			24

#define OM2_IDX_DATA_3_SENSOR_NUM	25
#define OM2_IDX_DATA_3_HI			26
#define OM2_IDX_DATA_3_LO			27

/*-----------------------------------------*/
#define OM2_IDX_DATA_4_SENSOR_NUM	28	//short data area only 1 byte
#define OM2_IDX_DATA_4_LO			29  //used to pass msg err info
#define OM2_IDX_CRC_HI				30
#define OM2_IDX_CRC_LO				31

#define OM2_MAX_DATA_ENTRY_COUNT	4
#define OM2_MAX_DBL_DATA_ENTRY_COUNT (OM2_MAX_DATA_ENTRY_COUNT / 2)

#define OM2_MSG_LAST_BYTE_NUM_UC   OM2_IDX_CRC_LO
#define OM2_MSG_LAST_BYTE_NUM_UL ((ulong) OM2_MSG_LAST_BYTE_NUM_UC)

#define OM2_MSG_SIZE_IN_BYTES_UL (OM2_MSG_LAST_BYTE_IDX_UL + 1UL)
#define OM2_MSG_SIZE_IN_BITS_UL (OM2_MSG_SIZE_IN_BYTES_UL * 8UL)

#define OM2_MSG_IN_nS_UL (OM2_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

/* OPERATIONAL MODE 3 (ACK) DEFINES */

#define OM3_IDX_EOM_IDX		0
#define OM3_MSG_LAST_PKT_BAD_BIT	0x80
#define OM3_MSG_EMPTY_PKT_BIT		0x40

#define OM3_IDX_MSG_TYPE	1
#define OM3_MSG_TYPE_MASK	MAX_MSG_TYPE_MASK

#define OM3_IDX_GID_HI		2
#define OM3_IDX_GID_LO		3

#define OM3_IDX_SRC_SN_HI	4
#define OM3_IDX_SRC_SN_LO	5

#define OM3_IDX_DEST_SN_HI	6
#define OM3_IDX_DEST_SN_LO	7

#define OM3_IDX_ACKED_SEQ_NUM	8

//	#define OM3_IDX_LINKUP_DCNT			9 //Dcnt Radio Link Algorithm
//	#define OM3_IDX_LNK_CONFIRM			9 //LnkBlk Radio Link Algorithm
#define OM3_IDX_GENERIC_LINK_RETURN 9 //Generic Link Return Definition
#define OM3_IDX_CRC_HI			10
#define OM3_IDX_CRC_LO			11

#define OM3_MSG_LAST_BYTE_NUM_UC   OM3_IDX_CRC_LO
#define OM3_MSG_LAST_BYTE_NUM_UL ((ulong) OM3_MSG_LAST_BYTE_NUM_UC)

#define OM3_MSG_SIZE_IN_BYTES_UL (OM3_MSG_LAST_BYTE_IDX_UL + 1UL)
#define OM3_MSG_SIZE_IN_BITS_UL (OM3_MSG_SIZE_IN_BYTES_UL * 8UL)
#define OM3_MSG_IN_nS_UL (OM3_MSG_SIZE_IN_BITS_UL * RADIO_BIT_IN_nS_UL)

/* TS1 INDEXES */

#define TS1_IDX_EOM_IDX		0

#define TS1_IDX_MSG_TYPE	1
#define TS1_MSG_TYPE_MASK	MAX_MSG_TYPE_MASK

#define TS1_IDX_GID_HI		2
#define TS1_IDX_GID_LO		3

#define TS1_IDX_SRC_SN_HI	4
#define TS1_IDX_SRC_SN_LO	5

#define TS1_IDX_DEST_SN_HI	6
#define TS1_IDX_DEST_SN_LO	7

#define TS1_IDX_MSG_SEQ_NUM			 8

/* rest of msg is test data to CRC */

#define TS1_IDX_CRC_HI				30
#define TS1_IDX_CRC_LO				31

#define TS1_MSG_LAST_BYTE_NUM_UC   TS1_IDX_CRC_LO

void vComm_PackReportDE(uchar ucSourceID);
uchar ucComm_chkMsgIntegrity(uchar ucCheckByteBits, uchar ucReportByteBits, uchar ucMsgType, uint uiExpectedSrcSN, uint uiExpectedDestSN);
void vComm_showSNmismatch(uint uiExpectedVal, uint uiGotVal, uchar ucCRLF_flag //YES_CRLF, NO_CRLF
    );
void vComm_SendBeacon(void);
void vComm_Request_to_Join(void);
uchar ucComm_waitForMsgOrTimeout(void);

void vComm_Receive_OpMSG(void);
void vComm_Send_OpMSG(void);

/* ROUTINE DEFINITIONS */
uchar ucMSG_chkMsgIntegrity(uchar ucCheckByteBits, uchar ucReportByteBits, uchar ucMsgType, uint uiExpectedSrcSN, uint uiExpectedDestSN);

void vMSG_showSNmismatch(uint uiExpectedVal, uint uiGotVal, uchar ucCRLF_flag //YES_CRLF, NO_CRLF
    );

void vMSG_showStorageErr(const char *cpLeadinMsg, ulong ulFailAddr, ulong ulWroteVal, ulong ulReadVal);
void vMSG_buildMsgHdr_GENERIC( //HDR=Len,Type,Group,Src,Dest
    uchar ucMsgLen, uchar ucMsgType, uint uiDestSN, uchar ucFlags);

uchar ucMSG_waitForMsgOrTimeout(void);
void vMSG_rec_obnd_msgs(void);
void vMSG_showMsgBuffer(uchar ucCRLF_termFlag, //YES_CRLF, NO_CRLF
    uchar ucShowTypeFlag //SHOW_MSG_RAW, SHOW_MSG_COOKED
    );

uchar ucMSG_doSubSecXmit( //RET: 1-sent, 0-too late
    uchar ucaSendTime[6], //full start time
    uchar ucClkChoiceFlag, //USE_CLK2, USE_CLK1
    uchar ucStartRecFlag //YES_RECEIVER_START, NO_RECEIVER_START
    );

void vMSG_stuffFakeMsgToSRAM(void);
void vMSG_checkFakeMsg(void);
uchar ucMSG_incMsgSeqNum( //RET: Incremented Msg Seq Num (not 0 or 255)
    void);
uchar ucMSG_getLastFilledEntryInOM2( //RET: 0=none, ##=idx of last entry in OM2
    void);

//! @}
#endif /* COMM_H_INCLUDED */

/* --------------------------  END of MODULE  ------------------------------- */
