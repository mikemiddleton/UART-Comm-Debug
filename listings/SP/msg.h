///////////////////////////////////////////////////////////////////////////////
//! \file msg.h
//! \brief The file contains the Messages module
//!
//! @addtogroup core
//! @{
//!
//! @addtogroup msg Messages
//! The Messages Module (msg.h) contains all of the data structures and defines
//! to communicate back and forth between the SP Board boards and the brain
//! board
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wireless Networks Research Lab
//     Dept Of Electrical Engineering, CEFNS
//     Northern Arizona University
//
//*****************************************************************************

#ifndef MSG_H_
  #define MSG_H_
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
  //! \def HAND_SHK (Test Packet)
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
  //!
  #define COMMAND_PKT   	0x01

  //! \def REPORT_DATA
  //! \brief This packet reports a transducer measurement to the CP Board
  //!
  //! This packet is only sent from the SP Board board to the CP Board. The
  //! transducer measurement is contained in data1 through data 8.
  //! The SP sends a 128 (1-8) bit or 32 (1-2) bit data packet, depending on the SP function
  #define REPORT_DATA     0x02

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
  #define REQUEST_DATA    0x04

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
  #define ID_PKT          0x06

  //! \def CONFIRM_COMMAND
  //! \brief This packet is used by the SP Board to confirm that the COMMAND_PKT
  //! sent by the CP Board is valid and that the commands will be executed.
  //!
  #define CONFIRM_COMMAND   0x07

  //! \def REPORT_ERROR
  //! \brief This packet is used by the SP Board to report an error to the CP.
  //!
  //! The SP sends a 32/128 bit packet to the CP to report an error. An error is
  //! if ANY one thing in the transducer function didn't work.
  //!
  #define REPORT_ERROR   	0x08

  //! \def REQUEST_BSL_PW
  //! \brief This packet is used by the CP board to request the bootstrap loader password
  //!
  //! The SP will look into flash address 0xFFE0 to 0xFFFF and send the contents to the 
  //! CP board.
  //!
  #define REQUEST_BSL_PW   0x09

  //! \def INTERROGATE
  //! \brief This packet is used by the CP board to request the transducer information
  //!
  //!
  #define INTERROGATE   0x0A

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
  //! @}

  //! \brief Structure of a SP Data Message
  //!
  //! This structure is only used to define the fields in SP_32BitDataMessage.
  struct SP_32BitDataMessage_Fields
  {
    uint8 ucMsgType;          //!< Type of the message.
    uint8 ucMsgSize;          //!< Size of the message
    uint8 ucMsgVersion;       //!< The version number of the message protocol
    uint8 ucSensorNumber;     //!< Sensor number.
    uint8 ucData1_HI_BYTE;    //!< High byte of the first data field
    uint8 ucData1_LO_BYTE;    //!< Low byte of the first data field
    uint8 ucData2_HI_BYTE;    //!< High byte of the second data field
    uint8 ucData2_LO_BYTE;    //!< Low byte of the second data field
  };



  //! \brief Structure of a SP Data Message -scb
   //!
   //! Needed 128 bits of data to be sent
   //! This structure is only used to define the fields in SP_128BitDataMessage.
  struct SP_128BitDataMessage_Fields
  {
    uint8 ucMsgType;          //!< Type of the message.
    uint8 ucMsgSize;          //!< Size of the message
    uint8 ucMsgVersion;       //!< The version number of the message protocol
    uint8 ucSensorNumber;     //!< Sensor number.
    uint8 ucData1_HI_BYTE;    //!< High byte of the 1st data field
    uint8 ucData1_LO_BYTE;    //!< Low byte of the 1st data field
    uint8 ucData2_HI_BYTE;    //!< High byte of the 2nd data field
    uint8 ucData2_LO_BYTE;    //!< Low byte of the 2nd data field
    uint8 ucData3_HI_BYTE;    //!< High byte of the 3rd data field
    uint8 ucData3_LO_BYTE;    //!< Low byte of the 3rd data field
    uint8 ucData4_HI_BYTE;    //!< High byte of the 4th data field
    uint8 ucData4_LO_BYTE;    //!< Low byte of the 4th data field
    uint8 ucData5_HI_BYTE;    //!< High byte of the 5th data field
    uint8 ucData5_LO_BYTE;    //!< Low byte of the 5th data field
    uint8 ucData6_HI_BYTE;    //!< High byte of the 6th data field
    uint8 ucData6_LO_BYTE;    //!< Low byte of the 6th data field
    uint8 ucData7_HI_BYTE;    //!< High byte of the 7th data field
    uint8 ucData7_LO_BYTE;    //!< Low byte of the 7th data field
    uint8 ucData8_HI_BYTE;    //!< High byte of the 8th data field
    uint8 ucData8_LO_BYTE;    //!< Low byte of the 8th data field
  };

  //! \brief Structure of a SP Bootsrap Loader Password Message 
   //!
   //! 
   //!
  struct SP_256BitData_Message_Fields
  {
    uint8 ucMsgType;          //!< Type of the message.
    uint8 ucMsgSize;          //!< Size of the message
    uint8 ucMsgVersion;       //!< The version number of the message protocol
    uint8 ucSensorNumber;     //!< Sensor number.
    uint8 ucData1_HI_BYTE;    //!< High byte of the 1st data field
    uint8 ucData1_LO_BYTE;    //!< Low byte of the 1st data field
    uint8 ucData2_HI_BYTE;    //!< High byte of the 2nd data field
    uint8 ucData2_LO_BYTE;    //!< Low byte of the 2nd data field
    uint8 ucData3_HI_BYTE;    //!< High byte of the 3rd data field
    uint8 ucData3_LO_BYTE;    //!< Low byte of the 3rd data field
    uint8 ucData4_HI_BYTE;    //!< High byte of the 4th data field
    uint8 ucData4_LO_BYTE;    //!< Low byte of the 4th data field
    uint8 ucData5_HI_BYTE;    //!< High byte of the 5th data field
    uint8 ucData5_LO_BYTE;    //!< Low byte of the 5th data field
    uint8 ucData6_HI_BYTE;    //!< High byte of the 6th data field
    uint8 ucData6_LO_BYTE;    //!< Low byte of the 6th data field
    uint8 ucData7_HI_BYTE;    //!< High byte of the 7th data field
    uint8 ucData7_LO_BYTE;    //!< Low byte of the 7th data field
    uint8 ucData8_HI_BYTE;    //!< High byte of the 8th data field
    uint8 ucData8_LO_BYTE;    //!< Low byte of the 8th data field
    uint8 ucData9_HI_BYTE;    //!< High byte of the 9th data field
    uint8 ucData9_LO_BYTE;    //!< Low byte of the 9th data field
    uint8 ucData10_HI_BYTE;    //!< High byte of the 10th data field
    uint8 ucData10_LO_BYTE;    //!< Low byte of the 10th data field
    uint8 ucData11_HI_BYTE;    //!< High byte of the 11th data field
    uint8 ucData11_LO_BYTE;    //!< Low byte of the 11th data field
    uint8 ucData12_HI_BYTE;    //!< High byte of the 12th data field
    uint8 ucData12_LO_BYTE;    //!< Low byte of the 12th data field
    uint8 ucData13_HI_BYTE;    //!< High byte of the 13th data field
    uint8 ucData13_LO_BYTE;    //!< Low byte of the 13th data field
    uint8 ucData14_HI_BYTE;    //!< High byte of the 14th data field
    uint8 ucData14_LO_BYTE;    //!< Low byte of the 14th data field
    uint8 ucData15_HI_BYTE;    //!< High byte of the 15th data field
    uint8 ucData15_LO_BYTE;    //!< Low byte of the 15th data field
    uint8 ucData16_HI_BYTE;    //!< High byte of the 16th data field
    uint8 ucData16_LO_BYTE;    //!< Low byte of the 16th data field
  };

  //! @name Core Data Array
  //! This array is where the information passed between the transducer function
  //! and the core stored.
  //! @var g_unaCoreData

  //uint16 g_unaCoreData[8];

  //! \def SP_32BITDATAMESSAGE_SIZE
  //! \brief The size of a data message (in bytes).
  #define SP_32BITDATAMESSAGE_SIZE sizeof(struct SP_32BitDataMessage_Fields)


  //! \def SP_128BITDATAMESSAGE_SIZE -scb
  //! \brief The size of a data return message (in bytes).
  #define SP_128BITDATAMESSAGE_SIZE sizeof(struct SP_128BitDataMessage_Fields)


  //! \def SP_256BITDATAMESSAGE_SIZE -scb
  //! \brief The size of a data return message (in bytes).
  #define SP_256BITDATAMESSAGE_SIZE sizeof(struct SP_256BitData_Message_Fields)

  //! \brief This is the union used to work with data messages
  //!
  //! This union allows us to work with the variuos fields in the data message
  //! easily while also making it easy send and receive using byte oriented
  //! communication methods
  union SP_32BitDataMessage
  {
    uint8 ucByteStream[SP_32BITDATAMESSAGE_SIZE];
    struct SP_32BitDataMessage_Fields fields;
  };
  //! @}


  //! \brief This is the union used to work with data return messages -scb
  //!
  //! This union allows us to work with the various fields in the data return message
  //! easily while also making it easy send and receive using byte oriented
  //! communication methods
  union SP_128BitDataMessage
  {
    uint8 ucByteStream[SP_128BITDATAMESSAGE_SIZE];
    struct SP_128BitDataMessage_Fields fields;
  };
  
  //! \brief This is a union to work with the BSL password message
  //!
  //! This union allows us to work with the various fields in the data return message
  //! easily while also making it easy send and receive using byte oriented
  //! communication methods 
  union SP_256BitDataMessage
  {
    uint8 ucByteStream[SP_256BITDATAMESSAGE_SIZE];
    struct SP_256BitData_Message_Fields fields;
  };
  //! @}


  //****************  SP Board Label Message  *********************************//
  //! @defgroup msg_label SP Board Label Message
  //! The SP Board Label Message is used to send labels (byte strings) to the
  //! CP board. The label message is only ever sent in reponse to a
  //! LABEL_REQUEST Data Message. (See \ref msg_data).
  //! @{
  #define SP_LABELMESSAGE_VERSION 102        //!< v1.02

  // Message Types
  //! \name Label Message Types
  //! These are the possible Label Message types
  //! @{
  //! \def REPORT_LABEL
  //! \brief This packet contains a label indicated by the sensor number.
  //!
  //! This type of label packet is only sent by the SP Board board in response
  //! to a LABEL_REQUEST Data Message. It has a different packet length and
  //! is longer than a Data Message and therefore can be receieved as a
  //! Data Message in error.
  #define REPORT_LABEL             0x0A
  //! @}

  // Sensor Types
  //! \name Label Message Sensor Numbers
  //! These are the possible sensor numbers for Label Messages. While these
  //! do overlap with the sensor numbers in \ref msg_data it is important
  //! to use the right sensor number with the right packet incase one of the
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

  //! \brief Structure of a SP Board Label Message
  //!
  //! This structure is only used inside of the SP_LabelMessage. It
  //! should never be allocated on it's own.
  struct SP_LabelMessage_Fields
  {
    uint8 ucMsgType;          //!< Type of the message.
    uint8 ucMsgSize;          //!< Size of the message
    uint8 ucMsgVersion;       //!< The version number of the message protocol
    uint8 ucSensorNumber;          //!< Sensor Number
    uint8 ucaDescription[TRANSDUCER_LABEL_LEN];//!< 16-byte char array for the label
  };

  //! \def SP_LABELMESSAGE_SIZE
  //! \brief The size of the SP Board Label Message (in bytes).
  #define SP_LABELMESSAGE_SIZE sizeof(struct SP_LabelMessage_Fields)

  //! \brief The union used to work with label messages
  //!
  //! This union is what should be allocated to work with label messages. It
  //! allows easy access into the variuos fields of the message and makes it
  //! easy to send and receive using byte oriented communication methods.
  union SP_LabelMessage
  {
    uint8 ucByteStream[SP_LABELMESSAGE_SIZE];
    struct SP_LabelMessage_Fields fields;
  };
  //! @}

#endif /*MSG_H_*/
//! @}
//! @}

