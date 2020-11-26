
/*********************************************************************
 *
 *            J1939 Include File
 *
 *********************************************************************
 * FileName:        J1939.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 X.X
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Constants that are used by the J1939 C Library.
 *
 * Version     Date        Description
 * ----------------------------------------------------------------------
 * v01.00.00   2004/06/04  Initial Release
 *
 * Copyright 2004 Kimberly Otten Software Consulting
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Kim Otten            6/04/04     Original
 * Caio Gubel           6/11/04     Formatted
 ********************************************************************/

#include        <p18cxxx.h>
#include        "j1939.def"

#ifndef __J1939_H
#define __J1939_H


typedef enum _BOOL { FALSE = 0, TRUE } BOOL;


// Function Return Codes

#define RC_SUCCESS			0
#define RC_QUEUEEMPTY			1
#define RC_QUEUEFULL			1
#define RC_CANNOTRECEIVE		2
#define RC_CANNOTTRANSMIT		2
#define RC_PARAMERROR			3


// Internal Constants

#define J1939_FALSE				0
#define J1939_TRUE				1


// J1939 Default Priorities

#define J1939_CONTROL_PRIORITY			0x03
#define J1939_INFO_PRIORITY			0x06
#define J1939_PROPRIETARY_PRIORITY		0x06
#define J1939_REQUEST_PRIORITY			0x06
#define J1939_ACK_PRIORITY			0x06
#define J1939_TP_CM_PRIORITY			0x07
#define J1939_TP_DT_PRIORITY			0x07


// J1939 Defined Addresses

#define J1939_GLOBAL_ADDRESS			255
#define J1939_NULL_ADDRESS			254


// Some J1939 PDU Formats, Control Bytes, and PGN's

#define J1939_PF_REQUEST2			201
#define J1939_PF_TRANSFER			202

#define J1939_PF_ACKNOWLEDGMENT			232
#define J1939_ACK_CONTROL_BYTE			0
#define J1939_NACK_CONTROL_BYTE			1
#define J1939_ACCESS_DENIED_CONTROL_BYTE	2
#define J1939_CANNOT_RESPOND_CONTROL_BYTE	3

#define J1939_PF_REQUEST			234

#define J1939_PF_DT				235		// Data Transfer message

#define J1939_PF_TP_CM				236		// Connection Management message
#define J1939_RTS_CONTROL_BYTE			16		// Request to Send control byte of CM message
#define J1939_CTS_CONTROL_BYTE			17		// Clear to Send control byte of CM message
#define J1939_EOMACK_CONTROL_BYTE		19		// End of Message control byte of CM message
#define J1939_BAM_CONTROL_BYTE			32		// BAM control byte of CM message
#define J1939_CONNABORT_CONTROL_BYTE		255		// Connection Abort control byte of CM message

#define J1939_PGN2_REQ_ADDRESS_CLAIM		0x00
#define J1939_PGN1_REQ_ADDRESS_CLAIM		0xEA
#define J1939_PGN0_REQ_ADDRESS_CLAIM		0x00

#define J1939_PGN2_COMMANDED_ADDRESS		0x00
#define J1939_PGN1_COMMANDED_ADDRESS		0xFE	        // (-81 PDU Format)
#define J1939_PGN0_COMMANDED_ADDRESS		0xD8	        // (-81 PDU Specific)

#define J1939_PF_ADDRESS_CLAIMED		238		// With global address
#define J1939_PF_CANNOT_CLAIM_ADDRESS		238		// With null address
#define J1939_PF_PROPRIETARY_A			239
#define J1939_PF_PROPRIETARY_B			255


// J1939 Data Structures

// The J1939_MESSAGE_STRUCT is designed to map the J1939 messages pieces
// into the address map for the device.  Only the field PDU Format does
// not cleanly map into the device registers.  Users of the structure
// should simply use the field PDUFormat and ignore PDUFormat_Top.  Adjustments
// will be made immediately upon reception and just prior to transmission.

// Note: The compiler creates structures from low bit position to high bit
// position, so the order may appear not to match the device registers.

#define J1939_MSG_LENGTH	5
#define J1939_DATA_LENGTH	8


union J1939_MESSAGE_UNION {
	struct {
		unsigned int	PDUFormat_Top		: 3;	// This needs pre and post processing.
		unsigned int	DataPage			: 1;
		unsigned int	Res					: 1;
		unsigned int	Priority			: 3;
		unsigned char	PDUFormat;					// CA should use only PDUFormat.
		unsigned char	PDUSpecific;
		unsigned char	SourceAddress;
		unsigned int	DataLength 			: 4;
		unsigned int	RTR					: 4;	// RTR bit, value always 0x00
		unsigned char	Data[J1939_DATA_LENGTH];
	};
	unsigned char		Array[J1939_MSG_LENGTH + J1939_DATA_LENGTH];
};
#define GroupExtension 		PDUSpecific
#define DestinationAddress 	PDUSpecific
typedef union J1939_MESSAGE_UNION J1939_MESSAGE;


union J1939_FLAGS_UNION {
	struct {
		unsigned int	CannotClaimAddress				: 1;
		unsigned int	WaitingForAddressClaimContention: 1;
		unsigned int	GettingCommandedAddress			: 1;
		unsigned int	GotFirstDataPacket				: 1;
		unsigned int	ReceivedMessagesDropped			: 1; };
	unsigned char		FlagVal;
};
typedef union J1939_FLAGS_UNION J1939_FLAG;


// If we're using older devices, the port pins that we need to configure
// are different, and we have to use Legacy Mode.  Set up a single
// #define for indicating that we're using a device with a different pin-out.

#if   defined(__18F248)
	#define ECAN_IS_CAN_MODULE
#elif defined(__18F258)
	#define ECAN_IS_CAN_MODULE
#elif defined(__18F448)
	#define ECAN_IS_CAN_MODULE
#elif defined(__18F458)
	#define ECAN_IS_CAN_MODULE
#endif

// Make sure the user didn't forget to change the mode to Legacy Mode

#ifdef ECAN_IS_CAN_MODULE
	#undef ECAN_LEGACY_MODE
	#define ECAN_LEGACY_MODE	J1939_TRUE
#endif

// Set up various definitions based on the extra buffer configuration
// if we're using FIFO mode.  ECAN_CONFIGURE_BUFFERS is the initialization
// value for BSEL0 to configure the extra buffers as either transmit or
// receive.  ECAN_BUFFER_INTERRUPT_ENABLE is the value to put into BIE0 to
// enable the interrupts on the extra receive buffers (note that RXB0IE and
// RXB1IE are always set).  BUFFER_TABLE defines the various values needed
// to set up windowing and interrupts for the various transmit and receive
// configurations.  Note that some of these values are not needed if
// interrupts are not used or if the number of buffers is reduced, so the
// tables are reduced accordingly.

#if ECAN_LEGACY_MODE == J1939_TRUE
	#define ECAN_MAX_TX_BUFFERS					2

	#if J1939_POLL_ECAN == J1939_TRUE
		struct TX_BUFFER_INFO_STRUCT {
			unsigned char	WindowBits;	};
		static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
			{ 0x06 },		// TXB1
			{ 0x08 }};		// TXB0
	#else
		struct TX_BUFFER_INFO_STRUCT {
			unsigned char	PIE3Val;
			unsigned char	WindowBits;	};
		static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
			{ 0xA8, 0x06 },		// TXB1
			{ 0xA4, 0x08 }};	// TXB0
	#endif
#else
	#define ECAN_MAX_TX_BUFFERS					(2+(6-ECAN_EXTRA_RX_BUFFERS))

	#if J1939_POLL_ECAN == J1939_TRUE
		struct TX_BUFFER_INFO_STRUCT {
			unsigned char	WindowBits;	};
	#else
		struct TX_BUFFER_INFO_STRUCT {
			unsigned char	TXBIEVal;
			unsigned char	BIE0Val;
			unsigned char	WindowBits;	};
	#endif

	#if ECAN_EXTRA_RX_BUFFERS == 0
		#define ECAN_CONFIGURE_BUFFERS			0xFC
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x03
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 },	// B5
				{ 0x96 },	// B4
				{ 0x95 },	// B3
				{ 0x94 },	// B2
				{ 0x93 },	// B1
				{ 0x92 }};	// B0
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 },	// B5
				{ 0x00, 0x43, 0x96 },	// B4
				{ 0x00, 0x23, 0x95 },	// B3
				{ 0x00, 0x13, 0x94 },	// B2
				{ 0x00, 0x0B, 0x93 },	// B1
				{ 0x00, 0x07, 0x92 }};	// B0
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 1
		#define ECAN_CONFIGURE_BUFFERS			0xF8
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x07
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 },	// B5
				{ 0x96 },	// B4
				{ 0x95 },	// B3
				{ 0x94 },	// B2
				{ 0x93 }};	// B1
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 },	// B5
				{ 0x00, 0x43, 0x96 },	// B4
				{ 0x00, 0x23, 0x95 },	// B3
				{ 0x00, 0x13, 0x94 },	// B2
				{ 0x00, 0x0B, 0x93 }};	// B1
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 2
		#define ECAN_CONFIGURE_BUFFERS			0xF0
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x0F
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 },	// B5
				{ 0x96 },	// B4
				{ 0x95 },	// B3
				{ 0x94 }};	// B2
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 },	// B5
				{ 0x00, 0x43, 0x96 },	// B4
				{ 0x00, 0x23, 0x95 },	// B3
				{ 0x00, 0x13, 0x94 }};	// B2
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 3
		#define ECAN_CONFIGURE_BUFFERS			0xE0
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x1F
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 },	// B5
				{ 0x96 },	// B4
				{ 0x95 }};	// B3
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 },	// B5
				{ 0x00, 0x43, 0x96 },	// B4
				{ 0x00, 0x23, 0x95 }};	// B3
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 4
		#define ECAN_CONFIGURE_BUFFERS			0xC0
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x3F
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 },	// B5
				{ 0x96 }};	// B4
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 },	// B5
				{ 0x00, 0x43, 0x96 }};	// B4
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 5
		#define ECAN_CONFIGURE_BUFFERS			0x80
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0x7F
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 },	// TXB0
				{ 0x97 }};	// B5
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 },	// TXB0
				{ 0x00, 0x83, 0x97 }};	// B5
		#endif
	#elif ECAN_EXTRA_RX_BUFFERS == 6
		#define ECAN_CONFIGURE_BUFFERS			0x00
		#define ECAN_BUFFER_INTERRUPT_ENABLE	0xFF
		#if J1939_POLL_ECAN == J1939_TRUE
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x84 },	// TXB1
				{ 0x83 }};	// TXB0
		#else
			static rom struct TX_BUFFER_INFO_STRUCT BUFFER_TABLE[ECAN_MAX_TX_BUFFERS] = {
				{ 0x08, 0x03, 0x84 },	// TXB1
				{ 0x04, 0x03, 0x83 }};	// TXB0
		#endif
	#endif
#endif


// Give visibility to the global variables.

extern unsigned char	CA_Name[J1939_DATA_LENGTH];
extern unsigned char 	J1939_Address;
extern J1939_FLAG    	J1939_Flags;
extern unsigned char	RXQueueCount;


// Library function prototypes


#ifdef J1939_ACCEPT_CMDADD
void			J1939_CommandedAddressHandling( void );
#endif
unsigned char	        J1939_DequeueMessage( J1939_MESSAGE *MsgPtr );
unsigned char  	        J1939_EnqueueMessage( J1939_MESSAGE *MsgPtr );
void 			J1939_Initialization( BOOL );
void			J1939_ISR( void );
void 			J1939_Poll( unsigned long ElapsedTime );

#ifdef                  __J1939_SOURCE
static void 		J1939_ReceiveMessages( void );
static void 		J1939_RequestForAddressClaimHandling( void );
static unsigned char 	J1939_TransmitMessages( void );
static void 		J1939_AddressClaimHandling( unsigned char Mode );
#endif

#endif


