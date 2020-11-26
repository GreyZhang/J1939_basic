/*********************************************************************
 *
 *            J1939 Main Source Code
 *
 *********************************************************************
 * FileName:        J1939.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18xx8x and PIC18xx8
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
 *
 * Library routines for the J1939 C Library for PIC18
 * devices with ECAN module.  Please refer to the J1939 C Library User Guide
 * for information on configuring and using this library.
 *
 * This file requires the following header file:
 *
 * 	p18cxxx.h (MPLAB C18)
 * 	j1939.h
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

#ifndef         __J1939_SOURCE
#define         __J1939_SOURCE
#endif

#include <p18cxxx.h>
#include "J1939.H"

#define J1939_TRUE	1
#define J1939_FALSE	0


// Internal definitions

#define ADDRESS_CLAIM_TX			1
#define ADDRESS_CLAIM_RX			2
#define ECAN_CONFIG_MODE			0x80
#define ECAN_ERROR_INT_ENABLE		0xA0
#define ECAN_NORMAL_MODE			0x00
#define ECAN_RX_INT_ENABLE_FIFO		0x02
#define ECAN_RX_INT_ENABLE_LEGACY	0x03
#define ECAN_SELECT_RX_BUFFER		0x10
#define ECAN_SET_LEGACY_MODE		0x00
#define ECAN_SET_FIFO_MODE			0xA0
#define ECAN_TX_INT_ENABLE_LEGACY	0x0C

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;


// Since we'll be mapping the various buffers into the access bank, we'll
// create definitions for the key registers and bits of the overlayed mapping,
// so it doesn't look like the wrong registers are being used.

#define MAPPED_CONbits		RXB0CONbits
#define MAPPED_CON			RXB0CON
#define MAPPED_SIDH			RXB0SIDH
#ifdef ECAN_IS_CAN_MODULE
	#define MAPPED_TXREQ	RXRTRRO
#else
	#define MAPPED_TXREQ	FILHIT3
#endif


// Errata DS80162B, section 7

#define FIFOEMPTY_MASK		0x80
#define FIFOEMPTY_READ_BIT	FIFOEMPTY
#define FIFOEMPTY_WRITE_BIT	RXBnOVFL


// Define a macro for setting the window address bits for the network
// management transmit buffer (TXB2).
#if ECAN_LEGACY_MODE == J1939_TRUE
	#define SET_NETWORK_WINDOW_BITS {CANCON = ECAN_NORMAL_MODE | 0x04;}
#else
	#define	SET_NETWORK_WINDOW_BITS {ECANCON = ECAN_SET_FIFO_MODE | 0x05;}
#endif


// Global variables.  Some of these will be visible to the CA.

unsigned char					CA_Name[J1939_DATA_LENGTH];
unsigned char 					CommandedAddress;
#if J1939_ACCEPT_CMDADD == J1939_TRUE
	unsigned char				CommandedAddressSource;
	unsigned char 				CommandedAddressName[J1939_DATA_LENGTH];
#endif
unsigned long 					ContentionWaitTime;
unsigned char 					J1939_Address;
J1939_FLAG    					J1939_Flags;
J1939_MESSAGE 					OneMessage;

unsigned char 					RXHead;
unsigned char 					RXTail;
unsigned char 					RXQueueCount;
J1939_MESSAGE 					RXQueue[J1939_RX_QUEUE_SIZE];

unsigned char 					TXHead;
unsigned char 					TXTail;
unsigned char 					TXQueueCount;
J1939_MESSAGE 					TXQueue[J1939_TX_QUEUE_SIZE];

#if ECAN_LEGACY_MODE
	unsigned char				TXIntsEnabled;
#endif

// Function Prototypes

#if J1939_ACCEPT_CMDADD == J1939_TRUE
	BOOL CA_AcceptCommandedAddress( void );
#endif

#if J1939_ARBITRARY_ADDRESS != 0x00
	BOOL CA_RecalculateAddress( unsigned char * );
#endif

/*********************************************************************
CompareName

This routine compares the passed in array data NAME with the CA's
current NAME as stored in CA_Name.

Parameters:	unsigned char *		Array of NAME bytes
Return:		-1 - CA_Name is less than the data
		 	0  - CA_Name is equal to the data
			1  - CA_Name is greater than the data
*********************************************************************/
signed char CompareName( unsigned char *OtherName )
{
	unsigned char	i;

	for (i = 0; (i<J1939_DATA_LENGTH) && (OtherName[i] == CA_Name[i]); i++);

	if (i == J1939_DATA_LENGTH)
		return 0;
	else if (CA_Name[i] < OtherName[i] )
		return -1;
	else
		return 1;
}

/*********************************************************************
CopyName

This routine copies the CA's NAME into the message buffer's data array.
We can afford to make this a function, since another function is always
called after this is done, and we won't be using any additional stack
space.

Parameters:	None
Return:		None
*********************************************************************/
void CopyName(void)
{
	unsigned char i;

	for (i=0; i<J1939_DATA_LENGTH; i++)
		OneMessage.Data[i] = CA_Name[i];
}

/*********************************************************************
SetECANMode

This routine sets the ECAN module to a specific mode.  It requests the
mode, and then waits until the mode is actually set.

Parameters:	unsigned char	ECAN module mode.  Must be either
							ECAN_CONFIG_MODE or ECAN_NORMAL_MODE
Return:		None
*********************************************************************/
void SetECANMode( unsigned char Mode )
{
	CANCON = Mode;
	while ((CANSTAT & 0xE0) != Mode);
}

/*********************************************************************
SetAddressFilter

This routine sets filter 3 to the specified value (destination address).
It is used to allow reception of messages sent to this node specifically
or simply to the global address if this node does not have an address
(Address will be J1939_GLOBAL_ADDRESS).

Parameters:	unsigned char	J1939 Address of this CA (or global)
Return:		None
*********************************************************************/
void SetAddressFilter( unsigned char Address )
{
	SetECANMode( ECAN_CONFIG_MODE );
	RXF3EIDH = Address;
	SetECANMode( ECAN_NORMAL_MODE );
}

/*********************************************************************
SendOneMessage

This routine sends the message located at the pointer passed in.  It
also uses the message's data length field to determine how much of the
data to load.  At this point, all of the data fields, such as data
length, priority, and source address, must be set.  This routine will
set up the CAN bits, such as the extended identifier bit and the
remote transmission request bit.  The window address bits for the
correct operational mode must be set before this routine is called.

Parameters:	J1939_MESSAGE far *		Pointer to message to send
Return:		None
*********************************************************************/
void SendOneMessage( J1939_MESSAGE *MsgPtr )
{
	unsigned char Loop;
	unsigned char *RegPtr;
	unsigned char Temp;

	// Set up the final pieces of the message and make sure DataLength isn't
	// out of spec.

	MsgPtr->Res = 0;
	MsgPtr->RTR = 0;
	if (MsgPtr->DataLength > 8)
		MsgPtr->DataLength = 8;

	// Put PDUFormat into the structure to match the J1939-CAN format.  This
	// involves splitting the original value in PDUFormat into two pieces,
	// leaving some holes for the TXBnSIDL register, and setting the EXIDE bit.

	MsgPtr->PDUFormat_Top = MsgPtr->PDUFormat >> 5;		// Put the top three bits into SID5-3
	Temp = MsgPtr->PDUFormat & 0x03;					// Save the bottom two bits.
	MsgPtr->PDUFormat = (MsgPtr->PDUFormat & 0x1C) << 3;// Move up bits 4-2 into SID2-0.
	MsgPtr->PDUFormat |= Temp | 0x08;					// Put back EID17-16, set EXIDE.

	// Wait until the requested buffer can be used to transmit.  We shouldn't
	// need a time-out here unless something else in the design isn't working
	// (or we have to send a LOT of network management messages).

	while (MAPPED_CONbits.MAPPED_TXREQ);

	// Load the message buffer.  Load the first 5 bytes of the message,
	// then load whatever part of the data is necessary.

	RegPtr = &MAPPED_SIDH;
	for (Loop=0; Loop<J1939_MSG_LENGTH+MsgPtr->DataLength;  Loop++, RegPtr++)
		*RegPtr = MsgPtr->Array[Loop];

	// Now tell the module to send the message.

	MAPPED_CONbits.MAPPED_TXREQ = 1;
}

/*********************************************************************
J1939_AddressClaimHandling

This routine is called either when the CA must claim its address on
the bus or another CA is trying to claim the same address on the bus and
we must either defend ourself or relinquish the address.  If the CA has
an address in the proprietary range of 0-127 or 248-253, it can take
the address immediately.

Parameters:	unsigned char	ADDRESS_CLAIM_RX indicates an Address
							Claim message has been received and this
							CA must either defend or give up its
							address.
							ADDRESS_CLAIM_TX indicates that the CA is
							initiating a claim to its address.
Return:		None
*********************************************************************/
static void J1939_AddressClaimHandling( unsigned char Mode )
{
	// Get most of the message ready here, since it'll be very similar
	// for both messages to be sent.  Note that J1939_PF_ADDRESS_CLAIMED
	// is the same value as J1939_PF_CANNOT_CLAIM_ADDRESS.  We can't copy
	// the data yet because we might need to look at the old data.

	OneMessage.Priority = J1939_CONTROL_PRIORITY;
	OneMessage.PDUFormat = J1939_PF_ADDRESS_CLAIMED;
	OneMessage.DestinationAddress = J1939_GLOBAL_ADDRESS;
	OneMessage.DataLength = J1939_DATA_LENGTH;

	if (Mode == ADDRESS_CLAIM_TX)
		goto SendAddressClaim;

	if (OneMessage.SourceAddress != J1939_Address)
		return;

	if (CompareName( OneMessage.Data ) != -1) // Our CA_Name is not less
	{
		#if J1939_ARBITRARY_ADDRESS != 0x00
			if (CA_RecalculateAddress( &CommandedAddress ))
				goto SendAddressClaim;
		#endif

		// Send Cannot Claim Address message
		CopyName();
		OneMessage.SourceAddress = J1939_NULL_ADDRESS;
		SET_NETWORK_WINDOW_BITS;
		SendOneMessage( (J1939_MESSAGE *) &OneMessage );

		// Set up filter to receive messages sent to the global address
		SetAddressFilter( J1939_GLOBAL_ADDRESS );

		J1939_Flags.CannotClaimAddress = 1;
		J1939_Flags.WaitingForAddressClaimContention = 0;
		return;
	}

SendAddressClaim:
	// Send Address Claim message for CommandedAddress
	CopyName();
	OneMessage.SourceAddress = CommandedAddress;
	SET_NETWORK_WINDOW_BITS;
	SendOneMessage( (J1939_MESSAGE *) &OneMessage );

	if (((CommandedAddress & 0x80) == 0) ||			// Addresses 0-127
		((CommandedAddress & 0xF8) == 0xF8))		// Addresses 248-253 (254,255 illegal)
	{
		J1939_Flags.CannotClaimAddress = 0;
		J1939_Address = CommandedAddress;

		// Set up filter to receive messages sent to this address
		SetAddressFilter( J1939_Address );
	}
	else
	{
		// We don't have a proprietary address, so we need to wait.
 		J1939_Flags.WaitingForAddressClaimContention = 1;
		ContentionWaitTime = 0;
	}
}

/*********************************************************************
J1939_DequeueMessage

This routine takes a message from the receive queue and places it in
the caller's buffer.  If there is no message to return, an appropriate
return code is returned.  If we're using interrupts, disable the
receive interrupt around the queue manipulation.

Parameters:	J1939_MESSAGE *		Pointer to the caller's message buffer
Return:		RC_SUCCESS			Message dequeued successfully
			RC_QUEUEEMPTY		No messages to return
			RC_CANNOTRECEIVE	System cannot currently receive
						messages.  This will be returned only
						after the receive queue is empty.
*********************************************************************/
unsigned char J1939_DequeueMessage( J1939_MESSAGE *MsgPtr )
{
	unsigned char	rc = RC_SUCCESS;

	#if J1939_POLL_ECAN == J1939_FALSE
		#if ECAN_LEGACY_MODE == J1939_TRUE
			PIE3 &= ~ECAN_RX_INT_ENABLE_LEGACY;
		#else
			PIE3bits.RXBnIE = 0;
		#endif
	#endif

	if (RXQueueCount == 0)
	{
		if (J1939_Flags.CannotClaimAddress)
			rc = RC_CANNOTRECEIVE;
		else
			rc = RC_QUEUEEMPTY;
	}
	else
	{
		*MsgPtr = RXQueue[RXHead];
		RXHead ++;
		if (RXHead >= J1939_RX_QUEUE_SIZE)
			RXHead = 0;
		RXQueueCount --;
	}

	#if J1939_POLL_ECAN == J1939_FALSE
		#if ECAN_LEGACY_MODE == J1939_TRUE
			PIE3 |= ECAN_RX_INT_ENABLE_LEGACY;
		#else
			PIE3bits.RXBnIE = 1;
		#endif
	#endif

	return rc;
}

/*********************************************************************
J1939_EnqueueMessage

This routine takes a message from the caller's buffer and places it in
the transmit queue.  If the message cannot be queued or sent, an appropriate
return code is returned.  If interrupts are being used, then the
transmit interrupt is enabled after the message is queued.  If we are in
FIFO mode and interrupts were not previously enabled, then we must also
go enable an interrupt on the first available transmit buffer (TXB1).

NOTE: With this module, we do not get an interrupt automatically if we set
an interrupt enable on an empty buffer.  So if interrupts were not
previously enabled, we must enable the interrupt AND set the interrupt
flag.  If interrupts were already set from before, we just re-enable
the interrupt.

Parameters:	J1939_MESSAGE *		Pointer to the caller's message buffer
Return:		RC_SUCCESS			Message dequeued successfully
			RC_QUEUEFULL		Transmit queue full; message not queued
			RC_CANNOTTRANSMIT	System cannot currently transmit
								messages.
*********************************************************************/
unsigned char J1939_EnqueueMessage( J1939_MESSAGE *MsgPtr )
{
	unsigned char	rc = RC_SUCCESS;

	#if J1939_POLL_ECAN == J1939_FALSE
		PIE3 &= ~ECAN_TX_INT_ENABLE_LEGACY;
	#else
		PIE3bits.TXBnIE = 0;
	#endif

	if (J1939_Flags.CannotClaimAddress)
		rc = RC_CANNOTTRANSMIT;
	else
	{
		if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
			 (TXQueueCount < J1939_TX_QUEUE_SIZE))
		{
			if (TXQueueCount < J1939_TX_QUEUE_SIZE)
			{
				TXQueueCount ++;
				TXTail ++;
				if (TXTail >= J1939_TX_QUEUE_SIZE)
					TXTail = 0;
			}
			TXQueue[TXTail] = *MsgPtr;
		}
		else
			rc = RC_QUEUEFULL;

	}

	#if J1939_POLL_ECAN == J1939_FALSE
		#if ECAN_LEGACY_MODE == J1939_TRUE
			PIE3 |= ECAN_TX_INT_ENABLE_LEGACY;
			if (!TXIntsEnabled)
				PIR3bits.TXB1IF = 1; // The module won't set the flag by itself
		#else
			PIE3bits.TXBnIE = 1;
			if ((TXBIE == 0) && ((BIE0 & ~ECAN_BUFFER_INTERRUPT_ENABLE) == 0))
			{
				TXBIEbits.TXB1IE = 1;
				PIR3bits.TXBnIF = 1; // The module won't set the flag by itself
			}
		#endif
	#endif

	return rc;
}

/*********************************************************************
J1939_Initialization

This routine is called on system initialization.  It initializes
global variables, microcontroller peripherals, and interrupts.
It then starts the process of claiming the CA's address.
If the CA has a fixed NAME and Address, then it should call this
routine with a TRUE value passed in.  If it is Self-configurable or
Arbitrary Address Capable, then the CA should initialize NAME and
Address before calling this routine and call it with FALSE passed in.

NOTE: CA NAME is initialized by setting the CA_Name byte array.  The
Address is initialized by setting the value of J1939_Address.

NOTE: This routine will NOT enable global interrupts.  The CA needs
to do that when it's ready.

Parameters:		BOOL	Whether or not to initialize NAME and Address
						values.
Return:			None
*********************************************************************/
void J1939_Initialization( BOOL InitNAMEandAddress )
{
	unsigned char	i;

	// Initialize global variables;
	J1939_Flags.FlagVal = 1;	// Cannot Claim Address, all other flags cleared.
	ContentionWaitTime = 0l;
	TXHead = 0;
	TXTail = 0xFF;
	TXQueueCount = 0;
	RXHead = 0;
	RXTail = 0xFF;
	RXQueueCount = 0;

	if (InitNAMEandAddress)
	{
		J1939_Address = J1939_STARTING_ADDRESS;
		CA_Name[7] = J1939_CA_NAME7;
		CA_Name[6] = J1939_CA_NAME6;
		CA_Name[5] = J1939_CA_NAME5;
		CA_Name[4] = J1939_CA_NAME4;
		CA_Name[3] = J1939_CA_NAME3;
		CA_Name[2] = J1939_CA_NAME2;
		CA_Name[1] = J1939_CA_NAME1;
		CA_Name[0] = J1939_CA_NAME0;
	}
	CommandedAddress = J1939_Address;

	// Put the ECAN module into configuration mode and set it to the
	// desired mode.  Then configure the extra buffers for receive or
	// transmit as selected by the user.
	SetECANMode( ECAN_CONFIG_MODE );
	#if ECAN_LEGACY_MODE == J1939_TRUE
		#ifndef ECAN_IS_CAN_MODULE
			ECANCON = ECAN_SET_LEGACY_MODE;
		#endif
	#else
		ECANCON = ECAN_SET_FIFO_MODE;
		BSEL0   = ECAN_CONFIGURE_BUFFERS;
	#endif

	// Set up mask 0 to receive broadcast messages.  Set up mask 1 to
	// receive messages sent to the global address (or eventually us).
	RXM0SIDH = 0x07;
	RXM0SIDL = 0x88; //0x80;
	RXM0EIDH = 0x00;
	RXM0EIDL = 0x00;
	RXM1SIDH = 0x00;
	RXM1SIDL = 0x08;
	RXM1EIDH = 0xFF;
	RXM1EIDL = 0x00;

	// Set up filter 0 to accept only broadcast messages (PF = 240-255).
	// Set up filter 2 and 3 to accept only the global address.  Once we
	// get an address for the CA, we'll change filter 3 to accept that
	// address.
	RXF0SIDH = 0x07;
	RXF0SIDL = 0x88;
	RXF2SIDL = 0x08;
	RXF2EIDH = J1939_GLOBAL_ADDRESS;
	RXF3SIDL = 0x08;
	RXF3EIDH = J1939_GLOBAL_ADDRESS;

	// If we're in Legacy Mode, we need to set up filters 1, 4,
	// and 5 also, since we can't disable them.
	#if ECAN_LEGACY_MODE == J1939_TRUE
		RXF1SIDH = 0x07;
		RXF1SIDL = 0x88;
		RXF4SIDL = 0x08;
		RXF4EIDH = J1939_GLOBAL_ADDRESS;
		RXF5SIDL = 0x08;
		RXF5EIDH = J1939_GLOBAL_ADDRESS;
	#endif

	#if ECAN_LEGACY_MODE == J1939_FALSE
		// Set mask 0 to filter 0, and mask 1 to filters 2 and 3.
		MSEL0    = 0x5C;

		// Leave all filters set to RXB0.  The filters will apply to
		// all receive buffers.
		RXFBCON0  = 0x00;
		RXFBCON1  = 0x00;
		RXFBCON2  = 0x00;

		// Enable filters 0, 2, and 3.  Disable the others.
		RXFCON0  = 0x0D;
		RXFCON1  = 0x00;
	#endif

	// Set up bit timing as defined by the CA
	BRGCON1 = ECAN_BRGCON1;
	BRGCON2 = ECAN_BRGCON2;
	BRGCON3 = ECAN_BRGCON3;

	// Put the ECAN module into Normal Mode
	SetECANMode( ECAN_NORMAL_MODE );

	// Give the network management transmit buffer the highest priority.
	SET_NETWORK_WINDOW_BITS;
	MAPPED_CON = 0x03;

	// Configure the port pins for CAN operation
	#ifdef ECAN_IS_CAN_MODULE
		TRISBbits.TRISB2 = 0;	// CANTX
		TRISBbits.TRISB3 = 1;	// CANRX
	#else
		#if defined(__18F4680) || defined(__18F4585) || defined(__18F2680) || defined(__18F2585)
			TRISBbits.TRISB3 = 1;	// CANRX - Changed to accomodate 18F4680 - CG
		#endif
		#if defined(__18F8680) || defined(__18F8585) || defined(__18F6680) || defined(__18F6585)
			TRISGbits.TRISG3 = 1;	// CANRX
		#endif
	#endif

	// Establish the ECAN interrupt priorities and enable the ECAN receive
	// interrupt.  The caller must enable global interrupts when ready.
	#if J1939_POLL_ECAN == J1939_FALSE
		#if ECAN_LEGACY_MODE == J1939_TRUE
			TXIntsEnabled = 0;
			PIE3 = ECAN_RX_INT_ENABLE_LEGACY | ECAN_ERROR_INT_ENABLE;
		#else
			BIE0 = ECAN_BUFFER_INTERRUPT_ENABLE;
			PIE3 = ECAN_RX_INT_ENABLE_FIFO | ECAN_ERROR_INT_ENABLE;
		#endif
		#if J1939_PRIORITIZED_INT == J1939_TRUE
			RCONbits.IPEN = 1;
		#else
			RCONbits.IPEN = 0;
		#endif
		IPR3 &= 0xE0;
		IPR3 |= ECAN_INTERRUPT_PRIORITY;
	#endif

	// Start the process of claiming our address
	J1939_AddressClaimHandling( ADDRESS_CLAIM_TX );
}

/*********************************************************************
J1939_ISR

This function is called by the CA if it gets an interrupt from the CAN
controller.  First we'll clear the interrupt flags.  Then we'll call
the receive and transmit functions to process any received messages
and to transmit any messages in the transmit queue.

NOTE: In FIFO mode, RXB0IF follows RXBnIF instead of being forced to 0
as per the data sheet, so we clear them both.  The transmit interrupts
do not have this issue.

Parameters:	None
Return:		None
*********************************************************************/
#if J1939_POLL_ECAN == J1939_FALSE
void J1939_ISR( void )
{
	#if ECAN_LEGACY_MODE == J1939_TRUE
		if (PIR3 & ECAN_RX_INT_ENABLE_LEGACY)
		{
			PIR3 &= ~ECAN_RX_INT_ENABLE_LEGACY;
			J1939_ReceiveMessages();
		}

		if (PIR3 & ECAN_TX_INT_ENABLE_LEGACY)
		{
			PIR3 &= ~ECAN_TX_INT_ENABLE_LEGACY;
			J1939_TransmitMessages();
		}

		if (PIR3 & ECAN_ERROR_INT_ENABLE)
		{
			if (PIR3bits.ERRIF)
				PIR3bits.ERRIF = 0;

			if (PIR3bits.IRXIF)
				PIR3bits.IRXIF = 0;
		}
	#else
		if (PIR3bits.RXBnIF)
		{
			PIR3 &= ~ECAN_RX_INT_ENABLE_LEGACY;
			J1939_ReceiveMessages();
		}

		if (PIR3bits.TXBnIF)
		{
			PIR3bits.TXBnIF = 0;
			J1939_TransmitMessages();
		}

		if (PIR3 & ECAN_ERROR_INT_ENABLE)
		{
			if (PIR3bits.ERRIF)
				// Future Errata,  erroneous error flag
				if ((COMSTAT & 0x3F) == 0)
				{
					COMSTAT &= ~FIFOEMPTY_MASK;		// Reset FIFOEMPTY bit

					PIR3bits.ERRIF = 0;
				}
			else
				PIR3bits.ERRIF = 0;

			if (PIR3bits.IRXIF)
				PIR3bits.IRXIF = 0;
		}
	#endif
}
#endif

/*********************************************************************
J1939_Poll

If we're polling for messages, then this routine should be called by
the CA every few milliseconds during CA processing.  The routine receives
any messages that are waiting and transmits any messages that are queued.
Then we see if we are waiting for an address contention response.  If
we are and we've timed out, we accept the address as ours.

If the CA is using interrupts, then this routine should be called by
the CA every few milliseconds while the WaitingForAddressClaimContention
flag is set after calling J1939_Initialization.  If the Commanded Address
message can be accepted, this routine must also be called every few
milliseconds during CA processing in case we are commanded to change our
address.  If using interrupts, this routine will not check for received
or transmit messages; it will only check for a timeout on address
claim contention.

Parameters:	unsigned char	The number of milliseconds that have
							passed since the last time this routine was
							called.  This number can be approximate,
							and to meet spec, should be rounded down.
Return:		None
*********************************************************************/
void J1939_Poll( unsigned long ElapsedTime )
{
	unsigned int	Temp;

	// Update the Contention Wait Time.  We have to do that before
	// we call J1939_ReceiveMessages in case the time gets reset back
	// to zero in that routine.

	ContentionWaitTime += ElapsedTime;

	#if J1939_POLL_ECAN == J1939_TRUE
		J1939_ReceiveMessages();
		J1939_TransmitMessages();
	#endif

	if (J1939_Flags.WaitingForAddressClaimContention &&
		(ContentionWaitTime >= 250000l))
	{
		J1939_Flags.CannotClaimAddress = 0;
		J1939_Flags.WaitingForAddressClaimContention = 0;
		J1939_Address = CommandedAddress;

		// Set up filter to receive messages sent to this address.
		// If we're using interrupts, make sure that interrupts are disabled
		// around this section, since it will mess up what we're doing.
		#if J1939_POLL_ECAN == J1939_FALSE
			#if (J1939_PRIORITIZED_INT == J1939_FALSE) || (ECAN_RX_INTERRUPT_PRIORITY == 0x00) || (ECAN_TX_INTERRUPT_PRIORITY == 0x00)
				INTCONbits.GIEL = 0;
			#endif
			#if (ECAN_RX_INTERRUPT_PRIORITY != 0x00) || (ECAN_TX_INTERRUPT_PRIORITY != 0x00)
				INTCONbits.GIEH = 0;
			#endif
		#endif
		SetAddressFilter( J1939_Address );
		#if J1939_POLL_ECAN == J1939_FALSE
			#if (J1939_PRIORITIZED_INT == J1939_FALSE) || (ECAN_RX_INTERRUPT_PRIORITY == 0x00) || (ECAN_TX_INTERRUPT_PRIORITY == 0x00)
				INTCONbits.GIEL = 1;
			#endif
			#if (ECAN_RX_INTERRUPT_PRIORITY != 0x00) || (ECAN_TX_INTERRUPT_PRIORITY != 0x00)
				INTCONbits.GIEH = 1;
			#endif
		#endif
	}
}

/*********************************************************************
J1939_ReceiveMessage

This routine is called either when an interrupt is received from the
ECAN or by polling.  If a message has been received, it is read in.
If it is a network management message, it is processed.  Otherwise, it
is placed in the receive queue for the user.  Note that interrupts are
disabled during this routine, since it is called from the interrupt handler.

NOTE: To save stack space, the function J1939_CommandedAddressHandling
was brought inline.

Parameters:	None
Return:		None
*********************************************************************/
static void J1939_ReceiveMessages( void )
{
	unsigned char	*RegPtr;
	unsigned char	RXBuffer = 0;
	unsigned char	Loop;

	#if ECAN_LEGACY_MODE == J1939_TRUE
		while (RXBuffer < 2)		// Repeat for both receive buffers
	#else
		while (COMSTAT & FIFOEMPTY_MASK)		// Repeat until the FIFO is empty
	#endif
	{
		// Set the Window Address bits to the message buffer
		#if ECAN_LEGACY_MODE == J1939_TRUE
			CANCON &= 0xF0;
			if (RXBuffer == 1)
				CANCON |= 0x0A;
			if (!MAPPED_CONbits.RXFUL)
				goto TryNextBuffer;	// If no message, bail out
		#else
			ECANCON = ECAN_SET_FIFO_MODE | ECAN_SELECT_RX_BUFFER | (CANCON & 0x07);
		#endif

		// Read a message from the mapped receive buffer.
		RegPtr = &MAPPED_SIDH;
		for (Loop=0; Loop<J1939_MSG_LENGTH; Loop++, RegPtr++)
			OneMessage.Array[Loop] = *RegPtr;
		if (OneMessage.DataLength > 8)
			OneMessage.DataLength = 8;
		for (Loop=0; Loop<OneMessage.DataLength; Loop++, RegPtr++)
			OneMessage.Data[Loop] = *RegPtr;

		// Clear any receive flags
		MAPPED_CONbits.RXFUL = 0;
		#if ECAN_LEGACY_MODE == J1939_FALSE
			// Errata DS80162B section 6, try to clear the FIFO Empty flag
			COMSTAT &= ~FIFOEMPTY_MASK;

			// If we happen to be on the last buffer, we need to hit it twice.
			COMSTAT &= ~FIFOEMPTY_MASK;

		#endif

		// Format the PDU Format portion so it's easier to work with.
		Loop = (OneMessage.PDUFormat & 0xE0) >> 3;			// Get SID2-0 ready.
		OneMessage.PDUFormat = (OneMessage.PDUFormat & 0x03) |
								Loop |
								((OneMessage.PDUFormat_Top & 0x07) << 5);

		switch( OneMessage.PDUFormat )
		{
#if J1939_ACCEPT_CMDADD == J1939_TRUE
			case J1939_PF_TP_CM:
				if ((OneMessage.Data[0] == J1939_BAM_CONTROL_BYTE) &&
					(OneMessage.Data[5] == J1939_PGN0_COMMANDED_ADDRESS) &&
					(OneMessage.Data[6] == J1939_PGN1_COMMANDED_ADDRESS) &&
					(OneMessage.Data[7] == J1939_PGN2_COMMANDED_ADDRESS))
				{
					J1939_Flags.GettingCommandedAddress = 1;
					CommandedAddressSource = OneMessage.SourceAddress;
				}
				break;
			case J1939_PF_DT:
				if ((J1939_Flags.GettingCommandedAddress == 1) &&
					(CommandedAddressSource == OneMessage.SourceAddress))
				{	// Commanded Address Handling
					if ((!J1939_Flags.GotFirstDataPacket) &&
						(OneMessage.Data[0] == 1))
					{
						for (Loop=0; Loop<7; Loop++)
							CommandedAddressName[Loop] = OneMessage.Data[Loop+1];
						J1939_Flags.GotFirstDataPacket = 1;
					}
					else if ((J1939_Flags.GotFirstDataPacket) &&
						(OneMessage.Data[0] == 2))
					{
						CommandedAddressName[7] = OneMessage.Data[1];
						CommandedAddress = OneMessage.Data[2];
						if ((CompareName( CommandedAddressName ) == 0) &&	// Make sure the message is for us.
							CA_AcceptCommandedAddress())					// and we can change the address.
							J1939_AddressClaimHandling( ADDRESS_CLAIM_TX );
						J1939_Flags.GotFirstDataPacket = 0;
						J1939_Flags.GettingCommandedAddress = 0;
					}
					else	// This really shouldn't happen, but just so we don't drop the data packet
						goto PutInReceiveQueue;
				}
				else
					goto PutInReceiveQueue;
				break;
#endif
			case J1939_PF_REQUEST:
				if ((OneMessage.Data[0] == J1939_PGN0_REQ_ADDRESS_CLAIM) &&
					(OneMessage.Data[1] == J1939_PGN1_REQ_ADDRESS_CLAIM) &&
					(OneMessage.Data[2] == J1939_PGN2_REQ_ADDRESS_CLAIM))
					J1939_RequestForAddressClaimHandling();
				else
					goto PutInReceiveQueue;
				break;
			case J1939_PF_ADDRESS_CLAIMED:
				J1939_AddressClaimHandling( ADDRESS_CLAIM_RX );
				break;
			default:
PutInReceiveQueue:
				if ( (J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
					(RXQueueCount < J1939_RX_QUEUE_SIZE))
				{
					if (RXQueueCount < J1939_RX_QUEUE_SIZE)
					{
						RXQueueCount ++;
						RXTail ++;
						if (RXTail >= J1939_RX_QUEUE_SIZE)
							RXTail = 0;
					}
					RXQueue[RXTail] = OneMessage;
				}
				else
					J1939_Flags.ReceivedMessagesDropped = 1;
		}
		#if ECAN_LEGACY_MODE == J1939_TRUE
TryNextBuffer:
			RXBuffer ++;
		#endif
	}
}

/*********************************************************************
J1939_RequestForAddressClaimHandling

This routine is called if we're received a Request for Address Claim
message.  If we cannot claim an address, we send out a Cannot Claim
Address message.  Otherwise, we send out an Address Claim message for
our address.

NOTE:	J1939_PF_CANNOT_CLAIM_ADDRESS is the same value as
		J1939_PF_ADDRESS_CLAIMED, so we can reduce code size by
		combining the two.  Only the source address changes between
		the two messages.
*********************************************************************/
static void J1939_RequestForAddressClaimHandling( void )
{
	if (J1939_Flags.CannotClaimAddress)
		OneMessage.SourceAddress = J1939_NULL_ADDRESS;	// Send Cannot Claim Address message
	else
		OneMessage.SourceAddress = J1939_Address;		// Send Address Claim for current address

	OneMessage.Priority = J1939_CONTROL_PRIORITY;
	OneMessage.PDUFormat = J1939_PF_ADDRESS_CLAIMED;	// Same as J1939_PF_CANNOT_CLAIM_ADDRESS
	OneMessage.DestinationAddress = J1939_GLOBAL_ADDRESS;
	OneMessage.DataLength = J1939_DATA_LENGTH;
	CopyName();
	SET_NETWORK_WINDOW_BITS;
	SendOneMessage( (J1939_MESSAGE *) &OneMessage );
}

/*********************************************************************
J1939_TransmitMessages

This routine transmits as many messages from the transmit queue as it
can.  If the system cannot transmit messages, an error code is returned.
Note that interrupts are disabled during this routine, since it is
called from the interrupt handler.

Since we need to maintain transmit order with more than two buffers
being used when we're in FIFO mode, we set an interrupt to go off
when the last used buffer finishes.  If there are no more messages to
send, then we disable the interrupt.  Otherwise, we load the buffers
back up.  One extra interrupt saves us a lot of processing (and ROM)
in here and in J1939_EnqueueMessage.

Parameters:	None
Return:		RC_SUCCESS			Message was transmitted successfully
			RC_CANNOTTRANSMIT	System cannot transmit messages.
								Either we cannot claim an address or
								all transmit buffers are busy.
*********************************************************************/
static unsigned char LastTXBufferUsed = 0;

static unsigned char J1939_TransmitMessages( void )
{
	unsigned char Mask = 0x04;
	unsigned char Status;

	if (TXQueueCount == 0)
	{
		// We don't have any more messages to transmit, so disable
		// the transmit interrupts and reset LastTXBufferUsed.

		#if J1939_POLL_ECAN == J1939_FALSE
			#if ECAN_LEGACY_MODE == J1939_TRUE
				TXIntsEnabled = 0;
				PIE3bits.TXB0IE = 0;
				PIE3bits.TXB1IE = 0;
			#else
				TXBIE = 0;
				BIE0 = ECAN_BUFFER_INTERRUPT_ENABLE;	// Keep RX interrupts enabled.
				PIE3bits.TXBnIE = 0;
			#endif
		#endif
		LastTXBufferUsed = 0;
	}
	else
	{
		if (J1939_Flags.CannotClaimAddress)
			return RC_CANNOTTRANSMIT;

		// Make sure the last buffer we used last time is done transmitting.
		// This should be redundant if we're using interrupts, but it is required if
		// we're polling.  If the last buffer is done, then reset our buffer pointer
		// to the beginning of the buffer list.

		if (LastTXBufferUsed != 0)
		{
			#if ECAN_LEGACY_MODE == J1939_TRUE
				CANCON  = BUFFER_TABLE[LastTXBufferUsed-1].WindowBits;
			#else
				ECANCON = BUFFER_TABLE[LastTXBufferUsed-1].WindowBits;
			#endif
			if (MAPPED_CONbits.MAPPED_TXREQ)
			{
#if 0 // shouldn't need this
				#if J1939_POLL_ECAN == J1939_FALSE
					#if ECAN_LEGACY_MODE == J1939_TRUE
						PIE3 |= BUFFER_TABLE[LastTXBufferUsed-1].PIE3Val;
					#else
						TXBIE = BUFFER_TABLE[LastTXBufferUsed-1].TXBIEVal;
						BIE0  = BUFFER_TABLE[LastTXBufferUsed-1].BIE0Val | ECAN_BUFFER_INTERRUPT_ENABLE;
					#endif
				#endif
#endif
				return RC_CANNOTTRANSMIT;
			}
			else
				LastTXBufferUsed = 0;
		}

		// All transmit buffers are available, so fill them up.

		while ((TXQueueCount > 0) && (LastTXBufferUsed < ECAN_MAX_TX_BUFFERS))
		{
			#if ECAN_LEGACY_MODE == J1939_TRUE
				CANCON  = BUFFER_TABLE[LastTXBufferUsed].WindowBits;
			#else
				ECANCON = BUFFER_TABLE[LastTXBufferUsed].WindowBits;
			#endif
			if (!MAPPED_CONbits.MAPPED_TXREQ)	// make sure buffer is free
			{
				TXQueue[TXHead].SourceAddress = J1939_Address;
				SendOneMessage( (J1939_MESSAGE *) &(TXQueue[TXHead]) );
				TXHead ++;
				if (TXHead >= J1939_TX_QUEUE_SIZE)
					TXHead = 0;
				TXQueueCount --;
			}
			LastTXBufferUsed++;
		}

		// Enable the interrupt on the last used buffer

		#if J1939_POLL_ECAN == J1939_FALSE
			#if ECAN_LEGACY_MODE == J1939_TRUE
				TXIntsEnabled = 1;
				PIE3bits.TXB0IE = 0;
				PIE3bits.TXB1IE = 0;
				PIE3 |= BUFFER_TABLE[LastTXBufferUsed-1].PIE3Val;
			#else
				TXBIE = BUFFER_TABLE[LastTXBufferUsed-1].TXBIEVal;
				BIE0  = BUFFER_TABLE[LastTXBufferUsed-1].BIE0Val | ECAN_BUFFER_INTERRUPT_ENABLE;
				PIE3bits.TXBnIE = 1;
			#endif
		#endif
	}
	return RC_SUCCESS;
}
