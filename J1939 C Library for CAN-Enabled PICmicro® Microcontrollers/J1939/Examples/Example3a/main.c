/*
Example 3a

This example shows the same concept as Example 2, using interrupts to 
check for a message to light an LED and to send a message if a button 
is pressed.  But for the first 5 button presses, the message is sent to
the wrong address.  On the 5th push, the Commanded Address message is
sent to command the other node to use the address that this node is
sending the message to.  Note that this node doesn’t even need to know what the other node’s first address is, as long as it knows the node’s NAME.

This example will use high priority interrupts.

Application Maestro should be run with the following options changed
from their default values (in addition to NAME, Address, and bit rate
values): 

Receive Interrupt Priority – High
Transmit Interrupt Priority - High
*/

#include <p18cxxx.h>
#include "j1939.h"

J1939_MESSAGE Msg;

// Define some arbitrary values.  They must agree with the other node's
// values.

#define SECOND_ADDRESS	132
#define TURN_ON_LED		92	
#define TURN_OFF_LED		94	
#define NODE_NAME0		51
#define NODE_NAME1		0
#define NODE_NAME2		0
#define NODE_NAME3		0
#define NODE_NAME4		0
#define NODE_NAME5		0
#define NODE_NAME6		0
#define NODE_NAME7		0

void InterruptHandlerHigh (void);

//--------------------------------------------------------------------
// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x0008
void InterruptVectorHigh( void )
{
  _asm
    goto InterruptHandlerHigh
  _endasm
}

//--------------------------------------------------------------------
// High priority interrupt routine

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh( void )
{
	if (PIR3 != 0x00)
		J1939_ISR();
}

//--------------------------------------------------------------------

void main( void )
{
	unsigned char	LastSwitch = 1;
	unsigned char	CurrentSwitch;
   unsigned char	PushCount = 0;

	TRISBbits.TRISB4 = 1;			// Switch pin
	TRISD = 0;							// LED pins
	LATD = 0;							// Turn off LED

	J1939_Initialization( TRUE );
   INTCONbits.GIEH = 1;

	// Wait for address contention to time out
	while (J1939_Flags.WaitingForAddressClaimContention)
		J1939_Poll(5);

	// Now we know our address should be good, so start checking for
	// messages and switches.

	while (1)
	{
		CurrentSwitch = PORTBbits.RB4;
		if (LastSwitch != CurrentSwitch)
		{
			Msg.DataPage				= 0;
			Msg.Priority				= J1939_CONTROL_PRIORITY;
			Msg.DestinationAddress	= SECOND_ADDRESS;
			Msg.DataLength				= 0;
			if (CurrentSwitch == 0)
				Msg.PDUFormat = TURN_ON_LED;
			else
			{
				Msg.PDUFormat = TURN_OFF_LED;
				if (PushCount < 6)
					PushCount ++;
			}
	   while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);
			LastSwitch = CurrentSwitch;

			if (PushCount == 5)
			{
				Msg.DataPage				= 0;
		Msg.Priority		= J1939_TP_CM_PRIORITY;
				Msg.DestinationAddress	= J1939_GLOBAL_ADDRESS;
				Msg.DataLength				= 8;
				Msg.PDUFormat 				= J1939_PF_TP_CM;
				Msg.Data[0]					= J1939_BAM_CONTROL_BYTE;
				Msg.Data[1]					= 9;    // 9 data bytes
				Msg.Data[2]					= 0;
				Msg.Data[3]					= 2;    // 2 packets
				Msg.Data[4]					= 0xFF; // Reserved
				Msg.Data[5]					= 0xD8; // PGN
				Msg.Data[6]					= 0xFE; // PGN
				Msg.Data[7]					= 0x00; // PGN
				while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);

				Msg.DataPage				= 0;
				Msg.Priority				= J1939_TP_DT_PRIORITY;
				Msg.DestinationAddress	= J1939_GLOBAL_ADDRESS;
				Msg.DataLength				= 8;
				Msg.PDUFormat 				= J1939_PF_DT;
				Msg.Data[0]					= 1;    // First packet
				Msg.Data[1]					= NODE_NAME0;
				Msg.Data[2]					= NODE_NAME1;
				Msg.Data[3]					= NODE_NAME2;
				Msg.Data[4]					= NODE_NAME3;
				Msg.Data[5]					= NODE_NAME4;
				Msg.Data[6]					= NODE_NAME5;
				Msg.Data[7]					= NODE_NAME6;
				while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);

				Msg.Data[0]					= 2;    // Second packet
				Msg.Data[1]					= NODE_NAME7;
				Msg.Data[2]					= SECOND_ADDRESS;
				Msg.Data[3]					= 0xFF;
				Msg.Data[4]					= 0xFF;
				Msg.Data[5]					= 0xFF;
				Msg.Data[6]					= 0xFF;
				Msg.Data[7]					= 0xFF;
				while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);
			}
		}

		while (RXQueueCount > 0)
		{
			J1939_DequeueMessage( &Msg );
			if (Msg.PDUFormat == TURN_ON_LED)
				LATDbits.LATD0 = 1;
			else if (Msg.PDUFormat == TURN_OFF_LED)
				LATDbits.LATD0 = 0;
		}
	}
}

