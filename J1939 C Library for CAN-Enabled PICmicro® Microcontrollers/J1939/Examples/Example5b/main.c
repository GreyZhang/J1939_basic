/*

Example 5b

This example shows what the corresponding node to Example 5A should
look like.  When the switch is pressed, it will ask Node 0 for the
engine speed.  If it receives the engine speed, it will display 
that value on the port D LED’s, matching Node 0’s LED’s.  If it
receives a NACK, it will light the RC2 LED instead.

Application Maestro should be run with the following options changed 
from their default values (in addition to NAME, Address, and bit rate 
values): 

Receive Interrupt Priority - High
Transmit Interrupt Priority - High
*/

#include <p18cxxx.h>
#include "j1939.h"

#define NODE1ADDRESS	128
#define NODE2ADDRESS	129

#define J1939_PGN0_REQ_ENGINE_SPEED	0x04
#define J1939_PGN1_REQ_ENGINE_SPEED	0xf0
#define J1939_PGN2_REQ_ENGINE_SPEED	0x00

//********************************************************************

void InterruptHandlerHigh (void);

//--------------------------------------------------------------------
// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x08
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

//********************************************************************

J1939_MESSAGE 	Msg;

//********************************************************************
void main()
{
	unsigned char	LastSwitchRB4 = 1;
	unsigned char	CurrentSwitch;
	unsigned char 	i;

	RCONbits.IPEN = 1;
	TRISBbits.TRISB4 = 1;
	TRISBbits.TRISB5 = 1;
	TRISCbits.TRISC2 = 0;
	TRISD = 0;

	LATD = 0;
	LATCbits.LATC2 = 0;

	J1939_Initialization( TRUE );

	INTCONbits.GIEH = 1;

	// Wait for address contention to time out
	while (J1939_Flags.WaitingForAddressClaimContention)
		J1939_Poll(5);

	// Now we know our address is good, so start checking for
	// messages and switches.

	while (1)
	{
		CurrentSwitch = PORTBbits.RB4;
		if (CurrentSwitch && !LastSwitchRB4)
		{
			// Ask for the engine speed
			Msg.DataPage			= 0;
			Msg.PDUFormat			= J1939_PF_REQUEST;
			Msg.Priority			= J1939_CONTROL_PRIORITY;
			Msg.DestinationAddress	= NODE1ADDRESS;
			Msg.DataLength			= 3;
			Msg.Data[0] = J1939_PGN0_REQ_ENGINE_SPEED;
			Msg.Data[1] = J1939_PGN1_REQ_ENGINE_SPEED;
			Msg.Data[2] = J1939_PGN2_REQ_ENGINE_SPEED;
			while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);
		}
		LastSwitchRB4 = CurrentSwitch;

		while (RXQueueCount > 0)
		{
			J1939_DequeueMessage( &Msg );
 			if (Msg.PDUFormat == J1939_PF_ACKNOWLEDGMENT)
			{
				LATD = 0;
				LATCbits.LATC2 = 1;
			}
			else if ((Msg.PDUFormat == J1939_PGN1_REQ_ENGINE_SPEED) &&
					 (Msg.GroupExtension == J1939_PGN0_REQ_ENGINE_SPEED))
			{
				LATD = Msg.Data[0];
				LATCbits.LATC2 = 0;
			}
		}
	}
}

