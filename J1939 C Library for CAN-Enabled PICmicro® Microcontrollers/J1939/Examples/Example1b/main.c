/*
Example 1

This example shows a very simple J1939 implementation.  It uses polling
to check for a message to light an LED and to send a message if a
button is pressed.

Both Node 0 and Node 1 should be programmed with the same code, except
that OTHER_NODE should be defined as the other node’s J1939 Address.

Application Maestro should be run with the following options changed
from their default values (in addition to NAME, Address, and bit rate
values):

Interrupts or Polling - Polling
*/

#include <p18cxxx.h>
#include "j1939.h"

J1939_MESSAGE Msg;

// Define some arbitrary values.  They must agree with the other node's
// values.

#define OTHER_NODE		128
#define TURN_ON_LED		92	
#define TURN_OFF_LED	94	

void main( void )
{
	unsigned char	LastSwitch = 1;
	unsigned char	CurrentSwitch;

	TRISBbits.TRISB4 = 1;			// Switch pin
	TRISD = 0;							// LED pins
	LATD = 0;							// Turn off LED

	J1939_Initialization( TRUE );

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
			Msg.DestinationAddress	= OTHER_NODE;
			Msg.DataLength				= 0;
			if (CurrentSwitch == 0)
				Msg.PDUFormat = TURN_ON_LED;
			else
				Msg.PDUFormat = TURN_OFF_LED;
		    while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS)
				J1939_Poll(5);
			LastSwitch = CurrentSwitch;
		}

		while (RXQueueCount > 0)
		{
			J1939_DequeueMessage( &Msg );
			if (Msg.PDUFormat == TURN_ON_LED)
				LATDbits.LATD0 = 1;
			else if (Msg.PDUFormat == TURN_OFF_LED)
				LATDbits.LATD0 = 0;
		}

		// Since we don’t accept the Commanded Address message,
		// the value passed here doesn’t matter.
		J1939_Poll(20);
	}
}
