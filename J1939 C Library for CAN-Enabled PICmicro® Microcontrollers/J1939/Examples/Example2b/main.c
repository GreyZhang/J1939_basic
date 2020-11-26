/*
Example 2

This example shows the same concept as Example 1, except that instead
of polling, it uses interrupts to check for a message to light an LED
and to send a message if a button is pressed.

Application Maestro should be run with the following options changed
from their default values (in addition to NAME, Address, and bit rate
values):

None
*/

#include <p18cxxx.h>
#include "j1939.h"

J1939_MESSAGE Msg;

// Define some arbitrary values.  They must agree with the other node's
// values.

#define OTHER_NODE		128
#define TURN_ON_LED		92
#define TURN_OFF_LED		94

void InterruptHandlerLow (void);

//--------------------------------------------------------------------
// Low priority interrupt vector

#pragma code InterruptVectorLow = 0x0018
void InterruptVectorLow( void )
{
  _asm
    goto InterruptHandlerLow
  _endasm
}

//--------------------------------------------------------------------
// Low priority interrupt routine

#pragma code
#pragma interruptlow InterruptHandlerLow

void InterruptHandlerLow( void )
{
	if (PIR3 != 0x00)
		J1939_ISR();
}

//--------------------------------------------------------------------

void main( void )
{
	unsigned char	LastSwitch = 1;
	unsigned char	CurrentSwitch;

	TRISBbits.TRISB4 = 1;			// Switch pin
	TRISD = 0;							// LED pins
	LATD = 0;							// Turn off LED

	J1939_Initialization( TRUE );
   INTCONbits.PEIE = 1;		// Enable peripheral interrupts
   INTCONbits.GIE = 1;		// Enable global interrupts

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

			// We don’t need to call J1939_Poll in the middle of this
			// loop, since the queue will be emptied during interrupt
			// processing.
			while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);

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

		// We don’t need to call J1939_Poll, since the queues will
		// be managed during the interrupt processing.
	}
}
