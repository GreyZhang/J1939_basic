/*
Example 5a

This example shows what a request and acknowledge sequence might look 
like.  This node reads the value of a potentiometer on an analog input 
pin.  When it receives a request for Engine Speed, it will check a 
switch value.  If the switch is pressed, the value of the potentiometer 
will be returned as the engine speed.  If the switch is not pressed, a 
NACK response will be sent.
 
Application Maestro should be run with the following options changed 
from their default values (in addition to NAME, Address, and bit rate 
values): 

None
*/

#include <p18cxxx.h>
#include "j1939.h"

J1939_MESSAGE Msg;

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
#define J1939_PGN0_REQ_ENGINE_SPEED	0x04
#define J1939_PGN1_REQ_ENGINE_SPEED	0xF0
#define J1939_PGN2_REQ_ENGINE_SPEED	0x00

void main()
{
   unsigned char	EngineSpeed;
	unsigned char	Temp;

	RCONbits.IPEN = 1;
	TRISAbits.TRISA5 = 1;
	TRISBbits.TRISB4 = 1;
	TRISD = 0;

	LATD = 0;
	ADCON0 = 0b00101001;
	ADCON1 = 0x00;

	J1939_Initialization( TRUE );
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;

	// Wait for address contention to time out
	while (J1939_Flags.WaitingForAddressClaimContention)
		J1939_Poll(1);

	// Now we know our address is good, so start checking for
	// messages and switches.

	while (1)
	{
		for (Temp=0; Temp<100; Temp++);
		ADCON0bits.GO = 1;
		while (ADCON0bits.DONE);
		EngineSpeed = ADRESH;

		LATD = EngineSpeed;

		while (RXQueueCount > 0)
		{
			J1939_DequeueMessage( &Msg );
 			if (Msg.PDUFormat == J1939_PF_REQUEST)
			{
				if ((Msg.Data[0] == J1939_PGN0_REQ_ENGINE_SPEED) &&
					 (Msg.Data[1] == J1939_PGN1_REQ_ENGINE_SPEED) &&
					 (Msg.Data[2] == J1939_PGN2_REQ_ENGINE_SPEED))
				{
					if (PORTBbits.RB4)
					{
						Msg.Priority			= J1939_ACK_PRIORITY;
						Msg.DataPage			= 0;
						Msg.PDUFormat			= J1939_PF_ACKNOWLEDGMENT;
						Msg.DestinationAddress	= Msg.SourceAddress;
						Msg.DataLength			= 8;
						Msg.Data[0]			= J1939_NACK_CONTROL_BYTE;
						Msg.Data[1] 		= 0xFF;
						Msg.Data[2] 		= 0xFF;
						Msg.Data[3] 		= 0xFF;
						Msg.Data[4] 		= 0xFF;
						Msg.Data[5] 		= J1939_PGN0_REQ_ENGINE_SPEED;
						Msg.Data[6] 		= J1939_PGN1_REQ_ENGINE_SPEED;
						Msg.Data[7] 		= J1939_PGN2_REQ_ENGINE_SPEED;
					}
					else
					{
						Msg.Priority	= J1939_INFO_PRIORITY;
						Msg.DataPage	= J1939_PGN2_REQ_ENGINE_SPEED & 0x01;
						Msg.PDUFormat	= J1939_PGN1_REQ_ENGINE_SPEED;
						Msg.GroupExtension = J1939_PGN0_REQ_ENGINE_SPEED;
						Msg.DataLength	= 1;
						Msg.Data[0]		= EngineSpeed;
					}
					while (J1939_EnqueueMessage( &Msg ) != RC_SUCCESS);
				}
			}
		}
	}
}
