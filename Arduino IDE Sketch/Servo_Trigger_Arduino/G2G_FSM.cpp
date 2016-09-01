/*--------------------------------------------------------------------------
Title: G2G_FSM.cpp
Version 0.1

Originally written by: Byron Jacquot
Original Title:  ServoTrigger.c
Device: Sparkfun Servo Trigger
Created: 7/3/2014 3:43:10 PM

Modified for the Arduino by: Kevin Gagnon
Created: 8/31/2016

Twitter: @GadgetsToGrow
---------------------------------------------------------------------------*/
#define DEBUG_ON

#include "G2G_FSM.h"


static const int32_t PHASOR_MAX     = 0x0000ffff; // Phasor counts from 0 to 0xffff
static const int32_t ADC_MAX        = 0x0000ffff; // ADC values are left-justified into 16-bits

/*---------------------------------------------------------------------------------- 
	Look Up Table (LUT) of timing constants
	
	These are looked up and added to the phasor on each pulse ISR.
	They are indexed & interpolated using the travelTime variable
	for each servo.

	Each value was calculated using the spreadsheet: translation.ods

	The formula per entry is:
	value = PHASOR_MAX/PWM freq in Hz/desired traveTime
	
	Table is 17 entries long so we can do linear interpolation between
	the N and N+1th entries, indexed using 4 MSBs of traveTime value, interpolated
	using the next 4 bits.
----------------------------------------------------------------------------------*/

static const int16_t timelut[17] =
{
	437,  583,   749,   1049,
	1311, 1748,  2185,  2621,
	3277, 4369,  5243,  6554,
	8738, 10923, 13107, 16384,
	26214
};

// ***
// *** Constructor
// ***
G2G_FSM::G2G_FSM(){};

// ***
// *** Destructor
// ***
G2G_FSM::~G2G_FSM(){};

/*-----------------------------------------------------------------------
void initPWM()

Uses Timer 1 to setup a PWM value suitable for a hobby servo at 50Hz

-----------------------------------------------------------------------*/
void G2G_FSM::initPWM(){

	/*---------------------------------------------------------------------- 
	TIMER CONTROL REGISTER A - TCCR1A
	// ***	Bits 6-7:	COM1A1 Clear OC1A on Compare Match, set OC1A at BOTTOM	
	// ***	Bits 4-5:	COM1B1 Clear OC1B on Compare Match, set OC1B at BOTTOM  
	// ***	Bits 0-2:	WGM LSBs (2 of 4) Waveform Generator bits (WGM11 is set)	
	// ***	16Mhz ATMega328 Clock / 8 = Prescale of 2Mhz
	----------------------------------------------------------------------*/
	TCCR1A = B10100010;
	/*---------------------------------------------------------------------- 
	TIMER CONTROL REGISTER A - TCCR1A
	// ***	Bits 6-7:	
	// ***	Bits 3-4:	WGM MSBs (2 of 4) Waveform Generator bits (WGM12/13 is set)
	// ***	Bits 0-2:	Clock Speed Selection (CS10-CS12) Clock/8	
	// ***	16Mhz ATMega328 Clock / 8 = Prescale of 2Mhz
	----------------------------------------------------------------------*/
	TCCR1B = B00011010;
	
	TCCR1C = 0;  //Not Used
	
	/*---------------------------------------------------------------------- 
	INPUT CAPTURE REGISTER (ICR1)
	// *** Prescale of 2Mhz / 40,000 = 50 Hz (40,000 2Mhz pulses)
	----------------------------------------------------------------------*/
	ICR1 = 40000 - 1;
	
	/*---------------------------------------------------------------------- 
	TIMER/COUNTER REGISTER (TCNT1) 16bit
	Set to zero
	----------------------------------------------------------------------*/
	TCNT1 = 0;
	
	/*---------------------------------------------------------------------- 
	OUTPUT COMPARE REGISTER A (OCR1A) 16bit		SERVO 1 PULSE WIDTH
	----------------------------------------------------------------------*/
	OCR1A = 1000;
	
	/*---------------------------------------------------------------------- 
	OUTPUT COMPARE REGISTER B (OCR1B) 16bit		SERVO 2 PULSE WIDTH
	----------------------------------------------------------------------*/
	OCR1B = 1000;

	/*---------------------------------------------------------------------- 
	INTERRUPT MASK REGISTER
	----------------------------------------------------------------------*/
	TIMSK1 = B00100110;
		
}

/*-----------------------------------------------------------------------
bool attachServo(uint8_t servoID)

Attaches Servo to hardware: PortB, PB1 (Arduino pin9), PB2 (Arduino pin10)
Sets the fsm_status[servoID].attached = true;
-----------------------------------------------------------------------*/

bool G2G_FSM::attachServoFSM(uint8_t servoID) {

	bool result = false;
	
	//Only allow servo numbers 0 and 1
	if (servoID < 2) {
		
		if(servoID == 0) {

			DDRB |= B00000010;
			fsm_status[0].attached = true;
			result = true;
		} else if (servoID ==1) {
			
			DDRB |= B00000100;
			fsm_status[1].attached = true;
			result = true;
		}
		
	}
	
	return result;
}

/*-----------------------------------------------------------------------
bool detachServo(uint8_t servoID)

Detaches Servo from hardware: PortB, PB1 (Arduino pin9), PB2 (Arduino pin10)
Sets the fsm_status[servoID].attached = false;
-----------------------------------------------------------------------*/

bool G2G_FSM::detachServoFSM(uint8_t servoID) {
	
	bool result = false;
	
	//Only allow servo numbers 0 and 1
	if (servoID < 2) {
		
		if(servoID == 0) {
			
			DDRB &= ~(1<<1);
			fsm_status[0].attached = false;
			result = true;
			
		} else if (servoID ==1) {
			
			DDRB &= ~(1<<2);
			fsm_status[1].attached = false;
			result = true;
		}
		
	}
	
	return result;
}


/*-----------------------------------------------------------------------
void calculate(uint8_t servoID)

Called by ISR(TIMER1_CAPT_vect) in main code.  Selects the servo's FSM
to calcuate next position.  FSM updates the us_val variable for the 
OCR1A or OCR1B update depending on the servoID.

-----------------------------------------------------------------------*/


void G2G_FSM::calculate(uint8_t servoID) {
	
	switch(fsm_status[servoID].fsmMode) {
		
		case ASTABLE:

			astableFSM(servoID);

			break;

		case BISTABLE:

			bistableFSM(servoID);

			break;
	
		case ONESHOT:
		
			oneshotFSM(servoID);
			
			break;

		default:
		
			break;
	}
	
}

/*------------------------------------------------------------------------
	calcDelta()

	Calculate how far to step in each increment, and return the step size.
	It's based on the reading of the travelTime variable.
	MS nibble indexes LUT value.
	Also reads the next adjacent LUT value, then uses next nibble to 
	calculate the linear interpolation between them.
------------------------------------------------------------------------*/
int16_t G2G_FSM::calcDelta(uint8_t servoID)
{

	// time reading is 12 bits, formatted into the top of 
	// a 16-bit value.
	// We'll use the 4 MSBs to look up a time value,
	// and the next 4 to do linear interpolation between it and the next value.

	uint8_t idx = fsm_status[servoID].travelTime >> 12;
	int16_t val = timelut[idx];
	
	// Calc window between this and next value
	volatile int16_t window = timelut[idx+1] - timelut[idx];

	// split that window into 16 chunks
	window >>= 4;  // IE: divide by 16
	
	// and take the number of chunks as determined by the next 4 bits
	window *= ((fsm_status[servoID].travelTime & 0x00000f00) >> 8);

	return val + window;

}

/*---------------------------------------------------------------------------------
	bool calcNextPhasor(int16_t increment)

    This calculates the value of the phasor.
    The phasor is range constrained:
      - it always goes from 0 to 0xffff (PHASOR_MAX)
      - it always increases on the A->B traverse
      - it always decreases on the B->A traverse
      - it sits at the endpoints (0 or 0xffff) when in the static positions

    This leaves us some resolution, so really slow transitions will slowly crawl 
    along, and really quick ones move really quick.

    The trick is that it will be scaled based on the pot settings before being applied.

    Returns true when segment overflows, to advance FSM.
-------------------------------------------------------------------------------------*/
bool G2G_FSM::calcNextPhasor(uint8_t servoID, int16_t increment)
{
	if(fsm_status[servoID].rising)
	{
		fsm_status[servoID].phasor += increment;
	}
	else
	{
		fsm_status[servoID].phasor -= increment;
	}
	
	// check for over/underflow indicating end of segment reached.
	// If so, truncate & return...
	if(fsm_status[servoID].phasor > PHASOR_MAX)
	{
		fsm_status[servoID].phasor = PHASOR_MAX;
		return true;		
	}
	else if(fsm_status[servoID].phasor < 0)
	{
		fsm_status[servoID].phasor = 0;
		return true;		
	}
	
	return false;
}

/*-----------------------------------------------------------------------------
    int16_t scalePhasor()

    This routine scales the phasor into the proper range.
    The phasor always goes from A to B by covering the 0 to 0xffff range
    But A and B may by nearer to each other than that
    and B might even be less than A, reversing things overall.
    This routine centralizes that translation.
------------------------------------------------------------------------------*/
int16_t G2G_FSM::scalePhasor(uint8_t servoID)
{
	int32_t range;
	int32_t offset;
	
	int32_t   result;

	// a, b are 16-bit unsigned values	
	// range and offset are 32-bit signed.
    // When a > b, range is negative
	// thus the ultimate result will be subtracted from the offset.
	range = fsm_status[servoID].positionB - fsm_status[servoID].positionA;
	offset = fsm_status[servoID].positionA;
	
	//Scale range and offset into uSec values.
	range = (range * PWM_RANGE_USEC)/ADC_MAX;
	offset = (offset * PWM_RANGE_USEC)/ADC_MAX;
		
	// scale phasor into range
	result = (fsm_status[servoID].phasor * range)/PHASOR_MAX;
		
	//add on offset
    // (or subtract, if positionA > positionB)
	result += offset;

	return result;
}

/*----------------------------------------------------------------------------
 void oneshotFSM()

    This FSM sits in idle state (positionA) waiting for trigger to set.
    When it sets, the servo does a complete cycle A-to-B-to-A,
	then waits for trigger to clear.
----------------------------------------------------------------------------*/
void G2G_FSM::oneshotFSM(uint8_t servoID)
{
	int16_t delta;
	
	delta = calcDelta(servoID);
	
	switch(fsm_status[servoID].servo_state)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(fsm_status[servoID].trigger == true)
			{
				fsm_status[servoID].servo_state = eATOB;
				fsm_status[servoID].rising = true;
			}
		}
		break;
		case eATOB:
		{
			// climbing up to b
			// only quits when it gets there - ignores switch
			if( calcNextPhasor(servoID, delta) )
			{
				fsm_status[servoID].servo_state = eBTOA;
				fsm_status[servoID].rising = false;
			}
		}
		break;
		case eATTOP:
		{
			// we shouldn't actually land here,
			// provide proper action in case we somehow do.
			fsm_status[servoID].servo_state = eBTOA;
			fsm_status[servoID].rising = false;
		}
		break;
		case eBTOA:
		{
			// dropping down to A
			// only quits when it gets there - ignores switch
			if( calcNextPhasor(servoID, delta) )
			{
				fsm_status[servoID].servo_state = eWAIT_TO_RESET;
			}
		}
		break;
		case eWAIT_TO_RESET:
		{
			// If the switch is still held
			// wait for it's release here
			if(!fsm_status[servoID].trigger)
			{
				fsm_status[servoID].servo_state = eIDLE;
			}
			
		}
		
			break;
		
		default:
		
			break;
	}
	
	fsm_status[servoID].us_val =  scalePhasor(servoID);
	
}




/*--------------------------------------------------------------
 void astableFSM()

  astable - runs back & forth while trigger is set to true.

---------------------------------------------------------------*/
void G2G_FSM::astableFSM(uint8_t servoID)
{
	int16_t delta;
	
	delta = calcDelta(servoID);
	
    //bool edge = edgeDetect();
    
	switch(fsm_status[servoID].servo_state)
	{
    	case eIDLE:
    	{
        	// Advance when we see the switch actuate
	       	if(fsm_status[servoID].trigger == true)
        	{
            	fsm_status[servoID].servo_state = eATOB;
            	fsm_status[servoID].rising         = true;
        	}
    	}
    	break;
    	case eATOB:
    	{
            // trigger goes away?  Stop where we are.
        	if(fsm_status[servoID].trigger == false)
        	{
            	fsm_status[servoID].servo_state = eIDLE;
            	fsm_status[servoID].rising         = false;
        	}
        	else if( calcNextPhasor(servoID, delta) )
        	{
            	fsm_status[servoID].servo_state = eBTOA;
            	fsm_status[servoID].rising         = false;
        	}
    	}
    	break;
    	case eATTOP:
    	{
        	// we shouldn't actually land here,
        	// provide proper action in case we somehow do.
        	if(fsm_status[servoID].trigger == false)
        	{
            	fsm_status[servoID].servo_state = eIDLE;
            	fsm_status[servoID].rising         = false;
        	}
            
            // Just fall through
        	fsm_status[servoID].servo_state = eBTOA;
        	fsm_status[servoID].rising         = false;
    	}
    	break;
    	case eBTOA:
    	{
        	// dropping down to A
        	// only quits when it gets there - ignores switch
        	if(fsm_status[servoID].trigger == false)
	        	{
    	        	fsm_status[servoID].servo_state = eIDLE;
    	        	fsm_status[servoID].rising         = false;
	        	}
            else if( calcNextPhasor(servoID, delta) )
        	{
            	fsm_status[servoID].servo_state = eATOB;
  	        	fsm_status[servoID].rising         = true;
        	}
    	}
    	break;
    	default:
		
		break;
    	//{
        	//// TODO: better fix?
        	//// debugger catch for invalid states
        	//while(1);
    	//}
	}
    
    

	fsm_status[servoID].us_val =  scalePhasor(servoID);
	
}

/*--------------------------------------------------------------------------------- 
	void bistableFSM()

    This FSM sits in bottom state (positionA) waiting for trigger to set to true.
	When it is set, it transits to positionB, taking time travelTime.
	It stays in positionB until the trigger clears, then it transits back to positionA,
	taking time travelTime.
-------------------------------------------------------------------------------------*/
void G2G_FSM::bistableFSM(uint8_t servoID)
{
	int16_t delta;
	
	delta = calcDelta(servoID);
	
	switch(fsm_status[servoID].servo_state)
	{
		case eIDLE:
		{
			// Advance when we see the switch actuate
			if(fsm_status[servoID].trigger == true)
			{
				fsm_status[servoID].servo_state = eATOB;
				fsm_status[servoID].rising         = true;
			}
		}
		break;
		case eATOB:
		{
            // Switch released early
			if(fsm_status[servoID].trigger == false)
			{
				fsm_status[servoID].servo_state = eBTOA;
				fsm_status[servoID].rising         = false;				
			}
			
			// climbing up to b
			if( calcNextPhasor(servoID, delta) )
			{
				fsm_status[servoID].servo_state = eATTOP;
			}
		}
		break;
		case eATTOP:
		{
            // waiting for switch release
			if(fsm_status[servoID].trigger == false)
			{
				fsm_status[servoID].servo_state = eBTOA;
				fsm_status[servoID].rising         = false;
			}
		}
		break;
		case eBTOA:
		{
            // Switch re-closed before reaching A
			if(fsm_status[servoID].trigger == true)
			{
				fsm_status[servoID].servo_state = eATOB;
				fsm_status[servoID].rising = true;
			}
			
			// dropping down to A
			if( calcNextPhasor(servoID, delta) )
			{
				fsm_status[servoID].servo_state = eIDLE;
			}
		}
		break;
		default:
		
		break;
		//{
			//// TODO: better fix?
			//// debugger catch for invalid states
			//while(1);
		//}
	}
	
	fsm_status[servoID].us_val =  scalePhasor(servoID);
	
}


/*-----------------------------------------------------------------------
INTERRUPT SERVICE REQUEST (ISR) - TIMER1_COMPA_vect and TIMER1_COMPB_vect

for OCR1A and OCR1B  - Not used yet.  Not sure if I'm going to
If not, let's unset Bits 1 and 2 of the TIMSK1 register

-----------------------------------------------------------------------*/

ISR(TIMER1_COMPA_vect) {
	
	//Serial.println("CompareA = " + String(OCR1A, HEX));
	
}

ISR(TIMER1_COMPB_vect) {
	
	//Serial.println("CompareB = " + String(OCR1B, HEX));
	
}