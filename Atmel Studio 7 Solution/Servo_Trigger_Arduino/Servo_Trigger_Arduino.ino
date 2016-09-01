/*******************************************************************************
 * Servo_Trigger_Arduino.ino
 *
 * Created:		8/31/2016 7:01:59 AM
 * Author:		Kevin Gagnon
 * Twitter:		@GadgetsToGrow
 * 
 * Purpose:		Emulate Sparkfun's Servo Trigger Firmware on an Arduino Uno,
 *				utilizing 2 servos, with independently controlled software 
 *				configuration of the servo associated Finite State Machines (FSMs).	
 *
 * References and Credit:
 * 
 * http://seateapea.tumblr.com/
 * https://www.sparkfun.com/products/13118
 * https://learn.sparkfun.com/tutorials/servo-trigger-hookup-guide
 * http://www.cs.princeton.edu/courses/archive/spr06/cos116/FSM_Tutorial.pdf
 * http://sculland.com/ATmega168/Interrupts-And-Timers/16-Bit-Timer-Setup/
 *
 *********************************************************************************/


/*-----------------------------------------------------------
 Include the G2G_FSM library and instantiate the fsm object
-----------------------------------------------------------*/
#include "G2G_FSM.h"
G2G_FSM fsm;


/*-----------------------------------------------------------
Global variables
-----------------------------------------------------------*/

//Non-blocking loop delay
unsigned long _previousMillis = 0;
unsigned long _currentMillis = 0;

void setup()
{
	
	
	/*---------------------------------------------------------- 
	Setup Timer 1 on the Arduino to output a stable 50Hz pulse
	----------------------------------------------------------*/
	fsm.initPWM();

	/*-----------------------------------------------------------
	Setup pin 9 (PORTB PB1) and/or pin 10 (PORTB PB2) for OUTPUT 
	and set the fsm_status[servoID].attached variable to TRUE.
	-----------------------------------------------------------*/
	fsm.attachServoFSM(0);
	fsm.attachServoFSM(1);

	
	/*--------------------------------------------------------------- 
	Set the initial Finite State Machine (FSM) "state" of each servo.
	---------------------------------------------------------------*/
	fsm.fsm_status[0].servo_state = fsm.eIDLE;
	fsm.fsm_status[1].servo_state = fsm.eIDLE;
	
	/*-------------------------------------------------------------------------
	positionA	//BOTTOM
	positionB	//TOP
	travelTime	//Time to move to each position
	
	Initial values for the Servos - the position numbers are unusually 
	large and were originally set by potentiometers on the Servo Trigger. 
	I haven't dug into the code enough to see if I can scale these further 
	and I ended up grinding a lot of gears	to figure out the ranges during 
	my initial coding attempts.  These seem to work on the servos I've tested. 
	
	CAUTION: If you change the positionA or positionB values, be ready to pull 
	the power quickly when you start hearing the grinding of gears!!  
	---------------------------------------------------------------------------*/
	//Servo 0
	fsm.fsm_status[0].positionA = 10000;
	fsm.fsm_status[0].positionB = 180000;
	fsm.fsm_status[0].travelTime = 750;
	//Servo 1
	fsm.fsm_status[1].positionA = 10000;
	fsm.fsm_status[1].positionB = 180000;
	fsm.fsm_status[1].travelTime = 1000;
	
	/*--------------------------------------------------------------------------- 
	fsm.fsm_status[Servo 0 or Servo 1].fsmMode
	
	Sets the initial FSM Mode for each servo attached.  For testing purposes of 
	the included FSM methods, Servo 0 will use ASTABLE and Servo 1 will use BISTABLE.  
	The Interrupt Service Request (ISR) will do all of the heavy lifting while the 
	main loop will toggle the "trigger" to allow experimentation with the different 
	FSMs and how the trigger affects each.  Create your own, or use the other FSMs 
	located in the G2G_FSM.cpp file.
	
	To keep things less cluttered and easy to understand I've included 3 of the 
	original FSMs (slightly modified) from the Sparkfun Servo Trigger firmware. 
	Note: The Servo Trigger allows one of two modes to be selected by soldering a 
	jumper on the board. If you want to change modes with the Servo Trigger for 
	Arduino, simply change the fsmMode below to the desired FSM.
	
	Description of available FSMs:
	
	ASTABLE -	This FSM does a complete cycle A-to-B-to-A while trigger is set to 
				true.
	
	BISTABLE -	This FSM sits in bottom state (positionA) waiting for trigger 
				to set to true. When it is set, it transits to positionB, taking 
				time travelTime. It stays in positionB until the trigger clears, 
				then it transits back to positionA, taking time travelTime.

	ONESHOT -	This FSM sits in idle state (positionA) waiting for trigger to 
				set to true. When it is set, the servo does a complete cycle 
				A-to-B-to-A, and waits for trigger to clear.
	
	-------------------------------------------------------------------------------*/	
	fsm.fsm_status[0].fsmMode = fsm.ASTABLE;
	fsm.fsm_status[1].fsmMode = fsm.BISTABLE;
	
	
	/*---------------------------------------------------------------------------- 
	fsm.fsm_status[Servo 0 or Servo 1].trigger
	
	Note: replaces "input" on the Sparkfun Servo Trigger firmware
	
	Sets the initial trigger value (moves FSM from state-to-state) 
	The loop() function below toggles this trigger to see how it affects each FSM.
	The _loopDelay variable determines the amount of time in milliseconds between 
	each toggle.
	-----------------------------------------------------------------------------*/	
	fsm.fsm_status[0].trigger = true;
	fsm.fsm_status[1].trigger = true;	

}

	/*----------------------------------------------------------------------------- 
	_loopDelay
	
	Change this variable in milliseconds to determine when each trigger is toggled 
	true or false for each FSM.  
	------------------------------------------------------------------------------*/
	unsigned long _loopDelay = 5000;


void loop()
{
	//Get the current millis()
	_currentMillis = millis();
		
	//Time to toggle?
	if ((unsigned long)(_currentMillis - _previousMillis >= _loopDelay)) {
			
		/*----------------------------------------
			Toggle the FSM trigger for Servo 0
		----------------------------------------*/
		if(fsm.fsm_status[0].trigger == true) {
				
			fsm.fsm_status[0].trigger = false;
				
			} else {
				
			fsm.fsm_status[0].trigger = true;
				
		}
		/*----------------------------------------
			Toggle the FSM trigger for Servo 1
		----------------------------------------*/
		if(fsm.fsm_status[1].trigger == true) {
			
			fsm.fsm_status[1].trigger = false;
			
		} else {
			
			fsm.fsm_status[1].trigger = true;
			
		}
			
		//Print results to the Serial Monitor
		Serial.println();
		Serial.println("Servo 0 Trigger Status: " + String(fsm.fsm_status[0].trigger));
		Serial.println();
		Serial.println("Servo 1 Trigger Status: " + String(fsm.fsm_status[1].trigger));
		Serial.println();
			
		//Update the previous Millis count to the current Millis
		_previousMillis = _currentMillis;
			
	}

}


/*-----------------------------------------------------------------------
INTERRUPT SERVICE REQUEST (ISR) <-- Where the magic happens

Capture the interrupt setup by initPWM() and calculate the next position
of the servo(s).
-----------------------------------------------------------------------*/
ISR(TIMER1_CAPT_vect) {

	// ***
	// *** Call the appropriate FSM:
	// *** Calculates the microsecond value to add to the OCR
	// *** for each servo.  Uses fsm_status[0 or 1].attached
	// *** to determine which to calculate.
	// ***
	// *** Update the Output Control Registers with the calculated
	// *** microsecond value for OCR1A or OCR1B to modify pulse width
	// ***
	if(fsm.fsm_status[0].attached) {
		fsm.calculate(0);
		OCR1A = fsm.PWM_MIN_USEC + fsm.fsm_status[0].us_val;
	}
	
	if (fsm.fsm_status[1].attached) {
		fsm.calculate(1);
		OCR1B = fsm.PWM_MIN_USEC + fsm.fsm_status[1].us_val;
	}
	
}