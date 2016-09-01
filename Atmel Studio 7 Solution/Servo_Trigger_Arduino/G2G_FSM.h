/*--------------------------------------------------------------------------
Title: G2G_FSM.h
Version 0.1

Originally written by: Byron Jacquot
Original Title:  ServoTrigger.c
Device: Sparkfun Servo Trigger
Created: 7/3/2014 3:43:10 PM

Modified for the Arduino by: Kevin Gagnon
Created: 8/31/2016

Twitter: @GadgetsToGrow
---------------------------------------------------------------------------*/

#ifndef _G2G_FSM_h
#define _G2G_FSM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class G2G_FSM {


	/*-------------
	PUBLIC
	-------------*/
	public:
	
	G2G_FSM();
	~G2G_FSM();
	void initPWM(void);
	void releasePWM(void);
	bool attachServoFSM(uint8_t servoID);
	bool detachServoFSM(uint8_t servoID);
	void calculate(uint8_t servoID);

	static const int32_t PWM_MIN_USEC   = 1000; // narrowest pulse is 1000 usec
	static const int32_t PWM_RANGE_USEC = 1000; // diff between min and max PWM pulses.

	/*--------------------
	AVAILABLE FSM MODES
	--------------------*/
	typedef enum FSM_MODE
	{
		ASTABLE = 0,
		BISTABLE,
		ONESHOT
		
	} FSM_MODE;

	/*------------------------
	CURRENT SERVO FSM STATE
	------------------------*/
	typedef enum SERVO_STATE
	{
		eIDLE = 0,		
		eATOB,			
		eATTOP,			
		eBTOA,			
		eWAIT_TO_RESET	
		
	} SERVO_STATE;
	
	/*-------------
	FSM STATUS
	-------------*/
	typedef struct FSM_STATUS
	{
		int32_t positionA = 0;		// latest ADC value for A pot
		int32_t positionB = 0;		// latest ADC value for B pot
		int32_t travelTime = 0;		// latest ADC value for T pot
		bool rising;				// A --> B = rising, B -- > A = falling		
		bool trigger;				// is trigger asserted? (move to next state)
		bool attached;				// is servo attached?
		FSM_MODE fsmMode;			// ASTABLE, BISTABLE, etc.
		SERVO_STATE servo_state;	// eIDLE, e
		int32_t phasor;				// counts 0 to 0xffff - needs to be signed to catch overflow
		int32_t us_val;				// microseconds to add to the PWM pulse after calculate()
		
	}FSM_STATUS;
	
	/*------------------------------------------------------
	Array to hold fsm_status for each servo independently
	------------------------------------------------------*/
	FSM_STATUS fsm_status[2];
	
	/*-------------
	PRIVATE
	-------------*/
	private:
	//Calculation functions
	int16_t calcDelta(uint8_t servoID);	
	bool calcNextPhasor(uint8_t servoID, int16_t increment);
	int16_t scalePhasor(uint8_t servoID);
	//FSM functions
	void oneshotFSM(uint8_t servoID);
	void astableFSM(uint8_t servoID);
	void bistableFSM(uint8_t servoID);

};

#endif

