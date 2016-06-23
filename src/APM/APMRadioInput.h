#ifndef APMRADIOINPUT_H
#define APMRADIOINPUT_H

#include "Arduino.h"
#if defined(__AVR_ATmega2560__)

namespace APMRadio {
	volatile uint16_t pulse[8]; //APM2.* has 8 input channels
	volatile static uint16_t pTime;

	static const uint16_t SCALE    = 100;
	static const uint16_t MINPULSE = 14900;
	static const uint16_t MAXPULSE = MINPULSE + 180*SCALE;
	static const uint16_t DEFAULTP = MINPULSE +  90*SCALE;

	/**
	 * Start tracking the radio inputs on an APM 2.* board
	 * This makes use of TIMER5's input capture capability
	 */
	void setup(){
		pinMode(48, INPUT);   //timer5 is pin 48
		TCCR5A = 0;
		TCCR5B = 0;
		TCCR5B |= _BV(ICES5); //rising edge trigger
		TIMSK5 |= _BV(ICIE5); //enable enternal interrupt capture
	    TCCR5B |= _BV(CS50);  //set clock at 1x prescaler
	    for(int i=0; i<8; i++) pulse[i] = DEFAULTP;
	}

	/** Get the radio signal on channel `num` mapped between 0 and 180 */
	uint8_t get(uint8_t num){
		uint16_t val = constrain(pulse[num], MINPULSE, MAXPULSE);
		return (val-MINPULSE)/SCALE;
	}

	/** Get the raw radio signal on channel at its highest resolution */
	uint16_t raw(uint8_t num){
		return pulse[num];
	}
}

ISR(TIMER5_CAPT_vect){
	static uint8_t cNum; //channel Number
	uint16_t dt = ICR5 - APMRadio::pTime;

	if (dt > 42000) { //sync pulse detected
		cNum = 0;
	} else {
		APMRadio::pulse[cNum] = dt;
		cNum = (cNum+1) % 8;
	}
	APMRadio::pTime = ICR5;
}

#endif
#endif
