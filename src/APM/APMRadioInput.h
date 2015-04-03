#ifndef APMRADIOINPUT_H
#define APMRADIOINPUT_H

#include "Arduino.h"
#if defined(__AVR_ATmega2560__)
volatile uint16_t _pulse[8]; //APM2.* has 8 input channels
volatile static uint16_t _pTime;

static const uint16_t _SCALE  	=   100;
static const uint16_t _MINPULSE = 14900;
static const uint16_t _MAXPULSE = _MINPULSE + 180*_SCALE;
static const uint16_t _DEFAULT 	= _MINPULSE +  90*_SCALE;

void setupAPM2radio(){
	pinMode(48, INPUT);   //timer5 is pin 48
	TCCR5A = 0;
	TCCR5B = 0;
	TCCR5B |= _BV(ICES5); //rising edge trigger
	TIMSK5 |= _BV(ICIE5); //enable enternal interrupt capture
    TCCR5B |= _BV(CS50);  //set clock at 1x prescaler
    for(int i=0; i<8; i++) _pulse[i] = _DEFAULT;
}

ISR(TIMER5_CAPT_vect){
	static uint8_t  cNum; //channel Number
	uint16_t dt = TCNT5-_pTime;

	if (dt > 42000) { //sync pulse detected
		cNum = 0;
	} else {
		_pulse[cNum] = dt;
		cNum = (cNum+1) % 8;
	}
	_pTime = TCNT5;
}

uint8_t getAPM2Radio(uint8_t num){
	uint16_t val = constrain(_pulse[num], _MINPULSE, _MAXPULSE);
	return (val-_MINPULSE)/_SCALE;
}

uint16_t getAPM2RadioRaw(uint8_t num){
	return _pulse[num];
}
#endif
#endif
