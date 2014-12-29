#ifndef MEGAINTERRUPT_H
#define MEGAINTERRUPT_H

volatile boolean _IN_LONG_INT = true;
volatile boolean _LONG_INT_OVERLAP = false;
void (*_ISRCallback)(void);

#if defined(__AVR_ATmega32U4__) //Leonardos
	ISR(TIMER3_COMPA_vect){
		char oldSREG = SREG;
		if(_IN_LONG_INT) {
			_LONG_INT_OVERLAP = true;
			SREG = oldSREG;
			return;
		}
		else _IN_LONG_INT = true;
		sei();

		_ISRCallback();

		_IN_LONG_INT=false;
		SREG = oldSREG;
	}

	void startInterrupt(void (*callback)(void), uint16_t periodInUs){
		uint8_t oldSREG = SREG;
		cli();
		_ISRCallback = callback;
		TCCR3A = 0;
		TCCR3B = (1<<WGM32) | //CTC mode
				 (1<<CS32)  | (1<<CS30); //1024 prescalar
		TIMSK3 = (1<<OCIE3A); //interrupt of compare to OCR3A
		OCR3AH = 0;
		OCR3AL = (microsecondsToClockCycles(periodInUs)/1024);
		TCNT3H = 0;
		TCNT3L = 0;
		_IN_LONG_INT = false;
		SREG = oldSREG;
	}
#else
	ISR(TIMER2_COMPA_vect){
		char oldSREG = SREG;
		if(_IN_LONG_INT){
			_LONG_INT_OVERLAP = true;
			SREG = oldSREG;
			return;
		}
		else _IN_LONG_INT = true;
		sei();

		_ISRCallback();

		_IN_LONG_INT=false;
		SREG = oldSREG;
	}

	void startInterrupt(void (*callback)(void), uint16_t periodInUs){
		_ISRCallback = callback;
		cli();
		TCCR2A = (1<<WGM21); //CTC mode
		TCCR2B = (1<<CS20) | (1<<CS21) | (1<<CS22); //1024 prescalar
		TIMSK2 = (1<<OCIE2A); //interrupt of compare to OCR2A
		OCR2A  = (microsecondsToClockCycles(periodInUs)/1024);
		sei();
		_IN_LONG_INT = false;
	}
#endif

#endif
