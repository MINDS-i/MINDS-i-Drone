#ifndef APMRADIOINPUT_H
#define APMRADIOINPUT_H

#include "Arduino.h"
#include "util/atomic.h"
#if defined(__AVR_ATmega2560__)

namespace APMRadio {
static const uint16_t SYNC_PULSE_LENGTH = 42000;
static const uint16_t SCALE = 100;
static const uint16_t MINPULSE = 14900;
static const uint16_t MAXPULSE = MINPULSE + 180 * SCALE;
static const uint16_t DEFAULTP = MINPULSE + 90 * SCALE;

volatile uint16_t pulse[8]; // APM2.* has 8 input channels

/**
 * Start tracking the radio inputs on an APM 2.* board
 * This makes use of TIMER5's input capture capability
 */
void setup() {
    pinMode(48, INPUT); // timer5 is pin 48
    TCCR5A = 0;
    TCCR5B = 0;
    TCCR5B |= _BV(ICES5); // rising edge trigger
    TIMSK5 |= _BV(ICIE5); // enable enternal interrupt capture
    TCCR5B |= _BV(CS50);  // set clock at 1x prescaler
    for (int i = 0; i < 8; i++) {
        pulse[i] = DEFAULTP;
    }
}

/** Get the raw radio signal on channel at its highest resolution */
uint16_t inline raw(uint8_t num) {
    uint16_t data;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { data = pulse[num]; }
    return data;
}

/** Get the radio signal on channel `num` mapped between 0 and 180 */
uint8_t inline get(uint8_t num) {
    uint16_t val = constrain(raw(num), MINPULSE, MAXPULSE);
    return (val - MINPULSE) / SCALE;
}
} // namespace APMRadio

ISR(TIMER5_CAPT_vect) {
    static uint8_t cNum; // channel Number
    static uint16_t previousTriggerTime;

    uint16_t dt = ICR5 - previousTriggerTime;
    previousTriggerTime = ICR5;

    if (dt > APMRadio::SYNC_PULSE_LENGTH) { // sync pulse detected
        cNum = 0;
    } else {
        APMRadio::pulse[cNum] = dt;
        cNum = (cNum + 1) % 8;
    }
}

#endif
#endif
