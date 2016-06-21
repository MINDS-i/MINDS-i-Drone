#ifndef PROFILE_H
#define PROFILE_H

#include "Arduino.h"
#include "util/atomic.h"

#if DEBUG
    namespace{
        const int NUM_PROFILES = 16;
        volatile uint32_t profile[NUM_PROFILES];
    }
    void __attribute__((always_inline)) inline tic(uint8_t i){
    	profile[i] = -micros();
    }
    void __attribute__((always_inline)) inline toc(uint8_t i){
    	profile[i] += micros();
    }
    uint32_t inline profileTime(uint8_t i){
        uint32_t v;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
            v = profile[i];
        }
        return v;
    }
#else
    void tic(uint8_t i) {}
    void toc(uint8_t i) {}
    uint32_t profileTime(uint8_t i) { return 0; }
#endif

#endif
