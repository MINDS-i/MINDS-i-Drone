#pragma once

class Timer16b{
public:
    volatile uint8_t&  TCCRA;
    volatile uint8_t&  TCCRB;
    volatile uint8_t&  TIFR;
    volatile uint8_t&  TIMSK;
    volatile uint16_t& TCNT;
};

#define TIMER_NUM 1
#define TIMER_REGS(N) TIMER_REGS_EXP(N)
#define TIMER_REGS_EXP(N) {TCCR##N##A, TCCR##N##B, TIFR##N, TIMSK##N, TCNT##N}
#define TIMER_ISR(N,V) TIMER_ISR_EXP(N,V)
#define TIMER_ISR_EXP(N,V) TIMER ## N ## _ ## V ## _vect

constexpr Timer16b regs = TIMER_REGS(TIMER_NUM);

ISR(TIMER_ISR(TIMER_NUM, COMPA)){

}
