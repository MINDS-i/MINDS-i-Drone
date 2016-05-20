#pragma once
#include <util/atomic.h>

#define TIMER_NUM 1

#define EXPCAT(A,B,C) EXPANDEDCONCATENATE(A,B,C)
#define EXPANDEDCONCATENATE(A,B,C) A ## B ## C
#define TIMER_ISR(N,V) TIMER_ISR_EXP(N,V)
#define TIMER_ISR_EXP(N,V) TIMER ## N ## _ ## V ## _vect

namespace {
    constexpr volatile uint8_t& TCCRA = EXPCAT(TCCR, TIMER_NUM,A);
    constexpr volatile uint8_t& TCCRB = EXPCAT(TCCR, TIMER_NUM,B);
    constexpr volatile uint8_t& TCCRC = EXPCAT(TCCR, TIMER_NUM,C);
    constexpr volatile uint8_t& TIFR  = EXPCAT(TIFR, TIMER_NUM, );
    constexpr volatile uint8_t& TIMSK = EXPCAT(TIMSK,TIMER_NUM, );
    constexpr volatile uint16_t& ICR  = EXPCAT(ICR,  TIMER_NUM, );
    constexpr volatile uint16_t& TCNT = EXPCAT(TCNT, TIMER_NUM, );
    constexpr volatile uint16_t& OCRA = EXPCAT(OCR,  TIMER_NUM,A);
    constexpr volatile uint16_t& OCRB = EXPCAT(OCR,  TIMER_NUM,B);

    class Output{
    public:
        Output(): highTime(0xffff), pinMask(0), pinReg(0) {}
        uint16_t highTime;
        uint8_t pinMask;
        volatile uint8_t* pinReg;
        bool enabled() const volatile { return pinMask != 0; }
        void disable() volatile { pinMask = 0; pinReg = 0; highTime = 0xffff; }
        void setPin(int p) volatile {
            pinMask = digitalPinToBitMask(p);
            pinReg = portOutputRegister(digitalPinToPort(p));
        }
        void setHigh() const volatile { *pinReg |= pinMask; }
        void setLow() const volatile { *pinReg &= ~pinMask; }
    };

    constexpr uint8_t MAX_OUTPUTS = 8;
    volatile Output output[MAX_OUTPUTS];
    volatile uint8_t activeOutputs = 0;

    class Action{ //lawsuit
    public:
        uint16_t time;
        uint8_t channel;
    };
    Action actions[MAX_OUTPUTS+1];
    Action * next = actions;
    // constructor attribute makes this get run once before main
    void setupActions() __attribute__ ((constructor));
    void setupActions() {
        actions[MAX_OUTPUTS] = {0xffff, 0};
        for(uint8_t i=0; i<MAX_OUTPUTS; i++){
            actions[i] = {0xffff, i};
        }
    }

    constexpr uint8_t PRESCALAR = 8;
    //works for prescalars less than or equal to 16
    constexpr uint8_t TICKS_PER_MS = (F_CPU / (PRESCALAR * 1e6L));

    int16_t constexpr intervalFromMicros(uint32_t us){
        return us * TICKS_PER_MS;
    }
}

namespace ServoGenerator{
    void set(int channel, uint16_t us){
        output[channel].highTime = intervalFromMicros(us);
    }

    void disable(int channel){
        if(output[channel].enabled()){
            output[channel].disable();
            activeOutputs--;
        }
    }

    bool enable(int channel, int pin){
        pinMode(pin, OUTPUT);
        output[channel].setPin(pin);
        if(!output[channel].enabled()){
            activeOutputs++;
            return true;
        }
        return false;
    }

    void setup(uint16_t refreshIntervalMicroseconds){
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
            // CTC, clear when TCNT == ICR, prescalar = 8
            TCCRA = 0;
            TCCRB = _BV(WGM13) | _BV(WGM12) | _BV(CS11);

            // enable the ICF (TCNT==ICR) and OCRA (TCNT==OCRA) interrupt
            TIMSK |= _BV(ICIE1);
            TIMSK |= _BV(OCIE1A);

            ICR  = intervalFromMicros(refreshIntervalMicroseconds);
            OCRA = 0xffff;

            // clear the timer count and pending interrupts
            TCNT   = 0;
            TIFR  |= _BV(OCF1A);
            TIFR  |= _BV(ICF1);
        }
    }

    class Servo{
        uint8_t channel;
        Servo(): channel(-1) {
            setup(20000);
        }
        Servo(uint16_t frameUs): channel(-1) {
            setup(frameUs);
        }
        bool attach(uint8_t arduinopin){
            //find open channel, try to attach
            uint8_t ch = -1;
            for(uint8_t i; i<MAX_OUTPUTS; i++){
                if(!output[i].enabled()){
                    ch = i;
                    break;
                }
            }
            if(ch == -1) return false;
            enable(ch, arduinopin);
            channel = ch;
            return true;
        }
        void detach(){
            if(channel != -1)
                disable(channel);
            channel = -1;
        }
        void write(uint8_t sig){
            if(channel != -1)
                set(channel, sig*10 + 600); // convert [0,180] to [600,2400]
        }
        void writeMicroseconds(uint16_t us){
            if(channel != -1)
                set(channel, us);
        }
        bool attached() {
            return channel != -1;
        }
    };
}

ISR(TIMER_ISR(TIMER_NUM, COMPA)){
    do {
        // stops when it finds an output with OCRA=0xffff at
        // the end of actions[]
        output[next->channel].setLow();
        next++;
        OCRA = next->time;
    } while (OCRA <= TCNT);
}

ISR(TIMER_ISR(TIMER_NUM, CAPT)){
    //update highTime values from outputs
    for(uint8_t i=0; i<MAX_OUTPUTS; i++){
        actions[i].time = output[actions[i].channel].highTime;
    }
    //sort actions by time
    for(uint8_t i=1; i<MAX_OUTPUTS; i++){
        uint16_t time = actions[i].time;
        uint8_t j = i;
        while(j>0 && time < actions[j-1].time){
            Action a = actions[j-1];
            actions[j-1] = actions[j];
            actions[j] = a;
            j--;
        }
    }
    //set all signals high
    //this preserves order because TCNT is monotonically increasing
    for(uint8_t i=0; i<activeOutputs; i++){
        output[actions[i].channel].setHigh();
        actions[i].time += TCNT;
    }

    next = actions;
    OCRA = next->time;
}

