#include "ServoGenerator.h"

#define EXPCAT(A,B,C) EXPANDEDCONCATENATE(A,B,C)
#define EXPANDEDCONCATENATE(A,B,C) A ## B ## C
#define TIMER_ISR(N,V) TIMER_ISR_EXP(N,V)
#define TIMER_ISR_EXP(N,V) TIMER ## N ## _ ## V ## _vect

using namespace ServoGenerator;

namespace {
    volatile uint8_t& TCCRA = EXPCAT(TCCR, TIMER_NUM,A);
    volatile uint8_t& TCCRB = EXPCAT(TCCR, TIMER_NUM,B);
    volatile uint8_t& TCCRC = EXPCAT(TCCR, TIMER_NUM,C);
    volatile uint8_t& TIFR  = EXPCAT(TIFR, TIMER_NUM, );
    volatile uint8_t& TIMSK = EXPCAT(TIMSK,TIMER_NUM, );
    volatile uint16_t& ICR  = EXPCAT(ICR,  TIMER_NUM, );
    volatile uint16_t& TCNT = EXPCAT(TCNT, TIMER_NUM, );
    volatile uint16_t& OCRA = EXPCAT(OCR,  TIMER_NUM,A);
    volatile uint16_t& OCRB = EXPCAT(OCR,  TIMER_NUM,B);

    class Output{
    public:
        Output(): highTime(0xffff), pinMask(0), pinReg(0) {}
        uint16_t highTime;
        uint8_t pinMask;
        volatile uint8_t* pinReg;
        void disable() volatile {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
                pinMask = 0;
                pinReg = 0;
                highTime = 0xffff;
            }
        }
        void setPin(int p) volatile {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
                pinMask = digitalPinToBitMask(p);
                pinReg = portOutputRegister(digitalPinToPort(p));
            }
        }
        bool enabled() const volatile { return pinMask != 0; }
        void setHigh() const volatile { *pinReg |= pinMask; }
        void setLow() const volatile { *pinReg &= ~pinMask; }
    };

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

    // function pointer and state guard for frame update callbacks
    volatile UpdateFunc frameCallback;
    volatile bool IN_FRAME_CALLBACK;

    constexpr uint8_t PRESCALAR = 8;
    //works for prescalars less than or equal to 16
    constexpr uint8_t TICKS_PER_MS = (F_CPU / (PRESCALAR * 1e6L));

    uint16_t constexpr intervalFromMicros(uint16_t us){
        return us * TICKS_PER_MS;
    }

    uint16_t constexpr microsFromInterval(uint16_t ticks){
        return ticks / TICKS_PER_MS;
    }
}

namespace ServoGenerator{
    bool begun;

    void set(uint8_t channel, uint16_t us){
        output[channel].highTime = intervalFromMicros(us);
    }

    void disable(uint8_t channel){
        if(output[channel].enabled()){
            output[channel].disable();
            activeOutputs--;
        }
    }

    bool enable(uint8_t channel, int pin){
        bool pass = false;
        if(!output[channel].enabled()){
            activeOutputs++;
            pass = true;
            if(!begun) begin(DEFAULT_REFRESH_INTERVAL);
        }
        pinMode(pin, OUTPUT);
        output[channel].setPin(pin);
        return pass;
    }

    void begin(uint16_t refreshIntervalMicroseconds){
        begun = true;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
            // CTC (WGM 12), clear when TCNT == ICR, prescalar = 8
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

    void setUpdateCallback(UpdateFunc callback){
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
            frameCallback = callback;
        }
    }


    Servo::Servo(): channel(-1) {}
    bool Servo::attach(uint8_t arduinopin){
        if(channel != -1) return false; //already attached

        //find open channel, try to attach
        uint8_t ch = 0;
        for(; ch<MAX_OUTPUTS; ch++){
            if(!output[ch].enabled()) break;
        }
        if(ch < MAX_OUTPUTS){
            enable(ch, arduinopin);
            channel = ch;
            return true;
        }
        return false;
    }
    void Servo::detach(){
        if(channel != -1)
            disable(channel);
        channel = -1;
    }
    bool Servo::attached() {
        return channel != -1;
    }
}

ISR(TIMER_ISR(TIMER_NUM, COMPA)){
    do {
        output[next->channel].setLow();
        next++;
        uint16_t t = next->time;
        OCRA = t;

        if(t > TCNT) break;
        //clear any interrupt that may have been generated when OCRA was set
        else TIFR = _BV(OCF1A);
    } while(true);
}

ISR(TIMER_ISR(TIMER_NUM, CAPT)){
    //update each channels highTime; 0xffff if the channel is off
    for(uint8_t i=0; i<MAX_OUTPUTS; i++){
        actions[i].time = output[actions[i].channel].highTime;
    }
    //run insertion sort on actions[]; optimised with a cycle count benchmark
    for(uint8_t i=1; i<MAX_OUTPUTS; i++){
        //load the time and channel of action[i] only when needed
        uint16_t time = actions[i].time;
        //quickly check for sorted input
        if(time >= actions[i-1].time) continue;
        //swap up the data to make room for actions[i]
        uint8_t channel = actions[i].channel;
        uint8_t j = i;
        while(j>0 && time < actions[j-1].time){
            // copying all the components individually is slightly faster
            actions[j].time = actions[j-1].time;
            actions[j].channel = actions[j-1].channel;
            j--;
        }
        //put the data from actions[i] back into actions[j]
        actions[j].time = time;
        actions[j].channel = channel;
    }
    //set all signals high
    //this preserves order because TCNT is monotonically increasing
    for(uint8_t i=0; i<activeOutputs; i++){
        output[actions[i].channel].setHigh();
        actions[i].time += TCNT;
    }

    next = actions;
    OCRA = next->time;

    if(frameCallback != NULL && IN_FRAME_CALLBACK == false){
        IN_FRAME_CALLBACK = true;
        NONATOMIC_BLOCK(NONATOMIC_FORCEOFF){
            frameCallback(microsFromInterval(ICR));
        }
        IN_FRAME_CALLBACK = false;
    }
}

