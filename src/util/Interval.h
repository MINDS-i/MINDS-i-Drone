#ifndef INTERVAL_H
#define INTERVAL_H

namespace Interval{
    class RepeatedInterval{
        uint32_t next;
        uint32_t span;
    public:
        RepeatedInterval(uint32_t span): span(span) { next = millis() + span; }
        boolean operator()() {
            if(millis() > next){
                next += span;
                return true;
            }
            return false;
        }
    };
    /**
     * Returns a functor that returns true once every `milliseconds`
     * @param  milliseconds The interval time
     * @return              A functor
     */
    RepeatedInterval every(uint32_t milliseconds){
        return RepeatedInterval(milliseconds);
    }

    class SingleInterval{
        uint32_t endTime;
    public:
        SingleInterval(uint32_t t): endTime(t) {}
        boolean operator()() { return millis() > endTime; }
    };
    /**
     * Returns a functor that returns false until `milliseconds` have elapsed
     * @param  milliseconds The interval time
     * @return              A functor
     */
    SingleInterval elapsed(uint32_t milliseconds){
        return SingleInterval( millis() + milliseconds);
    }

    class Timer{
        uint32_t lastCall;
    public:
        Timer(){ reset(); }
        void reset() { lastCall = micros(); }
        uint32_t operator()() {
            return micros() - lastCall;
        }
    };
    Timer timer(){
        return Timer();
    }
}

#endif
