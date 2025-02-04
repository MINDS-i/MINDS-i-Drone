#ifndef STATETIMER_H
#define STATETIMER_H

#include "Arduino.h"

class StateTimer {
  private:
    bool (*stateF)(void);
    uint32_t enterTime;
    bool lastState;

  public:
    /**
     * Construct a State Timer
     * @param stateF A fuction returning the state being observed
     */
    StateTimer(bool (*stateF)(void)) : stateF(stateF) {}
    /** Perform an observation of the state being tracked */
    void update() {
        bool state = stateF();
        if (state != lastState) {
            lastState = state;
            enterTime = millis();
        }
    }
    /**
     * Returns if the observed state has been true for `interval` milliseconds
     * @param  interval The milliseconds state must be true for
     * @return          If the state has been true for interval
     */
    bool trueFor(uint32_t interval) {
        update();
        return (lastState) && ((millis() - enterTime) > interval);
    }
    /**
     * Returns if the observed state has been false for `interval` milliseconds
     * @param  interval The milliseconds state must be false for
     * @return          If the state has been false for interval
     */
    bool falseFor(uint32_t interval) {
        update();
        return (!lastState) && ((millis() - enterTime) > interval);
    }
};

#endif
