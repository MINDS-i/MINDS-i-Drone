#ifndef PIDEXTERNALTIME_H
#define PIDEXTERNALTIME_H
#include "Arduino.h"
#include "util/PIDparameters.h"

class PIDexternaltime{
private:
    PIDparameters* param;
    float setPoint;
    float previous;
    float acc;
    boolean stopped;
public:
    PIDexternaltime(PIDparameters* pid): param(pid), setPoint(0),
                                         previous(0), acc(0),
                                         stopped(true) {}
    void tune(PIDparameters* pid){ param = pid; }
    void clearAccumulator(){ train(0); }
    void train(float out){
        acc = constrain(out, param->lowerBound, param->upperBound);
    }
    void set(float input){
        setPoint = input;
        stopped = false;
    }
    void stop() {
        clearAccumulator();
        stopped = true;
    }
    /**
     * Update the PID controller
     * current - the current process value
     * ms - milliseconds since last update
     */
    float update(float current, float ms){
        const float dt = min(ms/1000.0, 1.0); //convert to seconds, cap at 1

        const float error  = setPoint-current;
        const float newAcc = acc + param->I*error*dt;

        const float output = param->P * error
                           + newAcc
                           + param->D * (previous-current)/dt;

        previous = current;

        if (output > param->upperBound) {
            acc = min(acc, newAcc); //only let acc decrease
            return param->upperBound;
        } else if (output < param->lowerBound) {
            acc = max(acc, newAcc); //only let acc increase
            return param->lowerBound;
        }
        //to prevent integral windup, we only change the integral if the output
        //is not fully saturated
        acc = newAcc;

        return output;
    }
};

#endif
