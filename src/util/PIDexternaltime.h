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
    /** Set the pid parameters to be used */
    void tune(PIDparameters* pid){ param = pid; }
    /** Clear the accumulator to have no bias */
    void clearAccumulator(){ train(0); }
    /** Set the controller so the accumulator has a bias of `out` */
    void train(float out){
        acc = constrain(out, param->lowerBound, param->upperBound);
    }
    /** Set the setpoint used in the pid controller */
    void set(float input){
        setPoint = input;
        stopped = false;
    }

    float get()
    {
        return setPoint;
    }

    /**
     * Stop the pid controller. This clears the accumulator.
     * It will output 0 until the next time `set` is called
     */
    void stop() {
        clearAccumulator();
        stopped = true;
    }

    /**
     * Return if pid is stopped
     */
    boolean isStopped() { return stopped; }

    /**
     * Update the PID controller
     * current - the current process value
     * ms - milliseconds since last update
     */
    float update(float current, float ms){
        if(stopped) return 0;

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

        if(!isfinite(newAcc)){
            acc = 0.0;
            return 0.0;
        } else {
            acc = newAcc;
            return output;
        }
    }
};

#endif
