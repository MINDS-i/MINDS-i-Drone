#ifndef FLIGHT_STRATEGY_H
#define FLIGHT_STRATEGY_H

#include "filter/OrientationEngine.h"

class FlightStrategy{
public:
    /**
     * Calculate output torques for a airborn craft
     * orientation - an orientation model for the current craft
     * ms - milliseconds since the last update call
     * torques - output array of torques (pitch, roll, yaw, throttle)
     *              ranging from 0 to 1; throttle is the total percentage output
     *              of the motors and each angular torque is the percentage
     *              imbalance on each axis, counter clockwise positive
     */
    virtual void update(OrientationEngine&, float ms, float (&torques)[4]);
    /**
     * Clear any state or error accumulators used internaly; prepare for
     * fresh calculations
     */
    virtual void reset();
};
#endif
