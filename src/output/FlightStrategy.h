#ifndef FLIGHT_STRATEGY_H
#define FLIGHT_STRATEGY_H
#include "DroneLibs.h"

// abstract class for flight strategies
// Should take an orientation engine and set an array of 4 torques
// torques are pitch, roll, yaw, throttle
class FlightStrategy{
public:
    virtual void update(OrientationEngine&, float (&torques)[4]);
    virtual void reset();
};
#endif
