#ifndef FLIGHT_STRATEGY_H
#define FLIGHT_STRATEGY_H
// abstract class for flight strategies
// Should take an orientation engine and set an array of 4 torques
// torques are pitch, roll, yaw, throttle
class FlightStrategy{
public:
    // ms - time since last update in milliseconds
    virtual void update(OrientationEngine&, float ms, float (&torques)[4]);
    virtual void reset();
};
#endif
