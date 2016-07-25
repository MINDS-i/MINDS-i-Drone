#ifndef ALTITUDEHOLD_H
#define ALTITUDEHOLD_H

#include "Arduino.h"
#include "filter/Altitude.h"
#include "util/Interval.h"

class AltitudeHold{
private:
    /** The interval in milliseconds between altitude hold update calculations*/
    const uint32_t UPDATE_INTERVAL = 10;
    /** The interval in seconds between altituhe hold update calculations */
    const float DT = ((float) UPDATE_INTERVAL)/1000.0;
    /**
     * The minimum and maximum multiplier's away from hoverThrottle
     * that the output will ever go, effectivly setting a "liveband" on the
     * output relative the starting hover throttle
     */
    const float MIN_THROTTLE_RATIO = 0.66;
    const float MAX_THROTTLE_RATIO = 1.50;
    // Controller state variables
    float integral;
    float hover;
    float throttleOutput;
    // Response calibration variables
    float responseFactor;
    float velocityFactor;
    float integralFactor;
public:
    AltitudeHold() {}
    void setResponseFactor(float f) { responseFactor = f; }
    void setVelocityFactor(float f) { velocityFactor = f; }
    void setIntegralFactor(float f) { integralFactor = f; }
    /**
     * Initialize altitude hold mode
     * @param hoverThrottle The throttle value to start at in a hover
     */
    void setup(float hoverThrottle){
        hover = hoverThrottle;
        integral = 0.0;
        throttleOutput = 0.0;
    }
    /**
     * Update the altituhe hold controller and return the target throttle value
     * @param targetAltitude The desired altitude to fly at
     * @param measurements The altitude object tracking the quads current pos
     * @return Throttle value to apply to the craft [0,1]
     */
    float update(float targetAltitude, Altitude measurements){
        static auto timer = Interval::every(UPDATE_INTERVAL);
        if(timer()) {
            float error = targetAltitude-measurements.getAltitude();
            integral = integral + error*DT;
            float correction = error +
                               -measurements.getVelocity()*velocityFactor +
                               integral*integralFactor;
            throttleOutput = hover + responseFactor * correction;
        }
        return constrain(throttleOutput,
            hover*MIN_THROTTLE_RATIO,
            hover*MAX_THROTTLE_RATIO);
    }
};
#endif
