#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "util/Interval.h"

class Altitude {
  private:
    /** The interval in microseconds between altitude update calculations*/
    const uint32_t UPDATE_INTERVAL = 10000;
    // Altitude state variables
    float altitudeEst;
    float velocityEst;
    // Filter gain variables
    float barometerGain;
    float velocityGain;

  public:
    Altitude() {}
    void setBarometerGain(float g) { barometerGain = g; }
    void setVelocityGain(float g) { velocityGain = g; }
    float getAltitude() { return altitudeEst; }
    float getVelocity() { return velocityEst; }
    /** Initialize altitude tracking */
    void setup(float inputAltitude) {
        altitudeEst = inputAltitude;
        velocityEst = 0.0;
    }
    /** Update altitude model */
    void update(float inputAltitude) {
        static auto timer = Interval::timer();
        uint32_t dt = timer();
        if (dt > UPDATE_INTERVAL) {
            timer.reset();

            float C0 = barometerGain;
            float C1 = velocityGain;

            float altitude = inputAltitude * C0 + altitudeEst * (1.0 - C0);

            float dSecondsInv = 1E6 / ((float)dt);
            float newvelocity = (altitude - altitudeEst) * dSecondsInv;
            float velocity = newvelocity * C1 + velocityEst * (1.0 - C1);

            altitudeEst = (isnan(altitude)) ? 0.0 : altitude;
            velocityEst = (isnan(velocity)) ? 0.0 : velocity;
        }
    }
};

#endif
