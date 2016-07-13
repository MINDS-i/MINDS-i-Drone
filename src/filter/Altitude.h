#ifndef ALTITUDE_H
#define ALTITUDE_H

#include "input/APM/MS5611.h"
#include "util/Interval.h"

class Altitude{
private:
    /** The interval in milliseconds between altitude update calculations*/
    const uint32_t UPDATE_INTERVAL = 10;
    /** The interval in seconds between altituhe update calculations */
    const float DT = ((float) UPDATE_INTERVAL)/1000.0;
    // Altitude state variables
    float altitudeEst;
    float velocityEst;
    // Filter gain variables
    float barometerGain;
    float velocityGain;
public:
    Altitude(){}
    void setBarometerGain(float g){ barometerGain = g; }
    void setVelocityGain(float g){ velocityGain = g; }
    float getAltitude(){ return altitudeEst; }
    float getVelocity(){ return velocityEst; }
    /** Initialize altitude tracking */
    void altitudeSetup(MS5611 baro){
        altitudeEst = baro.getAltitude();
        velocityEst = 0.0;
    }
    /** Update altitude model */
    void altitudeUpdate(MS5611 baro){
        static auto timer = Interval::every(UPDATE_INTERVAL);
        if(timer()){
            float C0 = barometerGain;
            float C1 = velocityGain;

            float barometer = baro.getAltitude();
            float altitude = barometer*C0 + altitudeEst*(1.0-C0);

            float newvelocity = (altitude-altitudeEst) / DT;
            velocityEst = newvelocity*C1 + velocityEst*(1.0-C1);

            altitudeEst = altitude;
        }
    }
};

#endif
