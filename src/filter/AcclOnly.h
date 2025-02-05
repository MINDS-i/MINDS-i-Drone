#ifndef ACCLONLY_H
#define ACCLONLY_H

#include "filter/OrientationEngine.h"
#include "input/InertialManager.h"
#include "math/Quaternion.h"
#include "math/SpatialMath.h"
#include "math/Vec3.h"
#ifdef STAND_ALONE_TEST
#include "micros.h"
#endif

class AcclOnly : public OrientationEngine {
  private:
    Quaternion attitude;
    Vec3 rate;

  public:
    AcclOnly() {}
    void update(InertialManager& sensors, float ms);
    void calibrate(bool mode);
    Quaternion getAttitude() { return attitude; }
    Vec3 getRate() { return rate; }
    float getPitchRate() { return rate[0]; }
    float getRollRate() { return rate[1]; }
    float getYawRate() { return rate[2]; }
};
void AcclOnly::calibrate(bool mode) {}
void AcclOnly::update(InertialManager& sensors, float ms) {
    Vec3 rawA = sensors.getAccl();
    attitude = Quaternion(Vec3(0, 0, -1), rawA);
}
#endif
