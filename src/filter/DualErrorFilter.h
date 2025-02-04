#ifndef DualErrorFilter_H
#define DualErrorFilter_H

#include "filter/OrientationEngine.h"
#include "input/InertialManager.h"
#include "math/Quaternion.h"
#include "math/SpatialMath.h"
#include "math/Vec3.h"
#ifdef STAND_ALONE_TEST
#include "micros.h"
#else
#include "util/profile.h"
#endif

class DualErrorFilter : public OrientationEngine {
  private:
    Quaternion attitude;
    float estimateMSE;
    bool calMode;
    float calTrack;
    float sysMSE;
    float acclMSE;
    float acclEF;
    Vec3 rate, rateCal;
    float pitch, roll, yaw;
    float computeGain(float& estimate, float MSE);
    void updateStateModel(float ms);
    void updatePRY();

  public:
    DualErrorFilter(float systemMSE, float accelerometerMSE, float acclErrorFact)
        : sysMSE(systemMSE), acclMSE(accelerometerMSE), acclEF(acclErrorFact) {}
    void update(InertialManager& sensors, float ms);
    void calibrate(bool mode);
    Quaternion getAttitude() { return attitude; }
    Vec3 getRate() { return rate; }
    float getPitchRate() { return rate[1]; }
    float getRollRate() { return rate[0]; }
    float getYawRate() { return rate[2]; }
    float getPitch() { return pitch; }
    float getRoll() { return roll; }
    float getYaw() { return yaw; }
    void setSysMSE(float mse) { sysMSE = mse; }
    void setAcclMSE(float mse) { acclMSE = mse; }
    void setAcclEF(float aEF) { acclEF = aEF; }
    // temporary
    Vec3 getRateCal() { return rateCal; }
};
void DualErrorFilter::updatePRY() {
    pitch = attitude.getPitch();
    roll = attitude.getRoll();
    yaw = attitude.getYaw();
}
float DualErrorFilter::computeGain(float& estimate, float MSE) {
    float gain = estimate / (estimate + MSE);
    estimate = (1. - gain) * estimate;
    return gain;
}
void DualErrorFilter::updateStateModel(float dt) {
    // propogate process errors
    estimateMSE += dt * dt * sysMSE;

    attitude.integrate(rate * dt);
}
void DualErrorFilter::update(InertialManager& sensors, float ms) {
    rate = *sensors.gyroRef();
    Vec3 rawA = sensors.getAccl();

    if (!calMode)
        rate += rateCal;
    else {
        rateCal -= rate;
        calTrack++;
    }

    // make accelerometer quaternion
    Quaternion accl(Vec3(0, 0, -1), rawA);

    // calculate adjusted accelerometer MSE

    float tmp = rawA.length() - 1.0f;
    float oE = fabs(tmp);

    float aMSE = acclMSE + acclEF * oE;

    // calculate gains
    float acclGain = computeGain(estimateMSE, aMSE);

    // run model and lerp
    updateStateModel(ms);
    if (attitude.error())
        attitude = accl;
    else
        attitude.nlerpWith(accl, acclGain);
    updatePRY();
}
void DualErrorFilter::calibrate(bool calibrate) {
    if (calMode == true && calibrate == false) {
        if (calTrack != 0) {
            rateCal = rateCal / calTrack;
            attitude = Quaternion();
        }
    } else if (calMode == false && calibrate == true) {
        rateCal = Vec3();
        calTrack = 0;
    }
    calMode = calibrate;
}
#endif
