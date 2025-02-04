#ifndef RCFilter_H
#define RCFilter_H

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

// rate correction filter - heavily based on mahoney filter
class RCFilter : public OrientationEngine {
  private:
    /** If the filter is being run in calibrate mode */
    bool calMode;
    /** The inverse (negative) of the current gyro drift estimate */
    Vec3 rateCal;
    /** quaternion that rotates global frame vectors into the sensor frame */
    Quaternion attitude;
    /** Weight applied to the accelerometer correction values */
    float accelGain;
    /** Weight applied to the magnetometer correction values */
    float magGain;
    /** a count the readings stored in rateCal for averaging when calibrating */
    float calTrack;
    /** Stores the most recent rotation rate reading */
    Vec3 rate;
    /** Stores the most recent pitch, roll, and yaw angle */
    float pitch, roll, yaw;
    void updatePRY();

  public:
    RCFilter(float gain, float rGain)
        : calMode(false), rateCal(0, 0, 0), attitude(), accelGain(gain), magGain(rGain), pitch(0), roll(0), yaw(0) {}
    void update(InertialManager& sensors, float ms);
    void calibrate(bool mode);
    Quaternion getAttitude() { return attitude; }
    Vec3 getRate() { return rate; }
    float getPitchRate() { return rate[1]; }
    float getRollRate() { return rate[0]; }
    float getYawRate() { return rate[2]; }
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }
    void setAccelGain(float g) { accelGain = g; }
    void setMagGain(float r) { magGain = r; }
};
void RCFilter::updatePRY() {
    pitch = attitude.getPitch();
    roll = attitude.getRoll();
    yaw = attitude.getYaw();
}
void RCFilter::calibrate(bool calibrate) {
    if (calMode == true && calibrate == false) {
        // Apply the average measured gyroscope value as a rate calibration
        if (calTrack != 0) {
            rateCal = rateCal / calTrack;
        }
    } else if (calMode == false && calibrate == true) {
        // reset the calibration track variables
        rateCal = Vec3();
        calTrack = 0;
    }
    calMode = calibrate;
}
void RCFilter::update(InertialManager& sensors, float dt) {
    // This filter works by integrating the gyroscope while applying corrections
    // as rotation rate vectors derived from the absolute angular position
    // sensors. The rotation correction vectors are the cross products of
    // a sensor reading rotated into the global frame with the global reference
    // vector corresponding to that sensor. The rotation rate vector can then be
    // preintegrated into attitude.

    // This implementation elides the calculation of the accelerometer-cross-up
    // and magnetometer-cross-north vectors and their sum; instead a single
    // Vector is calculated that is equal to that sum for efficiency.
    // Note that the magnetometers influence on pitch is removed because it
    // is considered less reliable than the accelerometer in that regard.

    // This differs form the Mahoney or Madgewick/Mahoney filter which calculate
    // the rotation correction vectors in the sensor frame. The reason for this
    // is that in the global frame isolating the influence of the magnetometer
    // to just the yaw axis requires more calculation.

    // An Euler Integraton of the rates is all that is necessary; the
    // approximation is far more accurate then the sensor readings without
    // any higher order calculations being required.

    // Calculations are done using a North-East-Down coordinate system

    // get the gyroscope value
    rate = *sensors.gyroRef();

    if (!calMode) {
        // Apply gyro drift calibration terms
        rate += rateCal;
    } else {
        // Integrate gyroscope readings
        rateCal -= rate;
        calTrack++;
    }

    // Integrate the gyroscope rate
    attitude.integrate(rate * dt);

    // Retreive Accelerometer data, rotate into global frame
    Vec3 rawA = sensors.getAccl();
    rawA.rotateBy(~attitude);

    // Retreive Magnetometer data, rotate into global frame
    Vec3 rawM = sensors.getMag();
    rawM.rotateBy(~attitude);

    // Calculate a correction vector that would reduce error between the mapped
    // accelerometer and magnetometer values and their respective global
    // reference vectors, "up" and "north"; apply the delta as an integration
    Vec3 delta(-accelGain * rawA[1], accelGain * rawA[0], -magGain * rawM[1]);

    // While calibrating, dramatically increase the correction gains to
    // force the attitude estimate to settle on the inertial sensor's readings
    if (calMode) {
        // Scale so the largest gain becomes 0.25
        delta *= 0.25 / max(accelGain, magGain);
    }

    attitude.preintegrate(delta);

    // Normalize, check for errors, recalculate pitch/roll/yaw
    attitude.normalize();
    if (attitude.error())
        attitude = Quaternion();
    updatePRY();
}

#endif
