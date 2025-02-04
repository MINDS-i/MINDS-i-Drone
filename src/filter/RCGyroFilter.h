#ifndef RCGyroFilter_H
#define RCGyroFilter_H

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

// rate correction filter with online gyro drift estimation
class RCGyroFilter : public OrientationEngine {
  private:
    /** If the filter is being run in calibrate mode */
    bool calMode;
    /** The inverse (negative) of the current gyro drift estimate */
    Vec3 rateCal;
    /** quaternion that rotates global frame vectors into the sensor frame */
    Quaternion attitude;
    /** The gain applied to the accelerometer/magnetometer corrections */
    float rcGain;
    /** The gain value controlling how quickly the gyro drift estimate updates*/
    float gdGain;
    /** a count the readings stored in rateCal for averaging when calibrating */
    float calTrack;
    /** Stores the most recent rotation rate reading */
    Vec3 rate;
    /** Stores the most recent pitch, roll, and yaw angle */
    float pitch, roll, yaw;
    void updatePRY();

  public:
    RCGyroFilter(float rcgain, float rGain)
        : calMode(false), rateCal(0, 0, 0), attitude(), rcGain(rcgain), gdGain(rGain), pitch(0), roll(0), yaw(0) {}
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
    void setRateCorrectionGain(float g) { rcGain = g; }
    void setGyroDriftGain(float r) { gdGain = r; }
};
void RCGyroFilter::updatePRY() {
    pitch = attitude.getPitch();
    roll = attitude.getRoll();
    yaw = attitude.getYaw();
}
void RCGyroFilter::calibrate(bool calibrate) {
    if (calMode == true && calibrate == false) {
        if (calTrack != 0) {
            rateCal = rateCal / calTrack;
        }
    } else if (calMode == false && calibrate == true) {
        rateCal = Vec3();
        calTrack = 0;
    }
    calMode = calibrate;
}
void RCGyroFilter::update(InertialManager& sensors, float dt) {
    // This filter works by integrating the gyroscope while applying corrections
    // as rotation rate vectors derived from the absolute angular position
    // sensors. The rotation correction vectors are the cross products of
    // a sensor reading rotated into the global frame with the global reference
    // vector corresponding to that sensor. The rotation rate vector can then be
    // rotated back to the sensor frame and integrated along with the gyroscope
    // values. In the absence of any errors, the rotation correction vectors
    // would be equal to the gyroscope integration term, so they cane be used
    // to estimate the gyro drift while the filter is running.

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

    // Retreive Accelerometer data, rotate into global frame
    Vec3 rawA = sensors.getAccl();
    rawA.rotateBy(~attitude);

    // Retreive Magnetometer data, rotate into global frame
    Vec3 rawM = sensors.getMag();
    rawM.rotateBy(~attitude);

    // Calculate a correction vector that would reduce error between the mapped
    // accelerometer and magnetometer values and their respective global
    // reference vectors, "up" and "north"
    // Then rotate to the sensor frame
    Vec3 delta(-rawA[1], rawA[0], -rawM[1]);
    delta.rotateBy(attitude);

    // get the gyroscope value and apply gyro drift calibration terms
    rate = *sensors.gyroRef();
    rate *= dt;
    if (!calMode) {
        rate += rateCal;
        rateCal += gdGain * (delta - rate);
    } else {
        rateCal -= rate;
        calTrack++;
    }

    // Integrate the gyroscope and correction delta
    attitude.integrate(rate * (1.0 - rcGain) + delta * rcGain);

    // Normalize, check for errors, recalculate pitch/roll/yaw
    attitude.normalize();
    if (attitude.error())
        attitude = Quaternion();
    updatePRY();
}

#endif
