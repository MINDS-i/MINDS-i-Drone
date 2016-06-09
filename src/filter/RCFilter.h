#ifndef RCFilter_H
#define RCFilter_H
//rate correction filter - heavily based on mahoney filter
#include "input/InertialManager.h"
#include "filter/OrientationEngine.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "math/SpatialMath.h"
#ifdef STAND_ALONE_TEST
    #include "micros.h"
#else
    #include "util/profile.h"
#endif

class RCFilter : public OrientationEngine {
private:
    volatile uint32_t stateTime;
    bool              calMode;
    Vec3              rateCal;
    Quaternion        attitude;
    float             wGain, rateGain;
    float             calTrack;
    Vec3              rate;
    float             pitch, roll, yaw;
    void updateStateModel(Vec3 correction);
    void updateStateModel(){ updateStateModel(Vec3()); }
    void updatePRY();
public:
    RCFilter(float gain, float rGain)
        :stateTime(0), calMode(false),
         rateCal(0,0,0),
         attitude(),
         wGain(gain), rateGain(rGain),
         pitch(0), roll(0), yaw(0)
         {}
    void update(InertialManager& sensors);
    void calibrate(bool mode);
    Quaternion getAttitude(){ return attitude; }
    Vec3  getRate(){ return rate; }
    float getPitchRate(){ return rate[1]; }
    float getRollRate(){  return rate[0]; }
    float getYawRate(){   return rate[2]; }
    float getRoll(){  return roll; }
    float getPitch(){ return pitch;}
    float getYaw(){   return yaw;  }
    void setwGain(float g) { wGain = g; }
    void setRateGain(float r) { rateGain = r; }
};
void
RCFilter::updatePRY(){
    pitch = attitude.getPitch();
    roll  = attitude.getRoll();
    yaw   = attitude.getYaw();
}
void
RCFilter::updateStateModel(Vec3 correction){
	//keep track of passing time
    uint32_t cTime = micros();
    float dt = (cTime-stateTime);
    stateTime = cTime;
	dt /= 1000.f;
	attitude.integrate(rate*dt + correction);
}
void
RCFilter::calibrate(bool calibrate){
    if(calMode == true && calibrate == false){
        if(calTrack != 0) {
            rateCal = rateCal/calTrack;
        }
    } else if (calMode == false && calibrate == true){
        rateCal  = Vec3();
        calTrack = 0;
    }
    calMode = calibrate;
}
void
RCFilter::update(InertialManager& sensors){
    rate = *sensors.gyroRef();
    Vec3 rawA = sensors.getAccl();
    Vec3 rawM = sensors.getMag();

    //reference frame mahoney filter 2.0ms
    //delta is built from cross products rawAx(-down) and rawMxnorth, which both
    //equal the derivative of the quaternions mapping the paired vectors onto
    //eachother. Since rawA and rawM were already rotated into the global frame
    //by the best estimate attitude, the composite delta is the delta
    //that should be applied to the current rotation to become the mapping
    //by pre-multiplication. Taking the conjugate with attitude transforms it
    //to a post-multiplication that can be combined with the gyro rate
    //update step

    rawA.rotateBy(~attitude);
    rawM.rotateBy(~attitude);
    Vec3 delta(-rawA[1], rawA[0], -rawM[1]);//post rotation correction delta
    delta.rotateBy(attitude);

    if(!calMode) rate += rateCal;
    else {
        rateCal -= rate;
        calTrack++;
    }
    updateStateModel(delta*wGain);

    /*
    rateCal  = rateCal*rateGain + delta*(1.0f-rateGain);
    updateStateModel(delta*wGain + rateCal);
    */

    attitude.normalize();
    if(attitude.error()) attitude = Quaternion();
    updatePRY();
}
/*
//madgewick/mahoney filter 2.6ms
rawM.normalize();
Vec3 tm = rawM;
tm.rotateBy(~attitude);
//const float tmlen = sqrt(tm[0]*tm[0] + tm[1]*tm[1]);

Vec3 d = down;
Vec3 n = Vec3(tm[0], 0, tm[2]);
d.rotateBy(attitude);
n.rotateBy(attitude);
rawA.crossWith(d);
rawM.crossWith(n);
const Vec3 delta = (rawA + rawM)*wGain;
rate = gyro;
updateStateModel(delta);
*/
#endif
