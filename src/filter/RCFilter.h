#ifndef RCFilter_H
#define RCFilter_H
//rate correction filter - heavily based on mahoney filter
#include "input/InertialManager.h"
#include "filter/OrientationEngine.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "math/SpatialMath.h"
#ifdef STAND_ALONE_MATH
    #include "micros.h"
#else
    #include "util/profile.h"
#endif

class RCFilter : public OrientationEngine {
private:
    volatile uint32_t stateTime;
    Vec3              rateCal;
    Quaternion        attitude;
    float             wGain, rateGain;
    Vec3              rate;
    float             pitch, roll, yaw;
    void updateStateModel(Vec3 correction);
    void updateStateModel();
    void updatePRY();
public:
    RCFilter(float gain, float rGain)
        :stateTime(0),
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
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.f;
	attitude.integrate(rate*dt + correction);
}
void
RCFilter::updateStateModel(){
    //keep track of passing time
    float dt = (micros()-stateTime);
    stateTime = micros();
    dt /= 1000.f;
    attitude.integrate(rate*dt);
}
void
RCFilter::calibrate(bool calibrate){
    hello
    this
    is
    code
    that doesn't
    compile
    because
    you need
    to
    only
    reset
    on change
    if(calibrate) {
        attitude = Quaternion(); //reset state when calibrate is called
        rateCal  = Vec3();
    }
}
void
RCFilter::update(InertialManager& sensors){
	//collect raw inertial readings
	float g[3];
    float a[3];
    float m[3];
    sensors.getRotRates(g);
    sensors.getLinAccel(a);
    sensors.getMagField(m);

    //make gyro vector
    Vec3 gyro( g[1], g[0],-g[2]);
    Vec3 rawA(-a[1],-a[0], a[2]);
    Vec3 rawM( m[1], m[0], m[2]);

    //reference frame mahoney filter 2.2ms
    //delta is build from cross products rawAxdown and rawMxnorth, which both
    //equal the derivatives of the quaternion mapping the paired vectors onto
    //eachother. Since rawA and rawM were already rotated into the global frame
    //by the best estimate attitude, the composite delta is the rate
    //that should be applied to the current rotation to map together the vectors
    //by pre-multiplication. Taking the conjugate with attitude transforms it
    //to a post-multiplication that can be combined with the gyro rate
    //update step
    rate = gyro;
    //updateStateModel();
    rawA.rotateBy(~attitude);
    rawM.rotateBy(~attitude);
    Vec3 delta(rawA[1], -rawA[0], -rawM[1]);//post rotation correction delta
    //attitude.preintegrate(delta*wGain);
    delta.rotateBy(attitude);

    rateCal += delta;
    delta   *= wGain;
    updateStateModel(delta*wGain + rateCal*rateGain);

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

    if(attitude.error()) attitude = Quaternion();
    attitude.normalize();
    updatePRY();
}
#endif
