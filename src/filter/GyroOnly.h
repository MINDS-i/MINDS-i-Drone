#ifndef GYROONLY_H
#define GYROONLY_H

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

class GyroOnly : public OrientationEngine {
private:
	Quaternion 		  attitude;
	Vec3 			  rate;
	volatile uint32_t stateTime;
public:
	GyroOnly(){}
	void update(InertialManager& sensors, float ms);
	void calibrate(bool mode);
	Quaternion getAttitude(){ return attitude; }
	Vec3  getRate(){ return rate; }
	float getPitchRate(){ return rate[1]; }
	float getRollRate(){  return rate[0]; }
	float getYawRate(){   return rate[2]; }
};
void
GyroOnly::calibrate(bool mode){
}
void
GyroOnly::update(InertialManager& sensors, float dt){
    rate = *sensors.gyroRef();
	attitude.integrate(rate*dt);
	if(attitude.error()) attitude = Quaternion();
	attitude.normalize();
}

#endif
