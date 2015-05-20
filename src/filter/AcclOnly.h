#ifndef ACCLONLY_H
#define ACCLONLY_H

#include "input/InertialManager.h"
#include "filter/OrientationEngine.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "math/SpatialMath.h"
#include "DualErrorParams.h"
#ifdef STAND_ALONE_MATH
	#include "micros.h"
#else
	#include "util/profile.h"
#endif

class AcclOnly : public OrientationEngine {
private:
	Quaternion 		  attitude;
	Vec3 			  rate;
public:
	AcclOnly(){}
	void update(InertialManager& sensors);
	void calibrate(bool mode);
	Quaternion getAttitude(){ return attitude; }
	Vec3  getRate(){ return rate;}
	float getPitchRate(){ return rate[0]; }
	float getRollRate(){  return rate[1]; }
	float getYawRate(){   return rate[2]; }
};
void
AcclOnly::calibrate(bool mode){
}
void
AcclOnly::update(InertialManager& sensors){
	//collect raw inertial readings
	float rawAccl[3];
	sensors.getLinAccel(rawAccl);

	//make accelerometer quaternion
	Vec3 tmp(-rawAccl[0], -rawAccl[1], rawAccl[2]);
	attitude = Quaternion(Vec3(0,0,1),tmp);
}
#endif
