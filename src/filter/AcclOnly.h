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
	void updateRate(	Vec3 z,   float rms);
	void updateAttitude(Quaternion Z, float rms);
	Vec3 getRate();
	Quaternion getAttitude();
	Quaternion getLastAttitude();
	Quaternion getRateQuaternion();
	float getRoll();
	float getPitch();
	float getYaw();
	float getRollRate();
	float getPitchRate();
	float getYawRate();
};
void
AcclOnly::update(InertialManager& sensors){
	//collect raw inertial readings
	float rawAccl[3];
	sensors.getLinAccel(rawAccl);

	//make accelerometer quaternion
	Vec3 tmp(-rawAccl[0], -rawAccl[1], rawAccl[2]);
	attitude = Quaternion(Vec3(0,0,1),tmp);
}
void
AcclOnly::updateRate(Vec3 z, float rateMSE){
	//rate.lerpWith(z, computeGain(RATE, rateMSE));
}
void
AcclOnly::updateAttitude(Quaternion Z, float attitudeMSE){
	//updateStateModel();
	//attitude.nlerpWith(Z, computeGain(ATTITUDE, attitudeMSE));
}
Vec3
AcclOnly::getRate(){
	return rate;
}
Quaternion
AcclOnly::getAttitude(){
	return attitude;
}
Quaternion
AcclOnly::getLastAttitude(){ //deprecated
	return attitude;
}
Quaternion
AcclOnly::getRateQuaternion(){
	return Quaternion(rate);
}
float
AcclOnly::getRollRate(){
	return rate[0];
}
float
AcclOnly::getPitchRate(){
	return rate[1];
}
float
AcclOnly::getYawRate(){
	return rate[2];
}

#endif
