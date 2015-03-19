#ifndef GYROONLY_H
#define GYROONLY_H

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

class GyroOnly : public OrientationEngine {
private:
	Quaternion 		  attitude;
	Vec3 			  rate;
	volatile uint32_t stateTime;
	void updateStateModel();
public:
	GyroOnly(){}
	void update(InertialManager* sensors);
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
GyroOnly::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.;

	attitude.integrate(rate*dt);
}
void
GyroOnly::update(InertialManager* sensors){
	//collect raw inertial readings
	float rawGyro[3];
	sensors->getRotRates(rawGyro[0],rawGyro[1],rawGyro[2]);

	//make gyro vector
	Vec3 gyro = Vec3(-rawGyro[0],
					 -rawGyro[1],
					 rawGyro[2]);

	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = Quaternion();
	attitude.normalize();
}
void
GyroOnly::updateRate(Vec3 z, float rateMSE){
	//rate.lerpWith(z, computeGain(RATE, rateMSE));
}
void
GyroOnly::updateAttitude(Quaternion Z, float attitudeMSE){
	//updateStateModel();
	//attitude.nlerpWith(Z, computeGain(ATTITUDE, attitudeMSE));
}
Vec3
GyroOnly::getRate(){
	return rate;
}
Quaternion
GyroOnly::getAttitude(){
	return attitude;
}
Quaternion
GyroOnly::getLastAttitude(){ //deprecated
	return attitude;
}
Quaternion
GyroOnly::getRateQuaternion(){
	return Quaternion(rate);
}
float
GyroOnly::getRollRate(){
	return rate[0];
}
float
GyroOnly::getPitchRate(){
	return rate[1];
}
float
GyroOnly::getYawRate(){
	return rate[2];
}

#endif
