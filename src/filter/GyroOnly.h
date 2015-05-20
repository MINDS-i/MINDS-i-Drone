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
	void update(InertialManager& sensors);
	void calibrate(bool mode);
	Quaternion getAttitude(){ return attitude; }
	Vec3  getRate(){ return rate;}
	float getPitchRate(){ return rate[0]; }
	float getRollRate(){  return rate[1]; }
	float getYawRate(){   return rate[2]; }
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
GyroOnly::calibrate(bool mode){
}
void
GyroOnly::update(InertialManager& sensors){
	//collect raw inertial readings
	float rawGyro[3];
	sensors.getRotRates(rawGyro);

	//make gyro vector
	Vec3 gyro = Vec3(-rawGyro[0],
					 -rawGyro[1],
					  rawGyro[2]);

	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = Quaternion();
	attitude.normalize();
}

#endif
