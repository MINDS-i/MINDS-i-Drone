#ifndef DualErrorFilter_H
#define DualErrorFilter_H

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

class DualErrorFilter : public OrientationEngine {
private:
	DualErrorParams   params;
	Quaternion 		  attitude;
	Vec3 			  rate;
	float 			  estimateMSE;
	volatile uint32_t stateTime;
	float computeGain(float& estimate, float MSE);
	void updateStateModel();
public:
	DualErrorFilter():                  params(1,1,0) {}
	DualErrorFilter(DualErrorParams p): params(p)     {}
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
	float gain;
	float getAcclGain(){
		return gain;
	}
	void  setParams(DualErrorParams p){ params = p; }
};
float
DualErrorFilter::computeGain(float& estimate, float MSE){
	float gain = estimate/(estimate+MSE);
	estimate = (1.-gain)*estimate;
	return gain;
}
void
DualErrorFilter::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.;

	//propogate process errors
	estimateMSE += dt*dt*params.sysMSE;

	attitude.integrate(rate*dt);
}
void
DualErrorFilter::update(InertialManager* sensors){
	//collect raw inertial readings
	float rawGyro[3], rawAccl[3];
	sensors->getRotRates(rawGyro[0],rawGyro[1],rawGyro[2]);
	sensors->getLinAccel(rawAccl[0],rawAccl[1],rawAccl[2]);

	//make gyro vector
	Vec3 gyro = Vec3(-rawGyro[0],
					 -rawGyro[1],
					  rawGyro[2]);

	//make accelerometer quaternion
	Vec3 raw(-rawAccl[0], -rawAccl[1], rawAccl[2]);
	Quaternion accl(Vec3(0,0,1), raw);
	//calculate adjusted accelerometer MSE
	float aMSE = params.acclMSE
				+params.acclEF *fabs(log(raw.length()));

	//calculate gains
	float acclGain = computeGain(estimateMSE, aMSE);
	gain = acclGain;

	//run model and lerp
	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = accl;
	else 				 attitude.nlerpWith(accl, acclGain);
}
void
DualErrorFilter::updateRate(Vec3 z, float rateMSE){
	//rate.lerpWith(z, computeGain(RATE, rateMSE));
}
void
DualErrorFilter::updateAttitude(Quaternion Z, float attitudeMSE){
	//updateStateModel();
	//attitude.nlerpWith(Z, computeGain(ATTITUDE, attitudeMSE));
}
Vec3
DualErrorFilter::getRate(){
	return rate;
}
Quaternion
DualErrorFilter::getAttitude(){
	return attitude;
}
Quaternion
DualErrorFilter::getLastAttitude(){ //deprecated
	return attitude;
}
Quaternion
DualErrorFilter::getRateQuaternion(){
	return Quaternion(rate);
}
float
DualErrorFilter::getRollRate(){
	return rate[0];
}
float
DualErrorFilter::getPitchRate(){
	return rate[1];
}
float
DualErrorFilter::getYawRate(){
	return rate[2];
}

#endif
