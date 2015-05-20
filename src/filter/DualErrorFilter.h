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
	bool			  calMode;
	float			  calTrack;
	DualErrorParams   &params;
	Quaternion 		  attitude;
	Vec3 			  rate, rateCal;
	float 			  estimateMSE;
	volatile uint32_t stateTime;
	float computeGain(float& estimate, float MSE);
	void updateStateModel();
public:
	DualErrorFilter(DualErrorParams &p): params(p) {}
	void update(InertialManager& sensors);
	void calibrate(bool mode);
	Quaternion getAttitude(){ return attitude; }
	Vec3  getRate(){ return rate; }
	float getPitchRate(){ return rate[0]; }
	float getRollRate(){  return rate[1]; }
	float getYawRate(){   return rate[2]; }
	//temporary
	Vec3  getRateCal(){ return rateCal; }
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
DualErrorFilter::update(InertialManager& sensors){
	//collect raw inertial readings
	float rawGyro[3], rawAccl[3];
	sensors.getRotRates(rawGyro);
	sensors.getLinAccel(rawAccl);

	//make gyro vector
	Vec3 gyro = Vec3(-rawGyro[0],
					 -rawGyro[1],
					  rawGyro[2]);

	if(!calMode) gyro += rateCal;
	else {
		rateCal -= gyro;
		calTrack++;
	}

	//make accelerometer quaternion
	Vec3 raw(-rawAccl[0], -rawAccl[1], rawAccl[2]);
	Quaternion accl(Vec3(0,0,1), raw);
	//calculate adjusted accelerometer MSE
	float aMSE = params.acclMSE
				+params.acclEF *fabs(log(raw.length()));

	//calculate gains
	float acclGain = computeGain(estimateMSE, aMSE);

	//run model and lerp
	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = accl;
	else 				 attitude.nlerpWith(accl, acclGain);
}
void
DualErrorFilter::calibrate(bool calibrate){
	if(calMode == true && calibrate == false){
		if(calTrack != 0) rateCal = rateCal/calTrack;
	} else if (calMode == false && calibrate == true){
		rateCal  = Vec3();
		calTrack = 0;
	}
	calMode = calibrate;
}
#endif
