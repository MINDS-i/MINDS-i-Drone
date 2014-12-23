#ifndef DualErrorFilter_H
#define DualErrorFilter_H

#include "filter/OrientationEngine.h"
#include "math/quaternion.h"
#include "math/SpatialMath.h"
#include "math/SpatialMath.h"
#include "math/vector.h"

class DualErrorFilter : public OrientationEngine {
private:
	static const uint8_t RATE = 0;
	static const uint8_t ATTITUDE = 1;

	math::quaternion attitude;
	math::vector3d   rate;
	math::vector3d   oldRate;//for midpoint intergration method
	float estimateMSE[2];
	float growthMSE[2];
	uint32_t stateTime;
	float computeGain(uint8_t select, float MSE);
	void updateStateModel();
public:
	DualErrorFilter(): attitude(1,0,0,0), rate(0,0,0) {
		estimateMSE[RATE] 		= 1;
		estimateMSE[ATTITUDE] 	= 1;
		growthMSE[RATE] 		= 1;
		growthMSE[ATTITUDE] 	= 1;
	}
	DualErrorFilter(float growthRateMSE,  float growthAttitudeMSE,
					float initialRateMSE, float initialAttitudeMSE):
			attitude(1,0,0,0), rate(0,0,0) {
		growthMSE[RATE] 		= growthRateMSE;
		growthMSE[ATTITUDE] 	= growthAttitudeMSE;
		estimateMSE[RATE] 		= initialRateMSE;
		estimateMSE[ATTITUDE] 	= initialAttitudeMSE;
	}
	DualErrorFilter(float growthRateMSE,  float growthAttitudeMSE):
			attitude(1,0,0,0), rate(0,0,0) {
		growthMSE[RATE] 		= growthRateMSE;
		growthMSE[ATTITUDE] 	= growthAttitudeMSE;
		estimateMSE[RATE] 		= 1;
		estimateMSE[ATTITUDE] 	= 1;
	}
	void update(math::vector3d z, math::quaternion Z,
				float rateMSE, float attitudeMSE, boolean relativeYaw);
	void updateRate(	math::vector3d z,   float rms);
	void updateAttitude(math::quaternion Z, float rms);
	math::vector3d   getRate();
	math::quaternion getAttitude();
	math::quaternion getLastAttitude();
	math::quaternion getRateQuaternion();
	float getRoll();
	float getPitch();
	float getYaw();
	float getRollRate();
	float getPitchRate();
	float getYawRate();
	void  setRateGrowthMSE(float input){ growthMSE[RATE]=input; }
	void  setAttitudeGrowthMSE(float input){ growthMSE[ATTITUDE]=input; }
};
float
DualErrorFilter::computeGain(uint8_t select, float MSE){
	float gain = estimateMSE[select]/(estimateMSE[select]+MSE);
	estimateMSE[select] = (1.-gain)*estimateMSE[select];
	return gain;
}
void
DualErrorFilter::updateStateModel(){
	//keep track of passing time, avoiding unecessarily short updates
	float dt = (micros()-stateTime);
	if(dt < 2500) return;
	stateTime = micros();
	dt /= 1000.;

	//propogate process errors
	estimateMSE[ATTITUDE] += dt*dt*estimateMSE[RATE];
	for(int i=0; i<2; i++) estimateMSE[i] += growthMSE[i]*dt*dt;


	//apply midpoint rule to rates and intergrate to find new attitude
	dt /= 2;
	math::quaternion dr = fromEuler((rate.x+oldRate.x)*dt,
									(rate.y+oldRate.y)*dt,
									(rate.z+oldRate.z)*dt);
	if(!isnan(dr._s)) {
		attitude *= dr;
	}
	oldRate = rate;
}
void
DualErrorFilter::update(math::vector3d   z,
						math::quaternion Z,
						float rateMSE,
						float attitudeMSE,
						boolean relativeYaw){
	rate = lerp(z, rate, computeGain(RATE, rateMSE));
	updateStateModel();

	//If attitude input cannot distinguish yaw, use this to only track rel yaw
	if(relativeYaw){
		math::quaternion tilt = getLastAttitude();
		tilt._v.x = 0; tilt._v.y = 0;
		//normalize given x and y = 0
		float tiltMag = invSqrt(tilt._v.z*tilt._v.z + tilt._s*tilt._s);
		tilt._v.z *= tiltMag;
		tilt._s   *= tiltMag;

		Z = tilt*Z;
	}

	attitude = nlerp(Z, attitude, computeGain(ATTITUDE, attitudeMSE));
}
void
DualErrorFilter::updateRate(math::vector3d z, float rateMSE){
	rate =  lerp(z, rate, computeGain(RATE, rateMSE));
}
void
DualErrorFilter::updateAttitude(math::quaternion Z, float attitudeMSE){
	updateStateModel();
	attitude = nlerp(Z, attitude, computeGain(ATTITUDE, attitudeMSE));
}
math::vector3d
DualErrorFilter::getRate(){
	return rate;
}
math::quaternion
DualErrorFilter::getAttitude(){
	updateStateModel();
	return attitude;
}
math::quaternion
DualErrorFilter::getLastAttitude(){
	return attitude;
}
math::quaternion
DualErrorFilter::getRateQuaternion(){
	return fromEuler(rate.x, rate.y, rate.z);
}
float
DualErrorFilter::getRoll(){
	float const q0 (attitude.Scalar());
	float const q1 (attitude.Vector().x);
	float const q2 (attitude.Vector().y);
	float const q3 (attitude.Vector().z);
	return atan2 (2 * ((q0*q1) + (q2*q3)), 1 - 2 * ((q1*q1) + (q2*q2)));
}
float
DualErrorFilter::getPitch(){
	float const q0 (attitude.Scalar());
	float const q1 (attitude.Vector().x);
	float const q2 (attitude.Vector().y);
	float const q3 (attitude.Vector().z);
	return asin  (2 * ((q0*q2) - (q3*q1)));
}
float
DualErrorFilter::getYaw(){
	float const q0 (attitude.Scalar());
	float const q1 (attitude.Vector().x);
	float const q2 (attitude.Vector().y);
	float const q3 (attitude.Vector().z);
	return atan2 (2 * ((q0*q3) + (q1*q2)), 1 - 2 * ((q2*q2) + (q3*q3)));
}
float
DualErrorFilter::getRollRate(){
	return rate.x;
}
float
DualErrorFilter::getPitchRate(){
	return rate.y;
}
float
DualErrorFilter::getYawRate(){
	return rate.z;
}

#endif
