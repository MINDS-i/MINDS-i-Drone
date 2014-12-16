#ifndef KalmanFilter_H
#define KalmanFilter_H

#include "filter/OrientationEngine.h"
#include "math/matrix.h"
#include "math/quaternion.h"
#include "math/SpatialMath.h"
#include "math/SpatialMath.h"
#include "math/vector.h"

class KalmanFilter : public OrientationEngine {
private:
	math::quaternion Attitude;
	math::vector3d   rate;
	math::vector3d   oldRate;//for midpoint intergration method
	math::matrix2d   Q;
	math::matrix2d   P;
	uint32_t gainTime, stateTime;
	math::matrix2d computeGain(math::matrix2d Ra);
	void updateStateModel();
public:
	KalmanFilter(): gainTime(0), stateTime(0),
					Attitude(1,0,0,0), rate(0,0,0), Q(1), P(1){}
	KalmanFilter(math::matrix2d errorGrowth, math::matrix2d initialError):
					gainTime(0), stateTime(0),
					Attitude(1,0,0,0), rate(0,0,0), Q(errorGrowth), P(initialError) {}
	void update(math::vector3d z, math::quaternion Z,
				float rateMSE, float attitudeMSE, boolean relativeYaw);
	void updateRate(	math::vector3d z,   float MSE);
	void updateAttitude(math::quaternion Z, float MSE);
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
};
math::matrix2d
KalmanFilter::computeGain(math::matrix2d Ra){
	float dt = (micros()-gainTime)/1000.;
	gainTime = micros();

	math::matrix2d omega(1, 0,
						 dt,0);
	math::matrix2d I(1);
	P = omega*P*omega.transpose();
	math::matrix2d K = P*(P+Ra).inverse();
	P = (I-K)*P;

	return K;
}
void
KalmanFilter::updateStateModel(){
	//keep track of passing time, avoiding unecessarily short updates
	float dt = (micros()-stateTime);
	if(dt < 2500.) return;
	dt /= 1000.;
	stateTime = micros();

	P += Q*dt*dt; //time based error growth

	//apply midpoint rule to rates and intergrate to find new attitude
	dt /= 2;
	math::quaternion dr = fromEuler((rate.x+oldRate.x)*dt,
									(rate.y+oldRate.y)*dt,
									(rate.z+oldRate.z)*dt);
	if(!isnan(dr._s)) {
		Attitude *= dr;
	}
	oldRate = rate;
}
void
KalmanFilter::update(math::vector3d   z,
					 math::quaternion Z,
					 float rateRMS,
					 float attitudeRMS,
					 boolean relativeYaw){
	math::matrix2d Ra(rateRMS, 			 0,
							0, attitudeRMS );
	math::matrix2d K = computeGain(Ra);
	updateStateModel();
	if(relativeYaw){
		math::quaternion tilt = getLastAttitude();
		tilt._v.x = 0; tilt._v.y = 0;
		tilt.normalize();
		Z = tilt*Z;
	}
	rate 	 =  lerp(z, rate, K(0,0));
	Attitude = nlerp(Z, Attitude, K(1,1));
}
void
KalmanFilter::updateRate(math::vector3d z, float rms){
	math::matrix2d Ra(	rms,   		0,
						  0, 10000000 );
	math::matrix2d K = computeGain(Ra);
	rate =  lerp(z, rate, K(0,0));
}
void
KalmanFilter::updateAttitude(math::quaternion Z, float rms){
	updateStateModel();
	math::matrix2d Ra(10000000,   0,
							 0, rms );
	math::matrix2d K = computeGain(Ra);
	Attitude = nlerp(Z, Attitude, K(1,1));
}
math::vector3d
KalmanFilter::getRate(){
	return rate;
}
math::quaternion
KalmanFilter::getAttitude(){
	updateStateModel();
	return Attitude;
}
math::quaternion
KalmanFilter::getLastAttitude(){
	return Attitude;
}
math::quaternion
KalmanFilter::getRateQuaternion(){
	return fromEuler(rate.x, rate.y, rate.z);
}
float
KalmanFilter::getRoll(){
	float const q0 (Attitude.Scalar());
	float const q1 (Attitude.Vector().x);
	float const q2 (Attitude.Vector().y);
	float const q3 (Attitude.Vector().z);
	return atan2 (2 * ((q0*q1) + (q2*q3)), 1 - 2 * ((q1*q1) + (q2*q2)));
}
float
KalmanFilter::getPitch(){
	float const q0 (Attitude.Scalar());
	float const q1 (Attitude.Vector().x);
	float const q2 (Attitude.Vector().y);
	float const q3 (Attitude.Vector().z);
	return asin  (2 * ((q0*q2) - (q3*q1)));
}
float
KalmanFilter::getYaw(){
	float const q0 (Attitude.Scalar());
	float const q1 (Attitude.Vector().x);
	float const q2 (Attitude.Vector().y);
	float const q3 (Attitude.Vector().z);
	return atan2 (2 * ((q0*q3) + (q1*q2)), 1 - 2 * ((q2*q2) + (q3*q3)));
}
float
KalmanFilter::getRollRate(){
	return rate.x;
}
float
KalmanFilter::getPitchRate(){
	return rate.y;
}
float
KalmanFilter::getYawRate(){
	return rate.z;
}

#endif
