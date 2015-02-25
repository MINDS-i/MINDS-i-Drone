#ifndef DualErrorFilter_H
#define DualErrorFilter_H

#include "input/InertialManager.h"
#include "filter/OrientationEngine.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "math/SpatialMath.h"
#include "micros.h"
#include "DualErrorParams.h"

class DualErrorFilter : public OrientationEngine {
private:
	static const uint8_t RATE = 0;
	static const uint8_t ATTITUDE = 1;
	DualErrorParams params;
	Quaternion 		attitude;
	Vec3 			rate, oldRate;//for midpoint intergration method
	float 			estimateMSE[2];
	volatile uint32_t 		stateTime;
	float computeGain(uint8_t select, float MSE);
	void updateStateModel();
public:
	DualErrorFilter(): attitude(1,0,0,0), rate(0,0,0), params(1,1,1,1,0) {}
	DualErrorFilter(DualErrorParams p):
			attitude(1,0,0,0), rate(0,0,0), params(p) {}
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
	void  setParams(DualErrorParams p){ params = p; }
};
float
DualErrorFilter::computeGain(uint8_t select, float MSE){
	float gain = estimateMSE[select]/(estimateMSE[select]+MSE);
	estimateMSE[select] = (1.-gain)*estimateMSE[select];
	return gain;
}
void
DualErrorFilter::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.;

	//propogate process errors
	estimateMSE[ATTITUDE] += dt*dt*estimateMSE[RATE];
	for(int i=0; i<2; i++) estimateMSE[i] += params.systemMSE[i]*dt*dt;

	//apply midpoint rule to rates and intergrate to find new attitude
	dt /= 2;
	Vec3 gyro( -(rate[1]+oldRate[1])*dt,
			   -(rate[0]+oldRate[0])*dt,
			    (rate[2]+oldRate[2])*dt);

	//attitude.rotateByFast(gyro);	
	Quaternion tmp = Quaternion(gyro);
	tmp.rotateBy(attitude);
	attitude = tmp;
	
	oldRate = rate;
}
void
DualErrorFilter::update(InertialManager* sensors){
	//collect raw inertial readings
	float rawGyro[3], rawAccl[3];
	sensors->getRotRates(rawGyro[0],rawGyro[1],rawGyro[2]);
	sensors->getLinAccel(rawAccl[0],rawAccl[1],rawAccl[2]);
	
	//make gyro vector
	Vec3 gyro = Vec3(rawGyro[0], 
					 rawGyro[1],
					 rawGyro[2]);
	
	//make accelerometer quaternion
	Vec3 tmp(-rawAccl[1], rawAccl[0], rawAccl[2]);
	float tmag = tmp.length();
	float vmag = tmp[2]/tmag;
	tmp[2] = 0;
	tmp.normalize();
	Quaternion accl(tmp, acos(vmag));
	
	//calculate adjusted accelerometer MSE
	float aMSE = params.sensorMSE[params.ACCL]
				+params.acclErrorFac*fabs(log(tmag));
	std::cout << fabs(log(tmag)) << '\t' << aMSE << '\t';
	
	//calculate gains
	float acclGain = computeGain(ATTITUDE, aMSE);
	float gyroGain = computeGain(RATE, params.sensorMSE[params.GYRO]);
	
	std::cout << acclGain << '\t' << gyroGain << '\t';
	
	//run model and lerp
	rate.lerpWith(gyro, gyroGain);
	updateStateModel();	
	if(attitude.error()) attitude = accl;
	else 				 attitude.nlerpWith(accl, acclGain);
	std::cout << std::endl;
}
void
DualErrorFilter::updateRate(Vec3 z, float rateMSE){
	rate.lerpWith(z, computeGain(RATE, rateMSE));
}
void
DualErrorFilter::updateAttitude(Quaternion Z, float attitudeMSE){
	updateStateModel();
	attitude.nlerpWith(Z, computeGain(ATTITUDE, attitudeMSE));
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
