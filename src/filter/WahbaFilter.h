#ifndef WahbaFilter_H
#define WahbaFilter_H

#include "input/InertialManager.h"
#include "filter/OrientationEngine.h"
#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "math/SpatialMath.h"
#ifdef STAND_ALONE_MATH
	#include "micros.h"
#else
	#include "util/profile.h"
#endif

class WahbaFilter : public OrientationEngine {
private:
    float             sysMSE;
    float             acclMSE;
    float             acclEF;
	Quaternion 		  attitude;
	Vec3 			  rate;
	float 			  estimateMSE;
	volatile uint32_t stateTime;
	float computeGain(float& estimate, float MSE);
	void updateStateModel();
public:
	WahbaFilter(float systemMSE, float accelerometerMSE, float acclErrorFact)
        :sysMSE(systemMSE), acclMSE(accelerometerMSE), acclEF(acclErrorFact) {}
	void update(InertialManager& sensors);
    void calibrate(bool mode);
    Quaternion getAttitude(){ return attitude; }
    Vec3  getRate(){ return rate; }
    float getPitchRate(){ return rate[0]; }
    float getRollRate(){  return rate[1]; }
    float getYawRate(){   return rate[2]; }
    void setSysMSE(float mse) { sysMSE  = mse; }
    void setAcclMSE(float mse){ acclMSE = mse; }
    void setAcclEF(float aEF) { acclEF  = aEF; }
};
float
WahbaFilter::computeGain(float& estimate, float MSE){
	float gain = estimate/(estimate+MSE);
	estimate = (1.-gain)*estimate;
	return gain;
}
void
WahbaFilter::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.;

	//propogate process errors
	estimateMSE += dt*dt*sysMSE;

	attitude.integrate(rate*dt);
}
void
WahbaFilter::calibrate(bool mode){
}
void
WahbaFilter::update(InertialManager& sensors){
	//collect raw inertial readings
	float rawGyro[3];
	sensors.getRotRates(rawGyro);

	//make gyro vector
	Vec3 gyro = Vec3(-rawGyro[0],
					 -rawGyro[1],
					  rawGyro[2]);

	//make accelerometer quaternion




	//North and R need to be derived by a calibration soon
	static int start = 0;
    static Vec3 North(.40,-.11,-.91);
    static Vec3 Down(0,0,1);
    static Vec3 R;
	if(start == 0){
	    Vec3 R = Down; R.crossWith(North);
	    Down.normalize();
	    North.normalize();
	    R.normalize();
	    start = 1;
	}


    float a[3];
    float m[3];
    sensors.getLinAccel(a);
    sensors.getMagField(m);
    Vec3 rawA(-a[0],-a[1], a[2]);
    Vec3 rawM(m[0],m[1], m[2]);
    Vec3 M = rawA; M.crossWith(rawM);
	M.normalize();
    Vec3 bcr1 = rawA; bcr1.crossWith(Down);
    Vec3 bcr2 = rawM; bcr2.crossWith(North);
    Vec3 McrossR = M; McrossR.crossWith(R);
    Vec3 MplusR = M+R;
    Vec3  subCross = (bcr1+bcr2);
    float subDot   = (rawA.dot(Down)+rawM.dot(North));
    float MPRp1    = (1+M.dot(R));
    float A = (McrossR).dot(subCross) +
              MPRp1*subDot;
    float B = (M+R).dot(subCross);
    float Y = sqrt(A*A+B*B);

    float C1,C2;
    if(A>0){
        C1 = (Y+A);
        C2 = B;
    } else {
        C1 = B;
        C2 = (Y-A);
    }

    McrossR *= C1;
    MplusR  *= C2;

    float MdotR = MPRp1*C1;
    Quaternion wahba(MdotR, McrossR[0]+MplusR[0],
                          McrossR[1]+MplusR[1],
                          McrossR[2]+MplusR[2] );
    wahba.normalize();


	//calculate adjusted accelerometer MSE
	float aMSE = acclMSE
				+acclEF *fabs(log(rawA.length()+rawM.length()));

	//calculate gains
	float wGain = computeGain(estimateMSE, aMSE);
	gain = wGain;

	//run model and lerp
	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = wahba;
	else 				 attitude.nlerpWith(wahba, wGain);
}
#endif
