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
    bool              calMode;
    float             calTrack;
    Vec3              rateCal;

    float             sysMSE;
    float             acclMSE;
    float             acclEF;
    float             estimateMSE;
    Quaternion        attitude;
    Vec3              rate;
    volatile uint32_t stateTime;

    Vec3              north, east, down;
	float computeGain(float& estimate, float MSE);
	void updateStateModel();
public:
	WahbaFilter(float systemMSE, float accelerometerMSE, float acclErrorFact)
        :sysMSE(systemMSE), acclMSE(accelerometerMSE), acclEF(acclErrorFact) {}
	void update(InertialManager& sensors);
    void calibrate(bool mode);
    Quaternion getAttitude(){ return attitude; }
    Vec3  getRate(){ return rate; }
    float getPitchRate(){ return rate[1]; }
    float getRollRate(){  return rate[0]; }
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
WahbaFilter::update(InertialManager& sensors){
	//collect raw inertial readings
	float g[3];
    float a[3];
    float m[3];
    sensors.getRotRates(g);
    sensors.getLinAccel(a);
    sensors.getMagField(m);

    //make gyro vector
    Vec3 gyro( g[1], g[0], g[2]);
    Vec3 rawA(-a[1],-a[0], a[2]);
    Vec3 rawM( m[1], m[0], m[2]);


    if(calMode){
        rateCal -= gyro;
        down  += rawA;
        north += rawM;
        calTrack++;
        return;
    }
    gyro += rateCal;

    //what research paper did this come from?
    Vec3 M = rawA; M.crossWith(rawM);
	M.normalize();
    Vec3 bcr1 = rawA; bcr1.crossWith(down);
    Vec3 bcr2 = rawM; bcr2.crossWith(north);
    Vec3 McrossR = M; McrossR.crossWith(east);
    Vec3 MplusR = M+east;
    Vec3  subCross = (bcr1+bcr2);
    float subDot   = (rawA.dot(down)+rawM.dot(north));
    float MPRp1    = (1+M.dot(east));
    float A = (McrossR).dot(subCross) +
              MPRp1*subDot;
    float B = (M+east).dot(subCross);
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

	//run model and lerp
	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = wahba;
	else 				 attitude.nlerpWith(wahba, wGain);
}
void
WahbaFilter::calibrate(bool calibrate){
    if(calibrate == false){
        if(calTrack == 0) return;
        rateCal = rateCal/calTrack;

        //averaging samples built into normalization
        north.normalize();
        down.normalize();

        //transform down and north into earth frame
        Quaternion level(Vec3(0,0,1), down);
        down = Vec3(0,0,1);
        north.rotateBy(level);

        // get east by cross product and normalize
        east = down;
        east.crossWith(north);
        east.normalize();

    } else if (calibrate == true){
        rateCal  = Vec3();
        north    = Vec3();
        east     = Vec3();
        down     = Vec3(0,0,0);
        calTrack = 0;
    }
    calMode = calibrate;
}
#endif
