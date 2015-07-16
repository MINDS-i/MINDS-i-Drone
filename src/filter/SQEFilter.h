#ifndef SQEFilter_H
#define SQEFilter_H

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

class SQEFilter : public OrientationEngine {
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
    float             pitch, roll, yaw;
	float computeGain(float& estimate, float MSE);
	void updateStateModel();
    void updatePRY();
public:
	SQEFilter(float systemMSE, float accelerometerMSE, float acclErrorFact)
        :sysMSE(systemMSE), acclMSE(accelerometerMSE), acclEF(acclErrorFact) {}
	void update(InertialManager& sensors);
    void calibrate(bool mode);
    Quaternion getAttitude(){ return attitude; }
    Vec3  getRate(){ return rate; }
    float getPitchRate(){ return rate[1]; }
    float getRollRate(){  return rate[0]; }
    float getYawRate(){   return rate[2]; }
    float getRoll(){  return roll; }
    float getPitch(){ return pitch;}
    float getYaw(){   return yaw;  }
    void setSysMSE(float mse) { sysMSE  = mse; }
    void setAcclMSE(float mse){ acclMSE = mse; }
    void setAcclEF(float aEF) { acclEF  = aEF; }
};
void
SQEFilter::updatePRY(){
    pitch = attitude.getPitch();
    roll  = attitude.getRoll();
    yaw   = attitude.getYaw();
}
float
SQEFilter::computeGain(float& estimate, float MSE){
	float gain = estimate/(estimate+MSE);
	estimate = (1.-gain)*estimate;
	return gain;
}
void
SQEFilter::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1024.f;
    if(dt >= 250) return;
	//propogate process errors
	estimateMSE += dt*dt*sysMSE;

	attitude.integrate(rate*dt);
}
void
SQEFilter::update(InertialManager& sensors){
	//collect raw inertial readings
	float g[3];
    float a[3];
    float m[3];
    sensors.getRotRates(g);
    sensors.getLinAccel(a);
    sensors.getMagField(m);

    //make gyro vector
    Vec3 gyro( g[1], g[0],-g[2]);
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

    /*
    Taken from
    FAST QUATERNION ATTITUDE ESTIMATION FROM TWO VECTOR MEASUREMENTS
    by F. Landis Markley

    Integrates accelerometer and magnetometer, but assuming the
    accelerometer is far mare accurate (to pitch/roll from mag)
    */
    Vec3 M = rawA; M.crossWith(rawM);
    Vec3 b1Cr1 = rawA; b1Cr1.crossWith(down);
    Vec3 b3Cr3 = M; b3Cr3.crossWith(east);
    float b1r1P1 = 1 + rawA.dot(down);
    Vec3  b1Pr1  = rawA+down;
    float U = b1r1P1*(M.dot(east)) - (down.dot(M))*(rawA.dot(east));
    float V = b1Pr1.dot(b3Cr3);
    float P = sqrt(U*U + V*V);

    float C1,C2;
    if(U>0){
        C1 = (P+U);
        C2 = V;
    } else {
        C1 = V;
        C2 = (P-U);
    }

    b1Cr1 *= C1;
    b1Pr1 *= C2;

    float MdotR = b1r1P1*C1;
    Quaternion wahba(MdotR, b1Cr1[0]+b1Pr1[0],
                            b1Cr1[1]+b1Pr1[1],
                            b1Cr1[2]+b1Pr1[2] );
    wahba.normalize();


	//calculate adjusted accelerometer MSE
	float aMSE = acclMSE
				+acclEF *fabs(log(rawA.length()));

	//calculate gains
	float wGain = computeGain(estimateMSE, aMSE);

	//run model and lerp
	rate = gyro;
	updateStateModel();
	if(attitude.error()) attitude = wahba;
	else 				 attitude.nlerpWith(wahba, wGain);
    updatePRY();
}
void
SQEFilter::calibrate(bool calibrate){
    if(calibrate == false){
        if(calTrack == 0) return;
        rateCal = rateCal/calTrack;

        //averaging samples built into normalization
        north.normalize();
        down.normalize();

        //transform down and north into earth frame
        Quaternion level(Vec3(0,0,1), down);
        down = Vec3(0,0,1);
        north.rotateBy(~level);

        // get east by cross product and normalize
        east = down;
        east.crossWith(north);
        east.normalize();

        attitude = Quaternion();
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
