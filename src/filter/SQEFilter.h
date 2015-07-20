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
    volatile bool     calMode;
    volatile float    calTrack;
    Vec3              rateCal;

    float             wGain;
    Quaternion        attitude;
    Vec3              rate;
    volatile uint32_t stateTime;

    float             pitch, roll, yaw;
    Vec3              north, east, down;
    void updateStateModel();
    void updatePRY();
public:
	SQEFilter(float gain)
        :calMode(false), calTrack(0), rateCal(0,0,0),
         wGain(gain), attitude(), rate(0,0,0),
         stateTime(0),
         pitch(0), roll(0), yaw(0),
         north(1,0,0), east(0,1,0), down(0,0,1)
         {}
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
    void setwGain(float g) { wGain = g; }
};
void
SQEFilter::updatePRY(){
    pitch = attitude.getPitch();
    roll  = attitude.getRoll();
    yaw   = attitude.getYaw();
}
void
SQEFilter::updateStateModel(){
	//keep track of passing time
	float dt = (micros()-stateTime);
	stateTime = micros();
	dt /= 1000.f;
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

    if(calMode == true){
        rateCal -= gyro;
        down    += rawA;
        north   += rawM;
        calTrack = calTrack +1;
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
    const Vec3 b1 = rawA;
    const Vec3 b2 = rawM;
    Vec3 b3 = b1; b3.crossWith(b2);
    const Vec3 r1 = down;
    const Vec3 r2 = north;
    Vec3 r3 = r1; r3.crossWith(r2);
    Vec3 b3crossr3 = b3; b3crossr3.crossWith(r3);
    const float U = (1.0f+b1.dot(r1))*(b3.dot(r3)) - (b1.dot(r3))*(r1.dot(b3));
    const float V = (b1 + r1).dot(b3crossr3);
    const float P = sqrt(U*U + V*V);

    float c1, c2;
    if(U > 0){
        c1 = P+U;
        c2 = V;
    } else {
        c1 = V;
        c2 = P-U;
    }

    const float w = c1 * (1.0f + b1.dot(r1));
    Vec3 b1crossr1 = b1; b1crossr1.crossWith(r1);
    b1crossr1 *= c1;
    Vec3 b1plusr1 = b1 + r1;
    b1plusr1 *= c2;

    Quaternion wahba(w
                    ,b1crossr1[0] + b1plusr1[0]
                    ,b1crossr1[1] + b1plusr1[1]
                    ,b1crossr1[2] + b1plusr1[2] );
    wahba.normalize();

    //run model and lerp
    rate = gyro;
    updateStateModel();
    if(attitude.error()) attitude = wahba;
	else 				 attitude.nlerpWith(wahba, wGain);
    updatePRY();
}
void
SQEFilter::calibrate(bool calibrate){
    if(calibrate == false && calMode == true){
        if(calTrack != 0)
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

        //reset orientation
        attitude = Quaternion();
        rate     = Vec3();
    } else if (calibrate == true && calMode == false){
        rateCal  = Vec3(0,0,0);
        north    = Vec3(0,0,0);
        east     = Vec3(0,0,0);
        down     = Vec3(0,0,0);
        calTrack = 0;
    }

    calMode = calibrate;
}
#endif
