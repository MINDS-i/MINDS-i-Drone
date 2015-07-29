#ifndef INERTIAL_MANAGER_H
#define INERTIAL_MANAGER_H
#include "filter/OrientationEngine.h"
#include "input/Sensor.h"
#include "input/AxisTranslator.h"
#include "Arduino.h"
/*
-Inertial Manager is given an array of Inertial Sensors on intialization.
-It will propogate commands to all the sensors it owns
-when an Inertial Sensor is updated, it will pass the data back up to the
	Inertial Manager, which will store the values for use by the orientation
	code
-Inertial Manager receives unit normalized input, and should not modify the
	values it receives in any way
*/

class InertialManager{
    friend class HMC5883L;
    friend class MPU6000;
    friend class L3GD20H;
    friend class LSM303D;
private:
	InertialVec** sensor;
    Translator*   translator;
	uint8_t numSensors;
    Vec3 accl; //G's
    Vec3 gyro; //Radians per millisecond
    Vec3 mag;  //(local earth field)'s
public:
	InertialManager(InertialVec** s, Translator* ts, uint8_t num)
		: sensor(s), translator(ts), numSensors(num) {}

    void update(){
        for(int i=0; i<numSensors; i++) sensor[i]->update(*this, translator[i]);
    }
    void start(){
        for(int i=0; i<numSensors; i++) sensor[i]->begin();
    }
    void calibrate(){
        for(int i=0; i<numSensors; i++) sensor[i]->calibrate();
    }
    void update(OrientationEngine &orientation){
        update();
        orientation.update(*this);
    }
    void stop(){
        for(int i=0; i<numSensors; i++) sensor[i]->end();
    }

    Vec3 getGyro(){
        return gyro;
    }
    const Vec3* gyroRef(){
        return &gyro;
    }
    Vec3 getAccl(){
        return accl;
    }
    const Vec3* acclRef(){
        return &accl;
    }
    Vec3 getMag(){
        return mag;
    }
    const Vec3* magRef(){
        return &mag;
    }
	void getRotRates(float& dx, float& dy, float& dz){
		dx = gyro[0];
		dy = gyro[1];
		dz = gyro[2];
	}
	void getRotRates(float (&v)[3]){
		v[0] = gyro[0];
		v[1] = gyro[1];
		v[2] = gyro[2];
	}
	void getLinAccel(float&  x, float&  y, float&  z){
		x = accl[0];
		y = accl[1];
		z = accl[2];
	}
	void getLinAccel(float (&v)[3]){
		v[0] = accl[0];
		v[1] = accl[1];
		v[2] = accl[2];
	}
	void getMagField(float&  x, float&  y, float&  z){
		x = mag[0];
		y = mag[1];
		z = mag[2];
	}
	void getMagField(float (&v)[3]){
		v[0] = mag[0];
		v[1] = mag[1];
		v[2] = mag[2];
	}
};
#endif
