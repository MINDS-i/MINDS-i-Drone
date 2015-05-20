#ifndef INERTIAL_MANAGER_H
#define INERTIAL_MANAGER_H
#include "filter/OrientationEngine.h"
#include "input/Sensor.h"
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
private:
	Sensor** sensor;
	uint8_t numSensors;
	float rotRates[3]; //X,Y,Z
	float linAccel[3];
	float magField[3];
	float pressure[1];
public:
	InertialManager(Sensor** s, uint8_t num)
		: sensor(s), numSensors(num) {}

	void update();
	void update(OrientationEngine &orientation);

	void start();
	void calibrate();
	void stop();

	//variable setters/getters - implicitly inlined

	//rad per millisecond
	void updateRotRates(float dx, float dy, float dz){
		rotRates[0] = dx;
		rotRates[1] = dy;
		rotRates[2] = dz;
	}
	void updateRotRates(float (&delta)[3]){
		rotRates[0] = delta[0];
		rotRates[1] = delta[1];
		rotRates[2] = delta[2];
	}
	void getRotRates(float& dx, float& dy, float& dz){//rad per millisecond
		dx = rotRates[0];
		dy = rotRates[1];
		dz = rotRates[2];
	}
	void getRotRates(float (&v)[3]){
		v[0] = rotRates[0];
		v[1] = rotRates[1];
		v[2] = rotRates[2];
	}
	//G's
	void updateLinAccel(float  x, float  y, float  z){
		linAccel[0] = x;
		linAccel[1] = y;
		linAccel[2] = z;
	}
	void updateLinAccel(float (&v)[3]){
		linAccel[0] = v[0];
		linAccel[1] = v[1];
		linAccel[2] = v[2];
	}
	void getLinAccel(float&  x, float&  y, float&  z){
		x = linAccel[0];
		y = linAccel[1];
		z = linAccel[2];
	}
	void getLinAccel(float (&v)[3]){
		v[0] = linAccel[0];
		v[1] = linAccel[1];
		v[2] = linAccel[2];
	}
	//Gauss
	void updateMagField(float  x, float  y, float  z){
		magField[0] = x;
		magField[1] = y;
		magField[2] = z;
	}
	void updateMagField(float (&v)[3]){
		magField[0] = v[0];
		magField[1] = v[1];
		magField[2] = v[2];
	}
	void getMagField(float&  x, float&  y, float&  z){
		x = magField[0];
		y = magField[1];
		z = magField[2];
	}
	void getMagField(float (&v)[3]){
		v[0] = magField[0];
		v[1] = magField[1];
		v[2] = magField[2];
	}
	//Pascals
	void updatePressure(float  p){
		pressure[0] = p;
	}
	void getPressure(float&  p){
		p = pressure[0];
	}
};
#endif
