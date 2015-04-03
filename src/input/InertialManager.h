#ifndef INERTIAL_MANAGER_H
#define INERTIAL_MANAGER_H
#include "filter/OrientationEngine.h"
#include "input/Sensor.h"
#include "Arduino.h"
/*
-Inertial Manager is given an array of Inertial Sensors on intialization.
-It will propogate commands to all the sensors it owns
-when an Inertial Sensor is updated, it will pass the data back up to the
	Inertial Manager, which will normalize the data and feed to an
	OrientationEngine.
-This way sensor math is completly separate from sensor hardware code,
	and OrientationEngines don't have to worry about where their data comes from
	making swaps of OrientationEngines easier
-The math in InertialManager recieves unit normalized sensor output, and
	transforms it to standardized state data that describes what all the sesors
	are currently 'seeing' for the OrientatienEngine to fuse.
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
	void print(HardwareSerial* output);

	void updateRotRates(float dx, float dy, float dz);//rad per millisecond
	void updateLinAccel(float  x, float  y, float  z);//G's
	void updateMagField(float  x, float  y, float  z);//Gauss
	void updatePressure(float  p);					  //Pascals

	void getRotRates(float& dx, float& dy, float& dz);//rad per millisecond
	void getLinAccel(float&  x, float&  y, float&  z);//G's
	void getMagField(float&  x, float&  y, float&  z);//Gauss
	void getPressure(float&  p);					  //Pascals
};
#endif
