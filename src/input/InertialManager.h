#ifndef INERTIAL_MANAGER_H
#define INERTIAL_MANAGER_H

#include "math/SpatialMath.h"
#include "math/quaternion.h"
#include "math/vector.h"
#include "input/InertialSensor.h"

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
	InertialSensor** sensor;
	uint8_t numSensors;
	float rotRates[4]; //X,Y,Z, MSE (Mean Square Error)
	float linAccel[4];
	float magField[4];
	float pressure[2];
public:
	InertialManager(InertialSensor** s, uint8_t num)
		: sensor(s), numSensors(num) {}

	void start();
	void calibrate();
	void update(OrientationEngine &orientation);
	void stop();
	void print(HardwareSerial* output);

	void updateRotRates(float dx, float dy, float dz, float MSE);//rad per sec
	void updateLinAccel(float  x, float  y, float  z, float MSE);//G's
	void updateMagField(float  x, float  y, float  z, float MSE);//Gauss
	void updatePressure(float  p, float MSE);					 //Pascals
};
void
InertialManager::start(){
	for(int i=0; i<numSensors; i++) sensor[i]->init();
}
void
InertialManager::calibrate(){
	for(int i=0; i<numSensors; i++) sensor[i]->calibrate();
}
void
InertialManager::update(OrientationEngine &orientation){
	for(int i=0; i<numSensors; i++){
		sensor[i]->update(*this);
	}

	math::vector3d gyro = math::vector3d( -rotRates[1],
										  -rotRates[0],
										   rotRates[2]);
	//make accelerometer quaternion
	float mag = invSqrt(linAccel[0]*linAccel[0] +
						linAccel[1]*linAccel[1] +
						linAccel[2]*linAccel[2]   );
	float x = ((float)linAccel[0]) * mag;
	float y = ((float)linAccel[1]) * mag;

	math::quaternion accl = fromEuler(
				(fabs(x)>1.) ? copysign(M_PI_2,  x) : asin( x),
				(fabs(y)>1.) ? copysign(M_PI_2, -y) : asin(-y),
															  0);
				//atan2(magField[0],magField[1]));//orientation.getYaw() );

	orientation.update(gyro, accl, rotRates[3], linAccel[3], true);
}
void
InertialManager::print(HardwareSerial* output){
	for(int i=0; i<3; i++){
		output->print(rotRates[i]);
		output->print('\t');
	}
	for(int i=0; i<3; i++){
		output->print(linAccel[i]);
		output->print('\t');
	}
	for(int i=0; i<3; i++){
		output->print(magField[i]);
		output->print('\t');
	}
	output->print(pressure[0]);
	output->print('\n');
}
void
InertialManager::stop(){
	for(int i=0; i<numSensors; i++) sensor[i]->stop();
}
void
InertialManager::updateRotRates(float dx, float dy, float dz, float MSE){
	rotRates[0] = dx;
	rotRates[1] = dy;
	rotRates[2] = dz;
	rotRates[3] = MSE;
}
void
InertialManager::updateLinAccel(float  x, float  y, float  z, float MSE){
	linAccel[0] = x;
	linAccel[1] = y;
	linAccel[2] = z;
	linAccel[3] = MSE;
}
void
InertialManager::updateMagField(float  x, float  y, float  z, float MSE){
	magField[0] = x;
	magField[1] = y;
	magField[2] = z;
	magField[3] = MSE;
}
void
InertialManager::updatePressure(float p, float MSE){
	pressure[0] = p;
	pressure[1] = MSE;
}
#endif
