#include "InertialManager.h"
void
InertialManager::start(){
	for(int i=0; i<numSensors; i++) sensor[i]->init();
}
void
InertialManager::calibrate(){
	for(int i=0; i<numSensors; i++) sensor[i]->calibrate();
}
void
InertialManager::update(){
	for(int i=0; i<numSensors; i++){
		sensor[i]->update(*this);
	}
}
void
InertialManager::stop(){
	for(int i=0; i<numSensors; i++) sensor[i]->stop();
}
void
InertialManager::update(OrientationEngine &orientation){
	update();
	orientation.update(this);
}
void
InertialManager::updateRotRates(float dx, float dy, float dz){
	rotRates[0] = dx;
	rotRates[1] = dy;
	rotRates[2] = dz;
}
void
InertialManager::updateLinAccel(float  x, float  y, float  z){
	linAccel[0] = x;
	linAccel[1] = y;
	linAccel[2] = z;
}
void
InertialManager::updateMagField(float  x, float  y, float  z){
	magField[0] = x;
	magField[1] = y;
	magField[2] = z;
}
void
InertialManager::updatePressure(float p){
	pressure[0] = p;
}
void
InertialManager::getRotRates(float& dx, float& dy, float& dz){
	dx  = rotRates[0];
	dy  = rotRates[1];
	dz  = rotRates[2];
}
void
InertialManager::getLinAccel(float&  x, float&  y, float&  z){
	x   = linAccel[0];
	y   = linAccel[1];
	z   = linAccel[2];
}
void
InertialManager::getMagField(float&  x, float&  y, float&  z){
	x   = magField[0];
	y   = magField[1];
	z   = magField[2];
}
void
InertialManager::getPressure(float& p){
	p   = pressure[0];
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
