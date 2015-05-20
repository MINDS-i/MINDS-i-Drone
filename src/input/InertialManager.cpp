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
	for(int i=0; i<numSensors; i++) sensor[i]->update(*this);
}
void
InertialManager::update(OrientationEngine &orientation){
	update();
	orientation.update(*this);
}
void
InertialManager::stop(){
	for(int i=0; i<numSensors; i++) sensor[i]->stop();
}
