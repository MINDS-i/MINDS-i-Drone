#ifndef APMCOMPASS_H
#define APMCOMPASS_H

#include "APM/Compass.h"
#include "input/InertialManager.h"
#include "input/InertialSensor.h"
#include "util/LTATune.h"

class APMCompass : public InertialSensor{
protected:
	LTATune	LTA;
public:
	APMCompass(){}
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
	void tune(LTATune t);
};
void
APMCompass::tune(LTATune t){
	LTA = t;
}
void APMCompass::init(){
	beginCompass();
}
void 
APMCompass::stop(){
	
}
bool 
APMCompass::status(){
	return true;
}
void
APMCompass::calibrate(){
	
}
void 
APMCompass::update(InertialManager& man){
	int m[3];
	rawCompass(&m[0], &m[1], &m[2]);
	float M[3];
	for(int i=0; i<3; i++){
		M[i]  = m[i];
		M[i] += LTA.values.shift[i];
		M[i] *= LTA.values.scalar[i];
	}
	man.updateMagField(M[0],M[1],M[2]);
}

#endif
