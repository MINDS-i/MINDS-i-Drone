#ifndef APMCOMPASS_H
#define APMCOMPASS_H

#include "APM/Compass.h"
#include "input/InertialManager.h"
#include "input/Sensor.h"
#include "util/LTATune.h"

class APMCompass : public Sensor{
protected:
	LTATune	LTA;
public:
	APMCompass(){}
	void  init();
	void  stop();
	bool  status();
	void  calibrate();
	void  update(InertialManager& man);
	void  tune(LTATune t);
	float getAzimuth();
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
	return STATUS_OK;
}
void
APMCompass::calibrate(){

}
void
APMCompass::update(InertialManager& man){
	int m[3];
	rawCompass(&m[0], &m[1], &m[2]);
	float M[3];
	LTA.calibrate<int>(m, M);
	man.updateMagField(M[0],M[1],M[2]);
}
float
APMCompass::getAzimuth(){
	int m[3];
	rawCompass(&m[0], &m[1], &m[2]);
	float M[3];
	for(int i=0; i<3; i++){
		M[i]  = m[i];
		M[i] += LTA.shift[i];
		M[i] *= LTA.scalar[i];
	}
	return atan2(M[0], M[1]);
}

#endif
