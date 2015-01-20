#ifndef APMCOMPASS_H
#define APMCOMPASS_H

#include "input/InertialManager.h"
#include "input/InertialSensor.h"
#include "APM/Compass.h"

class APMCompass : public InertialSensor{
protected:
	float MSE;
	float shifts[3] = { 0, 0, 0};
	float scalar[3] = { 1, 1, 1};
public:
	APMCompass(): MSE(1) {}
	APMCompass(float mse): MSE(mse) {}
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
	void setMSE(float mse){ MSE = mse; }
	float getMSE() { return MSE; }
	void tune(float xs, float ys, float zs,
			  float xb, float yx, float zb);
};
void
APMCompass::tune(float xs, float ys, float zs,
				 float xb, float yb, float zb){
	shifts[0] = xs;
	shifts[1] = ys;
	shifts[2] = zs;

	scalar[0] = xb;
	scalar[1] = yb;
	scalar[2] = zb;
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
		M[i] += shifts[i];
		M[i] *= scalar[i];
	}
	man.updateMagField(M[0],M[1],M[2], MSE);
}

#endif
