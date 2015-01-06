#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "util/PIDparameters.h"

class PIDcontroller{
private:
	PIDparameters	param;
	float 			setPoint;
	uint32_t		time;
	float			lastError;
public:
	PIDcontroller(){}
	PIDcontroller(PIDparameters pid): param(pid) {}
	void set(float input){setPoint = input;}
	void tune(PIDparameters pid){ param = pid; }
	void clearAccumulator(){ param.resetAccumulator(); }
	float calc(float current){
		float error = setPoint-current;
		float dt = (millis()-time)/1024.f;//scale dt to bring closer to seconds
		float output = 0;

		param.acc += ((error+lastError)*dt)/2.f; //integrate with midpoint rule
		output += param.P * error;
		output += param.I * param.acc;
		output += param.D * (error-lastError)/dt;

		time = millis();
		lastError = error;
		return output;
	}
};

#endif
