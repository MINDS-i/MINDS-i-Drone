#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "util/PIDparameters.h"

class PIDcontroller{
private:
	PIDparameters	param;
	float 			setPoint;
	uint32_t		time;
	float			lastError;
	boolean			stopped;
public:
	PIDcontroller(){}
	PIDcontroller(PIDparameters pid): param(pid) {}
	void tune(PIDparameters pid){ param = pid; }
	void clearAccumulator(){ param.resetAccumulator(); }
	void set(float input){
		setPoint = input;
		stopped = false;
	}
	void stop() {
		setPoint = 0;
		param.acc = 0;
		stopped = true;
	}
	float calc(float current){
		if(stopped) {
			time = millis();
			lastError = 0;
			return 0;
		}

		float error = setPoint-current;
		float dt = (millis()-time)/1024.f;//scale dt to bring closer to seconds
		if(dt >= 0.5f) dt = 0.5f;
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
