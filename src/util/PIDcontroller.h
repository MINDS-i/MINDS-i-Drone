#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "util/PIDparameters.h"

class PIDcontroller{
private:
	PIDparameters&	param;
	float 			setPoint;
	float			lastError;
	boolean			stopped;
	uint32_t		time;
public:
	PIDcontroller(PIDparameters& pid): param(pid) {}
	void tune(PIDparameters& pid){ param = pid; }
	void clearAccumulator(){ param.resetAccumulator(); }
	void set(float input){
		setPoint = input;
		stopped = false;
	}
	void stop() {
		param.acc = 0;
		stopped = true;
	}
	float calc(float current){//deprecated
		return update(current);
	}
	float update(float current){
		if(stopped) {
			time = millis();
			lastError = 0;
			return 0;
		}

		float dt = millis()-time;
		dt /= 1024.f;//scale dt to bring closer to seconds
		if(dt >= 1.0f) dt = 1.0f;

		float error = setPoint-current;
		param.acc += ((error+lastError)*dt)/2.f; //integrate with midpoint rule

		float output = 0;
		output += param.P * error;
		output += param.I * param.acc;
		output += param.D * (error-lastError)/dt;

		time = millis();
		lastError = error;
		return output;
	}
};

#endif
