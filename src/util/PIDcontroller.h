#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "Arduino.h"
#include "util/PIDparameters.h"

namespace{
	const float SECOND = 1024.0f * 1024.0f;
}

class PIDcontroller{
private:
	PIDparameters*	param;
	float 			setPoint;
	float			previous;
	boolean			stopped;
	uint32_t		time;
public:
	PIDcontroller(PIDparameters* pid): param(pid) {}
	void tune(PIDparameters* pid){ param = pid; }
	void clearAccumulator(){ param->resetAccumulator(); }
	void set(float input){
		setPoint = input;
		stopped = false;
	}
	void stop() {
		clearAccumulator();
		stopped = true;
	}
	float calc(float current){//deprecated
		return update(current);
	}
	float update(float current){
		if(stopped) {
			time = micros();
			previous = 0;
			return 0;
		}

		const float dt = ((float)min( micros()-time, SECOND))/SECOND;
		time = micros();

		const float error  = setPoint-current;
		const float newAcc = param->acc + error*dt;

		const float output = param->P * error
					       + param->I * newAcc
					       + param->D * (previous-current)/dt;

		previous = current;

		if (output >= param->upperBound) {
			return param->upperBound;
		} else if (output <= param->lowerBound) {
			return param->lowerBound;
		}
		//to prevent integral windup, we only change the integral if the output
		//is not fully saturated
		param->acc = newAcc;

		return output;
	}
};

#endif
