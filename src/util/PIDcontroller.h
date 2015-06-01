#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "util/PIDparameters.h"

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
			time = millis();
			previous = 0;
			return 0;
		}

		const float dt = ((float)min( millis()-time, 1024 ))/1024.f;
		time = millis();

		const float error = setPoint-current;
		//integrate with midpoint rule
		const float newAcc = param->acc + error*dt;//((error+lastError)*dt)/2.f;

		const float output = param->P * error
					       + param->I * newAcc
					       + param->D * (previous-current)/dt;

		previous = current;

		if (output >= param->upperBound) {
			return param->upperBound;
		} else if (output <= param->lowerBound) {
			return param->lowerBound;
		} else {
			//to prevent integral windup, we only integrate if the output
				//is not fully saturated
			param->acc = newAcc;
		}

		return output;
	}
};

#endif
