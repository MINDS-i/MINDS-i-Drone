#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "Arduino.h"
#include "util/PIDparameters.h"

namespace{
	constexpr float SECOND = 1e6f;
}

class PIDcontroller{
private:
	PIDparameters*	param;
	float 			setPoint;
	float			previous;
	float           acc;
	boolean			stopped;
	uint32_t		time;
public:
	PIDcontroller(PIDparameters* pid): param(pid), setPoint(0),
									   previous(0), acc(0),
									   stopped(true), time(0) {}
	void tune(PIDparameters* pid){ param = pid; }
	void clearAccumulator(){ train(0); }
	void train(float out){
		acc = constrain(out, param->lowerBound, param->upperBound);
	}
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
		uint32_t cTime = micros();
		if(stopped) {
			time = cTime;
			previous = 0;
			return 0;
		}

		const float dt = ((float)min(cTime-time, uint32_t(SECOND)))/SECOND;
		time = cTime;

		const float error  = setPoint-current;
		const float newAcc = acc + param->I*error*dt;

		const float output = param->P * error
					       + newAcc
					       + param->D * (previous-current)/dt;

		previous = current;

		if (output > param->upperBound) {
			acc = min(acc, newAcc); //only let acc decrease
			return param->upperBound;
		} else if (output < param->lowerBound) {
			acc = max(acc, newAcc); //only let acc increase
			return param->lowerBound;
		}
		//to prevent integral windup, we only change the integral if the output
		//is not fully saturated
		acc = newAcc;

		return output;
	}
};

#endif
