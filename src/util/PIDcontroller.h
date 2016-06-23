#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "Arduino.h"
#include "util/PIDparameters.h"
#include "util/PIDexternaltime.h"

/**
 * A wrapper class for PIDexternaltime that automatically calculates the time
 * step using `micros()`
 */
class PIDcontroller{
private:
	PIDexternaltime pid;
	uint32_t		lastUpdateMicros;
public:
	PIDcontroller(PIDparameters* pid): pid(pid) {}
	void tune(PIDparameters* params){ pid.tune(params); }
	void clearAccumulator(){ pid.clearAccumulator(); }
	void train(float out){ pid.train(out); }
	void set(float input){ pid.set(input); }
	void stop() { pid.stop(); }
	float update(float current){
		uint32_t time = micros();

		// calculate microsecond delta and convert milliseconds
		const float ms = ((float)time-lastUpdateMicros)/1000.0;

		lastUpdateMicros = time;
		return pid.update(current, ms);
	}
};

#endif
