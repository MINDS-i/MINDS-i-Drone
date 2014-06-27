#ifndef HLA_H
#define HLA_H

#include "math.h"
#include "Arduino.h"

//Time controlled exponential running average
//set the half-life in milliseconds

class HLA{
private:
	double value;
	double halfLife;
	uint32_t time;
public:
	HLA(double half, double init): halfLife(half*1000.l), value(init) {
		time=micros();
	}
	double update(double newValue){
		if(isnan(newValue)) return value;
		double dt = double(micros()-time);
		double factor = pow(2.l, -dt/halfLife);
		value = (1.l-factor)*newValue + factor*value;
		time = micros();
		return value;
	}
	double get(){
		return value;
	}
	void set(double newValue){
		if(isnan(newValue)) return;
		value = newValue;
	}
	void resetTime(){
		time = micros();
	}
};

#endif
