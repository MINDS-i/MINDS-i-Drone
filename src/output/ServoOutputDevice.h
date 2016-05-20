#ifndef SERVO_OUTPUT_DEV_H
#define SERVO_OUTPUT_DEV_H

#include "output/OutputDevice.h"
#include "APM/ServoGenerator.h"
//wrapper for arduino Servo Library to OutputDevice type
class ServoOutput: public OutputDevice{
private:
	ServoGenerator::Servo servo;
	int pin;
public:
	ServoOutput(int in): pin(in) {}
	~ServoOutput(){}
	void  	startArming()	{ servo.attach(pin); 			}
	boolean continueArming(uint32_t dt)		{ return true;	}
	void  	startCalibrate(){ servo.attach(pin); 			}
	boolean continueCalibrate(uint32_t dt)	{ return true;	}
	void  set(float in)	{
		servo.writeMicroseconds(in*1000.+1000);
	}
	void  stop() {
		servo.detach();
	}
	float get() {
		return 0;//((float)servo.readMicroseconds()-1000.)/1000.;
	}
};
#endif
