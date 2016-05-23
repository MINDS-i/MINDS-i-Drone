#ifndef AFROESC_OUTPUT_DEV_H
#define AFROESC_OUTPUT_DEV_H

#include "output/OutputDevice.h"
//#include "APM/ServoGenerator.h"
#include "APM/ServoPWM.h"
#include "math/Algebra.h"
//wrapper for arduino Servo Library to OutputDevice type
class AfroESC: public OutputDevice{
private:
	constexpr static uint16_t STOP = 1060;
	constexpr static uint16_t IDLE = 1135;
	constexpr static uint16_t FULL = 1860;
	constexpr static uint16_t RANGE = FULL-STOP;
	uint8_t	pin;
public:
	AfroESC(uint8_t in): pin(in) {}
	~AfroESC(){ stop(); }
	void  startArming()	{
		ServoPWM::setup(6000);
	}
	boolean continueArming(uint32_t dt){
		if(dt<3500){
			ServoPWM::set(pin, STOP);
			return false;
		}
		return true;
	}
	void startCalibrate(){
		ServoPWM::setup(6000);
	}
	boolean continueCalibrate(uint32_t dt){
		if(dt<5000) {
			ServoPWM::set(pin,FULL);
			return false;
		}
		if(dt<10000) {
			ServoPWM::set(pin,STOP);
			return false;
		}
		return true;
	}
	void set(float in)	{
		if (in>=0.0f) {
			ServoPWM::set(pin,max(in*RANGE+STOP, IDLE));
		} else {
			ServoPWM::set(pin,STOP);
		}
	}
	void  stop() { }
	float get()	{ return 0;}//((float)servo.readMicroseconds()-MIN)/RANGE; }
	uint16_t getRaw() { return 0;}//servo.readMicroseconds(); }
};

#endif
