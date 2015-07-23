#ifndef HKESC_OUTPUT_DEV_H
#define HKESC_OUTPUT_DEV_H

#include "Servo.h"
#include "output/OutputDevice.h"
#include "math/Algebra.h"
//wrapper for arduino Servo Library to OutputDevice type
class HK_ESCOutputDevice: public OutputDevice{
private:
	const static float thrustCurve[4];// = { 0.776, -1.160, 1.382, 0.0 };
	const static uint16_t MIN    = 1000;
	const static uint16_t IDLE   = 1100;
	const static float    RANGE;// 1000
	Servo 	servo;
	uint8_t	pin;
public:
	HK_ESCOutputDevice(uint8_t in): pin(in) {}
	~HK_ESCOutputDevice(){ stop(); }
	void  startArming()	{
		Servo::setRefreshInterval(5000);
		servo.attach(pin);
	}
	boolean continueArming(uint32_t dt){
		if(dt<3500){
			servo.writeMicroseconds(MIN);
			return false;
		}
		servo.writeMicroseconds(IDLE);
		return true;
	}
	void startCalibrate(){
		Servo::setRefreshInterval(5000);
		servo.attach(pin);
	}
	boolean continueCalibrate(uint32_t dt){
		if(dt<3000) {
			servo.writeMicroseconds(MIN+RANGE);
			return false;
		}
		if(dt<10000) {
			servo.writeMicroseconds(MIN);
			return false;
		}
		servo.writeMicroseconds(IDLE);
		return true;
	}
	void  set(float in)	{
		if (in>=0.0f) {
			float throttle = cubicHorner(in, thrustCurve);
			servo.writeMicroseconds(max(throttle*RANGE+MIN,IDLE));
		} else {
			servo.writeMicroseconds(MIN);
		}
	}
	void  stop()		{ servo.detach(); }
	float get()			{ return ((float)servo.readMicroseconds()-MIN)/RANGE; }
	uint16_t getRaw()   { return servo.readMicroseconds(); }
};
const float HK_ESCOutputDevice::RANGE  = 1000;
const float HK_ESCOutputDevice::thrustCurve[4] = { 0.776, -1.160, 1.382, 0.0 };

#endif
