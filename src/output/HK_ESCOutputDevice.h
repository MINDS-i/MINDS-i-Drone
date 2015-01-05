#ifndef HKESC_OUTPUT_DEV_H
#define HKESC_OUTPUT_DEV_H

#include "Servo.h"
#include "output/OutputDevice.h"
//wrapper for arduino Servo Library to OutputDevice type
class HK_ESCOutputDevice: public OutputDevice{
private:
	const static uint8_t RANGE = 162;
	const static uint8_t MIN   =  18;
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
		if(dt<3000){
			servo.write(MIN);
			return false;
		}
		return true;
	}
	void startCalibrate(){
		Servo::setRefreshInterval(5000);
		servo.attach(pin);
	}
	boolean continueCalibrate(uint32_t dt){
		if(dt<2000) {
			servo.write(MIN+RANGE);
			return false;
		}
		if(dt<8000) {
			servo.write(MIN);
			return false;
		}
		return true;
	}
	void  set(float in)	{ if(in>=-.001) servo.write(in*RANGE+MIN); 	}
	void  stop()		{ servo.detach();   						}
	float get()			{ return ((float)servo.read()-MIN)/RANGE; 	}
};
#endif
