#ifndef HKESC_OUTPUT_DEV_H
#define HKESC_OUTPUT_DEV_H

#include "Servo.h"
#include "output/OutputDevice.h"
//wrapper for arduino Servo Library to OutputDevice type
class HK_ESCOutputDevice: public OutputDevice{
private:
	const static int RANGE = 162;
	const static int MIN   =  18;
	Servo 	servo;
	int		Pin;
public:
	HK_ESCOutputDevice(int pin): Pin(pin) {}
	~HK_ESCOutputDevice(){}
	void  startArming()	{
		Servo::setRefreshInterval(5000);
		servo.attach(Pin);
	}
	boolean continueArming(uint32_t dt){
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
