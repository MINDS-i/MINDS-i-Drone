#ifndef L2GD20H_H
#define L2GD20H_H

#include "Wire.h"

#include "input/altIMU/STMtwi.h"
#include "input/InertialManager.h"

//L3GD20H Gyroscope
class L3GD20H : public STMtwiDev {
private:
	//settings
	static const uint16_t CAL_SAMPLE_SIZE = 100;
	static const float OUTPUT_CONVERSION_FACTOR = .07f*.001*PI/180.f;//(2000.f/32767.f)*PI/180.f;
	//Read/Write
	static const uint8_t CTRL1 			= 0x20;
	static const uint8_t CTRL2 			= 0x21;
	static const uint8_t CTRL3 			= 0x22;
	static const uint8_t CTRL4 			= 0x23;
	static const uint8_t CTRL5 			= 0x24;
	static const uint8_t FIFO_CTRL 		= 0x2E;
	static const uint8_t IG_CFG 		= 0x30;
	static const uint8_t IG_DURATION 	= 0x38;
	static const uint8_t IG_THS_XH 		= 0x32;
	static const uint8_t IG_THS_XL 		= 0x33;
	static const uint8_t IG_THS_YH 		= 0x34;
	static const uint8_t IG_THS_YL 		= 0x35;
	static const uint8_t IG_THS_ZH 		= 0x36;
	static const uint8_t IG_THS_ZL 		= 0x37;
	static const uint8_t LOW_ODR 		= 0x39;
	static const uint8_t REFERENCE 		= 0x25;
	//Read Only
	static const uint8_t FIFO_SRC 		= 0x2F;
	static const uint8_t IG_SRC 		= 0x31;
	static const uint8_t OUT_TEMP 		= 0x26;
	static const uint8_t OUT_X_H 		= 0x29;
	static const uint8_t OUT_X_L 		= 0x28;
	static const uint8_t OUT_Y_H 		= 0x2B;
	static const uint8_t OUT_Y_L 		= 0x2A;
	static const uint8_t OUT_Z_H 		= 0x2D;
	static const uint8_t OUT_Z_L 		= 0x2C;
	static const uint8_t STATUS 		= 0x27;

	float LPfac;
	float lowPass[3];
public:
	L3GD20H()
		: STMtwiDev(0x6B, true), LPfac(.9999)  {}
	L3GD20H(float LP)
		: STMtwiDev(0x6B, true), LPfac(LP)     {}
	~L3GD20H() { stop(); }
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
	void getRawGyro(int16_t* buf);
};
void
L3GD20H::init(){
	this->write(CTRL1, 0x8F); //on at 200Hz
	this->write(CTRL2, 0x15); //high pass filter
	this->write(CTRL4, 0x30); //2000dps range
	this->write(CTRL5, 0x10); //enable HPF
}
void
L3GD20H::stop(){
	this->write(CTRL1, 0);
}
bool
L3GD20H::status(){
	return true;
}
void
L3GD20H::calibrate(){
	int16_t data[3];
	float average[3];
	for(int i=0; i<3; i++) average[i] = 0;
	for(int i=0; i<CAL_SAMPLE_SIZE; i++){
		getRawGyro(data);
		for(int j=0; j<3; j++) {
			average[j] += ((float)data[j]) / ((float)CAL_SAMPLE_SIZE);
		}
		delay(1000/CAL_SAMPLE_SIZE);
	}
	for(int i=0; i<3; i++) lowPass[i] = average[i];
}
void
L3GD20H::update(InertialManager& man){
	float rate[3];
	int16_t data[3];
	getRawGyro(data);
	for(int i=0; i<3; i++){
		lowPass[i] = lowPass[i]*LPfac + ((float)data[i])*(1.f-LPfac);
		rate[i]    = ((float)data[i])-lowPass[i];
		rate[i]   *= OUTPUT_CONVERSION_FACTOR;//convert to rps
	}
	man.updateRotRates(rate[0], rate[1], rate[2]);
}
void
L3GD20H::getRawGyro(int16_t* buf){
	uint8_t data[6];
	this->batchRead(OUT_X_L, 6, data);
	for(int i=0; i<3; i++){
		buf[i] = ((uint16_t)data[0+2*i]) | ((uint16_t)data[1+2*i])<<8;
	}
}
#endif
