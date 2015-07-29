#ifndef LPS25H_H
#define LPS25H_H

#include "Wire.h"

//LPS25H Barometer
class LPS25H : public STMtwiDev {
private:
	//Read/Write
	static const uint8_t CTRL_REG1               = 0x20;
	static const uint8_t CTRL_REG2               = 0x21;
	static const uint8_t CTRL_REG3               = 0x22;
	static const uint8_t CTRL_REG4               = 0x23;
	static const uint8_t FIFO_CTRL               = 0x2E;
	static const uint8_t FIFO_STATUS             = 0x2F;
	static const uint8_t INT_CFG  		         = 0x24;
	static const uint8_t INT_SOURC               = 0x25;
	static const uint8_t RES_CONF                = 0x10;
	static const uint8_t RPDS_H                  = 0x3A;
	static const uint8_t RPDS_L                  = 0x39;
	//Read only
	static const uint8_t PRESS_OUT_H             = 0x2A;
	static const uint8_t PRESS_OUT_L             = 0x29;
	static const uint8_t PRESS_OUT_XL            = 0x28;
	static const uint8_t REF_P_H                 = 0x0A;
	static const uint8_t REF_P_L                 = 0x09;
	static const uint8_t REF_P_XL                = 0x08;
	static const uint8_t STATUS_REG              = 0x27;
	static const uint8_t TEMP_OUT_H              = 0x2C;
	static const uint8_t TEMP_OUT_L              = 0x2B;
	static const uint8_t WHO_AM_I                = 0x0F;
public:
	LPS25H(): STMtwiDev(0x5D, false) {}
	~LPS25H() { end(); }
	void begin();
	void end();
	Sensor::Status status();
	void calibrate();
	void update(InertialManager& man);
	int32_t getRawPressure();
};
void
LPS25H::begin(){
	this->write(RES_CONF,  0x0F);
	this->write(CTRL_REG1, 0xB8);
	calibrate();
}
void
LPS25H::end(){
	this->write(CTRL_REG1, 0);
}
Sensor::Status
LPS25H::status(){
	return (this->read(STATUS_REG)!=0)? Sensor::OK : Sensor::BAD;
}
void
LPS25H::calibrate(){
	this->write(CTRL_REG2, 0x02);
}
void
LPS25H::update(InertialManager& man){
	int32_t pressure = getRawPressure();
	float pascals = (100.f/4096.f)*((float)pressure);
	man.updatePressure(pascals);
}
int32_t
LPS25H::getRawPressure(){
	uint8_t data[3];
	this->batchRead(PRESS_OUT_XL, 3, data);
	int32_t out = 	(((uint32_t) data[0]) << 8 ) |
		  			(((uint32_t) data[1]) << 16) |
		  			(((uint32_t) data[2]) << 24) ;
	out = out >> 8; //sign bit extension
	return out;
}

#endif
