#ifndef LMS303D_H
#define LMS303D_H

#include "Wire.h"

//LSM303D Accelerometer and Magnometer
class LSM303D : public STMtwiDev {
private:
	static const float ACC_CONVERSION_FACTOR;
	static const float MAG_CONVERSION_FACTOR;
	//Read/Write
	static const uint8_t Act_DUR 			= 0x3F;
	static const uint8_t Act_THS 			= 0x3E;
	static const uint8_t CLICK_CFG 			= 0x38;
	static const uint8_t CLICK_THS 			= 0x3A;
	static const uint8_t CTRL0 				= 0x1F;
	static const uint8_t CTRL1 				= 0x20;
	static const uint8_t CTRL2 				= 0x21;
	static const uint8_t CTRL3 				= 0x22;
	static const uint8_t CTRL4 				= 0x23;
	static const uint8_t CTRL5 				= 0x24;
	static const uint8_t CTRL6 				= 0x25;
	static const uint8_t CTRL7 				= 0x26;
	static const uint8_t FIFO_CTRL 			= 0x2E;
	static const uint8_t IG_CFG1 			= 0x30;
	static const uint8_t IG_CFG2 			= 0x34;
	static const uint8_t IG_DUR1 			= 0x33;
	static const uint8_t IG_DUR2 			= 0x37;
	static const uint8_t IG_THS1 			= 0x32;
	static const uint8_t IG_THS2 			= 0x36;
	static const uint8_t INT_CTRL_M 		= 0x12;
	static const uint8_t INT_THS_H_M 		= 0x15;
	static const uint8_t INT_THS_L_M 		= 0x14;
	static const uint8_t OFFSET_X_H_M 		= 0x17;
	static const uint8_t OFFSET_X_L_M 		= 0x16;
	static const uint8_t OFFSET_Y_H_M 		= 0x19;
	static const uint8_t OFFSET_Y_L_M 		= 0x18;
	static const uint8_t OFFSET_Z_H_M 		= 0x1B;
	static const uint8_t OFFSET_Z_L_M 		= 0x1A;
	static const uint8_t REFERENCE_X 		= 0x1C;
	static const uint8_t REFERENCE_Y 		= 0x1D;
	static const uint8_t REFERENCE_Z 		= 0x1E;
	static const uint8_t TIME_LATENCY 		= 0x3C;
	static const uint8_t TIME_LIMIT 		= 0x3B;
	static const uint8_t TIME_WINDOW 		= 0x3D;
	//Read only
	static const uint8_t CLICK_SRC 			= 0x39;
	static const uint8_t FIFO_SRC 			= 0x2F;
	static const uint8_t IG_SRC1 			= 0x31;
	static const uint8_t IG_SRC2 			= 0x35;
	static const uint8_t INT_SRC_M 			= 0x13;
	static const uint8_t OUT_X_H_A 			= 0x29;
	static const uint8_t OUT_X_H_M 			= 0x09;
	static const uint8_t OUT_X_L_A 			= 0x28;
	static const uint8_t OUT_X_L_M 			= 0x08;
	static const uint8_t OUT_Y_H_A 			= 0x2B;
	static const uint8_t OUT_Y_H_M 			= 0x0B;
	static const uint8_t OUT_Y_L_A 			= 0x2A;
	static const uint8_t OUT_Y_L_M 			= 0x0A;
	static const uint8_t OUT_Z_H_A 			= 0x2D;
	static const uint8_t OUT_Z_H_M 			= 0x0D;
	static const uint8_t OUT_Z_L_A 			= 0x2C;
	static const uint8_t OUT_Z_L_M 			= 0x0C;
	static const uint8_t STATUS_A 			= 0x27;
	static const uint8_t STATUS_M 			= 0x07;
	static const uint8_t TEMP_OUT_H 		= 0x06;
	static const uint8_t TEMP_OUT_L 		= 0x05;
	static const uint8_t WHO_AM_I 			= 0x0F;
public:
	LSM303D(): STMtwiDev(0x1D, true) {}
	~LSM303D() { end(); }
	void begin();
	void end();
	Sensor::Status status();
	void calibrate();
	void update(InertialManager& man);
	void getRawAccl(int16_t* buf);
	void getRawMag(int16_t* buf);
};
const float LSM303D::ACC_CONVERSION_FACTOR = (2.f/32767.f);
const float LSM303D::MAG_CONVERSION_FACTOR = (4.f/32767.f);
void
LSM303D::begin(){
	this->write(CTRL1, 0x87); //all accl axis on, 400Hz
	this->write(CTRL2, 0x00); //773Hz alti-alias; +-2g scale
	this->write(CTRL5, 0x74); //temp off; mag@100Hz; high res
	this->write(CTRL6, 0x20); //mag resolution +-4g
	this->write(CTRL7, 0x40); //accl high pass
}
void
LSM303D::end(){

}
Sensor::Status
LSM303D::status(){
	return Sensor::OK;
}
void
LSM303D::calibrate(){
}
void
LSM303D::update(InertialManager& man){
	int16_t rA[3];
	int16_t rM[3];
	getRawAccl(rA);
	getRawMag (rM);

	float accl[3];
	float  mag[3];
	for(int i=0; i<3; i++){
		accl[i] = ((float)rA[i])*ACC_CONVERSION_FACTOR;
		 mag[i] = ((float)rM[i])*MAG_CONVERSION_FACTOR;
	}

	man.updateLinAccel(accl[0], accl[1], accl[2]);
	man.updateMagField( mag[0],  mag[1],  mag[2]);
}
void
LSM303D::getRawAccl(int16_t* buf){
	uint8_t data[6];
	this->batchRead(OUT_X_L_A, 6, data);
	for(int i=0; i<3; i++){
		buf[i] = ((uint16_t)data[0+2*i]) | ((uint16_t)data[1+2*i])<<8;
	}
}
void
LSM303D::getRawMag(int16_t* buf){
	uint8_t data[6];
	this->batchRead(OUT_X_L_M, 6, data);
	for(int i=0; i<3; i++){
		buf[i] = ((uint16_t)data[0+2*i]) | ((uint16_t)data[1+2*i])<<8;
	}
}
#endif
