#ifndef MPUSENSOR_H
#define MPUSENSOR_H

class MpuSensor : public InertialSensor{
protected:
	static const uint16_t CAL_SAMPLE_SIZE = 100;
	static const double dPlsb = 4.f/65535.f;
	static const double GYRO_CONVERSION_FACTOR =  (4.f/65535.f)*PI/180.l;
	float acclShifts[3] = { 0, 0, 0};
	float acclScalar[3] = { 1, 1, 1};
	float gyroMSE, acclMSE;
public:
	MpuSensor()
		: gyroMSE(1.),  acclMSE(100.) {}
	MpuSensor(float gMSE, float aMSE)
		: gyroMSE(gMSE), acclMSE(aMSE)  {}
	void tuneAccl(float xs, float ys, float zs,
				  float xb, float yx, float zb);
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
	void setGyroMSE(float mse) { gyroMSE = mse; }
	void setAcclMSE(float mse) { acclMSE = mse; }
	float getGyroMSE() { return gyroMSE; }
	float getAcclMSE() { return acclMSE; }
};
void
MpuSensor::tuneAccl(float xs, float ys, float zs,
					float xb, float yb, float zb){
	acclShifts[0] = xs;
	acclShifts[1] = ys;
	acclShifts[2] = zs;

	acclScalar[0] = xb;
	acclScalar[1] = yb;
	acclScalar[2] = zb;
}
void
MpuSensor::init(){
	InitMPU();
}
void
MpuSensor::stop(){

}
bool
MpuSensor::status(){
	return true;
}
void
MpuSensor::calibrate(){
/*	double Gx, Gy, Gz;
	double tmp[3];
	for(int i=0; i<3; i++) tmp[i] = 0;
	for(int i=0; i<CAL_SAMPLE_SIZE; i++){
		Gx = MPU_Gx();
		Gy = MPU_Gy();
		Gz = MPU_Gz();
		tmp[0] += ((float)Gx)/((float)CAL_SAMPLE_SIZE);
		tmp[1] += ((float)Gy)/((float)CAL_SAMPLE_SIZE);
		tmp[2] += ((float)Gz)/((float)CAL_SAMPLE_SIZE);
		delay(1000/CAL_SAMPLE_SIZE);
	}
	for(int i=0; i<3; i++) lowPass[i] = tmp[i];*/
}
void
MpuSensor::update(InertialManager& man){
	float rate[3];
	float accl[3];

	//get gyro data
	float gyro[3];
	gyro[0] = MPU_Gx();
	gyro[1] = MPU_Gy();
	gyro[2] = MPU_Gz();
	for(int i=0; i<3; i++){
		//apply static rate shifts from calibration
	}

	//get accelerometer data
	accl[0] = MPU_Ax();
	accl[1] = MPU_Ay();
	accl[2] = MPU_Az();
	for(int i=0; i<3; i++){
		accl[i] += acclShifts[i];
		accl[i] *= acclScalar[i];
	}

	man.updateRotRates(rate[0], rate[1], rate[2], gyroMSE);
	man.updateLinAccel(accl[0], accl[1], accl[2], acclMSE);
}
#endif
