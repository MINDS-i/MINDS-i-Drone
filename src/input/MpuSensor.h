#ifndef MPUSENSOR_H
#define MPUSENSOR_H

class MpuSensor : public InertialSensor{
protected:
	static const uint16_t CAL_SAMPLE_SIZE = 100;
	static const double dPlsb = 4.f/65535.f;
	static const double GYRO_CONVERSION_FACTOR =  (4.f/65535.f)*PI/180.l;
	float LPfac;
	float lowPass[3];
	float acclShifts[3] = {   0,    0,    0};
	float acclBounds[3] = {8250, 8250, 8250};
	float gyroMSE, acclMSE;
public:
	MpuSensor()
		: LPfac(.9999), gyroMSE(1.),  acclMSE(100.) {}
	MpuSensor(float lp)
		: LPfac(lp),    gyroMSE(1.),  acclMSE(100.) {}
	MpuSensor(float lp, float gMSE, float aMSE)
		: LPfac(lp),    gyroMSE(gMSE), acclMSE(aMSE)  {}
	void tuneAccl(float xs, float ys, float zs,
				  float xb, float yx, float zb);
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
	void setGryoLPF(float lpf) { LPfac = lpf; }
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

	acclBounds[0] = xb;
	acclBounds[1] = yb;
	acclBounds[2] = zb;
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
	double Gx, Gy, Gz;
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
	for(int i=0; i<3; i++) lowPass[i] = tmp[i];
}
void
MpuSensor::update(InertialManager& man){
	float rate[3];
	float accl[3];

	//get gyro data
	double gyro[3];
	gyro[0] = MPU_Gx();
	gyro[1] = MPU_Gy();
	gyro[2] = MPU_Gz();
	for(int i=0; i<3; i++){
		lowPass[i] = lowPass[i]*LPfac + ((float)gyro[i])*(1.f-LPfac);
		rate[i]    = ((float)gyro[i])-lowPass[i];
		rate[i]   *= GYRO_CONVERSION_FACTOR;//convert to rps
	}

	//get accelerometer data
	accl[0] = MPU_Ax();
	accl[1] = MPU_Ay();
	accl[2] = MPU_Az();
	for(int i=0; i<3; i++){
		accl[i] = (accl[i]+acclShifts[i])/acclBounds[i];
	}

	man.updateRotRates(rate[0], rate[1], rate[2], gyroMSE);
	man.updateLinAccel(accl[0], accl[1], accl[2], acclMSE);
}
#endif
