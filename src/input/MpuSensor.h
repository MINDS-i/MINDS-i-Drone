#ifndef MPUSENSOR_H
#define MPUSENSOR_H

class MpuSensor : public InertialSensor{
protected:
	static const uint16_t CAL_SAMPLE_SIZE = 200;
							//+- 2000 dps per least sig bit, in ms
	static const float dPlsb = 2.f*(2.f/65535.f); 
	static const float GYRO_CONVERSION_FACTOR =  2.f*(2.f/65535.f) *PI/180.l;
	LTATune LTA;
	float gCal[3];
public:
	MpuSensor(){}
	void tuneAccl(LTATune t);
	void init();
	void stop();
	bool status();
	void calibrate();
	void update(InertialManager& man);
};
void
MpuSensor::tuneAccl(LTATune t){
	LTA = t;
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
	float av[3];
	for(int i=0; i<3; i++) av[i] = 0;
	for(int i=0; i<CAL_SAMPLE_SIZE; i++){
		av[0] += (float) MPU_Gx();
		av[1] += (float) MPU_Gy();
		av[2] += (float) MPU_Gz();
		delay(1000/CAL_SAMPLE_SIZE);
	}
	for(int i=0; i<3; i++) gCal[i] = -(av[i]/CAL_SAMPLE_SIZE);
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
		rate[i] = gyro[i] + gCal[i];
		rate[i]*= GYRO_CONVERSION_FACTOR;
		//apply static rate shifts from calibration
	}

	//get accelerometer data
	accl[0] = MPU_Ax();
	accl[1] = MPU_Ay();
	accl[2] = MPU_Az();
	for(int i=0; i<3; i++){
		accl[i] += LTA.values.shift[i];
		accl[i] *= LTA.values.scalar[i];
	}

	man.updateRotRates(rate[0], rate[1], rate[2]);
	man.updateLinAccel(accl[0], accl[1], accl[2]);
}
#endif
