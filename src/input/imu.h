#ifndef IMU_H
#define IMU_H

#include "math/SpatialMath.h"

const double dPlsb = 4.f/float(0xffff);
const double GYRO_CONVERSION_FACTOR = dPlsb*PI/180.l;//*1000;(for Qexp)
const double BAD_ACCL = -17;
//these need to be calculated for each board
// to tune the accelerometer readings
const double ACCL_SHIFTS[3] = {-450, 87.5, -350};
const double ACCL_BOUNDS[3] = {8200, 8213, 8300};
class IMU{
private:
	uint32_t time;
	double HL[2];
	double low[3];
	double high[3];
	math::matrix2d rms;
	math::vector3d accl;
public:
	IMU(double gyroLowHalfLife, double gyroHighHalfLife,
		double gyroRMS, double acclRMS);
	IMU();
	void calibrateGyro();
	math::quaternion getAccl();
	math::quaternion getGyro();
	math::vector3d   getGyroAngles();
    math::vector3d   getAcclMagnitudes();
    void runUpdate(KalmanFilter& filter);
};
IMU::IMU(double gyroLowHalfLife, double gyroHighHalfLife,
		 double gyroRMS, double acclRMS):
		time(0), low(),
		HL{gyroLowHalfLife*1000, gyroHighHalfLife*1000},
		rms(gyroRMS, 0, 0, acclRMS) , accl(0,0,0){
	calibrateGyro();
}
IMU::IMU(){}
void
IMU::calibrateGyro(){
	double Gx, Gy, Gz;
	double tmp[3];
	for(int i=0; i<3; i++) tmp[i] = 0;
	for(int i=0; i<100; i++){
		Gx = MPU_Gx();
		Gy = MPU_Gy();
		Gz = MPU_Gz();
		tmp[0] += ((float)Gx)/100;
		tmp[1] += ((float)Gy)/100;
		tmp[2] += ((float)Gz)/100;
		delay(10);
	}
	for(int i=0; i<3; i++) low[i] = tmp[i];
}
math::quaternion
IMU::getAccl(){
	accl = math::vector3d( (MPU_Ax()+ACCL_SHIFTS[0])/ACCL_BOUNDS[0],
						   (MPU_Ay()+ACCL_SHIFTS[1])/ACCL_BOUNDS[1],
						   (MPU_Az()+ACCL_SHIFTS[2])/ACCL_BOUNDS[2]  );
	math::vector3d v = accl;
	double mag = v.mag();
	if( fabs(1.l-mag) > .011) return math::quaternion(BAD_ACCL, 0, 0, 0);
	v /= mag;
	double theta = acos(v.z);
	math::vector3d q = math::vector3d(-v.y, v.x, 0);
	q.normalize();
	q *= sin(theta/2);
	return math::quaternion(cos(theta/2), q);
}
math::quaternion
IMU::getGyro(){
	double dt = double(micros()-time);
	time = micros();
	double factor[2] = {pow(2.l, -dt/HL[0]), pow(2.l, -dt/HL[1])};
	double gyro[3];
	gyro[0] = MPU_Gx();
	gyro[1] = MPU_Gy();
	gyro[2] = MPU_Gz();
	for(int i=0; i<3; i++){
		low[i]  = (1.l-factor[0])*gyro[i] + (factor[0])*low[i];
		high[i] = (1.l-factor[1])*(gyro[i]-low[i]) + (factor[1])*high[i];
		gyro[i] = high[i]*GYRO_CONVERSION_FACTOR;
	}
	return fromEuler(-gyro[1], -gyro[0], gyro[2]);
}
math::vector3d
IMU::getGyroAngles(){
	double dt = double(micros()-time);
	time = micros();
	double factor[2] = {pow(2.l, -dt/HL[0]), pow(2.l, -dt/HL[1])};
	double gyro[3];
	gyro[0] = MPU_Gx();
	gyro[1] = MPU_Gy();
	gyro[2] = MPU_Gz();
	for(int i=0; i<3; i++){
		low[i]  = (1.l-factor[0])*gyro[i] + (factor[0])*low[i];
		high[i] = (1.l-factor[1])*(gyro[i]-low[i]) + (factor[1])*high[i];
		gyro[i] = high[i]*GYRO_CONVERSION_FACTOR;
	}
	return math::vector3d(-gyro[1], -gyro[0], gyro[2]);
}
math::vector3d
IMU::getAcclMagnitudes(){
	return accl;
}
void
IMU::runUpdate(KalmanFilter& filter){
	math::quaternion accl = getAccl();
	if(accl._s != BAD_ACCL)
		filter.update(getGyroAngles(), accl, rms(0,0), rms(1,1), true);
	else
		filter.updateRate(getGyroAngles(), rms(0,0));
}
#endif
