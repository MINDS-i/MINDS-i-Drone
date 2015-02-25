#ifndef DUALERRORPARAMS_H
#define DUALERRORPARAMS_H
struct DualErrorParams{
	static const char GYRO = 0;
	static const char ACCL = 1;
	static const char RATE = 0;
	static const char ATTI = 1;
	float sensorMSE[2]; //gyro, accl
	float systemMSE[2]; //rate, pos
	float acclErrorFac; //factor for reducing accel certainty based on magnitude
	DualErrorParams(float gM, float aM, float rM, float pM, float aF){
		sensorMSE[GYRO] = gM;
		sensorMSE[ACCL] = aM;
		systemMSE[RATE] = rM;
		systemMSE[ATTI] = pM;
		acclErrorFac    = aF;
	}
	void setGyroMse(float gM){ sensorMSE[GYRO] = gM; }
	void setAcclMse(float aM){ sensorMSE[ACCL] = aM; }
	void setRateMse(float rM){ systemMSE[RATE] = rM; }
	void setATTIMse(float pM){ systemMSE[ATTI] = pM; }
	void setAcclEF(float aF) { acclErrorFac    = aF; }
};
#endif
