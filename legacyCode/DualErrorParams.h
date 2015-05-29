#ifndef DUALERRORPARAMS_H
#define DUALERRORPARAMS_H
class DualErrorParams{
public:
	float sysMSE;
	float acclMSE;
	float acclEF;
	DualErrorParams(float systemMSE, float accelerometerMSE, float acclErrorFact)
		:sysMSE(systemMSE), acclMSE(accelerometerMSE), acclEF(acclErrorFact) {}
	void setSysMSE(float mse) { sysMSE	= mse; }
	void setAcclMSE(float mse){ acclMSE	= mse; }
	void setAcclEF(float aEF) { acclEF	= aEF; }
};
#endif
