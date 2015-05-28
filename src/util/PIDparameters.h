#ifndef PID_PARAMETERS_H
#define PID_PARAMETERS_H
struct PIDparameters{
	float P,I,D;
	float acc;
	PIDparameters(float p, float i, float d):
			P(p), I(i), D(d), acc(0) {};
	PIDparameters(): P(0), I(0), D(0), acc(0) {};
	void resetAccumulator(){ acc = 0; }
};
#endif
