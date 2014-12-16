#ifndef PID_PARAMETERS_H
#define PID_PARAMETERS_H
struct PIDparameters{
	float P,I,D;
	PIDparameters(float p, float i, float d):
			P(p), I(i), D(d) {};
	PIDparameters(): P(1.), I(0), D(0) {};
};
#endif
