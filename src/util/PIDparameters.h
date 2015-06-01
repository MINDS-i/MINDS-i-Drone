#ifndef PID_PARAMETERS_H
#define PID_PARAMETERS_H
struct PIDparameters{
	float P,I,D;
	float acc;
    float lowerBound, upperBound;
	PIDparameters()
        : P(0), I(0), D(0), acc(0), lowerBound(0), upperBound(1) {};
    PIDparameters(float p, float i, float d)
        : P(p), I(i), D(d), acc(0), lowerBound(0), upperBound(1) {};
    PIDparameters(float p, float i, float d, float l, float u)
        : P(p), I(i), D(d), acc(0), lowerBound(l), upperBound(u) {};
    void setBounds(float lower, float upper){
        lowerBound = lower;
        upperBound = upper;
    }
	void resetAccumulator(){
        acc = 0;
    }
};
#endif
