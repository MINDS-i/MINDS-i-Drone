#ifndef PID_PARAMETERS_H
#define PID_PARAMETERS_H
struct PIDparameters {
    float P, I, D; // stored in ideal form for calculations
    float lowerBound, upperBound;
    PIDparameters() : P(0), I(0), D(0), lowerBound(-INFINITY), upperBound(INFINITY){};
    PIDparameters(float lb, float ub) : P(0), I(0), D(0), lowerBound(lb), upperBound(ub){};
    PIDparameters(float p, float i, float d) : P(p), I(i), D(d), lowerBound(-INFINITY), upperBound(INFINITY){};
    PIDparameters(float p, float i, float d, float l, float u) : P(p), I(i), D(d), lowerBound(l), upperBound(u){};
    void setBounds(float lower, float upper) {
        lowerBound = lower;
        upperBound = upper;
    }
    void setStandard(float kp, float ti, float td) {
        P = kp;
        if (ti == 0) {
            I = 0;
        } else {
            I = P / ti;
        }
        D = kp * td;
    }
    void setStandardP(float kp) {
        if (P != 0) {
            I *= (kp / P); // recalculate I and D with new parameter
            D *= (kp / P);
        }
        P = kp;
    }
    void setStandardI(float ti) {
        if (ti == 0) {
            I = 0;
        } else {
            I = P / ti;
        }
    }
    void setStandardD(float td) { D = P * td; }

    void setIdeal(float kp, float ki, float kd) {
        P = kp;
        I = ki;
        D = kd;
    }
    void setIdealP(float kp) { P = kp; }
    void setIdealI(float ki) { I = ki; }
    void setIdealD(float kd) { D = kd; }
};
#endif
