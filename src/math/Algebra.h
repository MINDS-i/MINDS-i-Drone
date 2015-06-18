#ifndef ALGEBRA_H
#define ALGEBRA_H
template <size_t rows, size_t cols>
static void
rowReduce(float (&matrix)[rows][cols]){
    //iterate down diagonol, zeroing up and down
    for(int d=0; d<min(rows,cols); d++){
        float m = matrix[d][d];
        if(m==0) continue;
        //iterate along rows that are not the row of interest
        for(int r=0; r<rows; r++){
            if(r==d) continue;
            float factor = -matrix[r][d]/m;
            //iterate along row doing calculations
            for(int i=d; i<cols; i++){
                matrix[r][i] += factor*matrix[d][i];
            }
        }
    }
}
inline float
cubicHorner(float x, const float (&horner)[4]){
    float b = horner[0];
    for(int i=1; i<4; i++){
        b *= x;
        b += horner[i];
    }
    return b;
}
inline float
sqrtCurve(float input){
    float res = sqrt(fabs(input));
    return copysign(res, input);
}
inline float
cubeCurve(float input){
    float res = input*input*input;
    return res;
}
inline float
squareCurve(float input){
    float res = input*input;
    return copysign(res, input);
}
inline float
atanCurve(float input){
    float res = atan(fabs(input));
    return copysign(res, input);
}
inline float
logCurve(float input){
    float res = log(fabs(input)+1.0);
    return copysign(res, input);
}
class ThrottleCurve{
    /**
     * Built from a cubic function (a*x^3 + b*x^2 + c*x + d) defined at
     * f(-1) = 0
     * f(1)  = 1
     * f(0)  = hoverPoint
     * with the last degree of freedom setting the x^3 vs x balance
     *     which corresponds to how "linear" the curve is
     */
private:
    float a,b,c,d;
public:
    ThrottleCurve(float linearity, float hoverPoint){
        this->set(linearity, hoverPoint);
    }
    void set(float linearity, float hoverPoint){
        a = 0.5-linearity;
        b = 0.5-hoverPoint;
        c = linearity;
        d = hoverPoint;
    }
    void setLinearity(float linearity){
        a = 0.5-linearity;
        c = linearity;
    }
    void setHoverPoint(float hoverPoint){
        b = 0.5-hoverPoint;
        d = hoverPoint;
    }
    float get(float value){
        const float x = value*2.0f - 1.0f;
        return d + x*(c + x*(b + x*a));
    }
};
#endif
