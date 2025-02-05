#ifndef ALGEBRA_H
#define ALGEBRA_H
template <size_t rows, size_t cols> static void rowReduce(float (&matrix)[rows][cols]) {
    // iterate down diagonol, zeroing up and down
    for (size_t d = 0; d < min(rows, cols); d++) {
        float m = matrix[d][d];
        if (m == 0) {
            continue;
        }
        // iterate along rows that are not the row of interest
        for (size_t r = 0; r < rows; r++) {
            if (r == d) {
                continue;
            }
            float factor = -matrix[r][d] / m;
            // iterate along row doing calculations
            for (size_t i = d; i < cols; i++) {
                matrix[r][i] += factor * matrix[d][i];
            }
        }
    }
}
inline float cubicHorner(float x, const float (&horner)[4]) {
    float b = horner[0];
    for (int i = 1; i < 4; i++) {
        b *= x;
        b += horner[i];
    }
    return b;
}
inline float sqrtCurve(float input) {
    float res = sqrt(fabs(input));
    return copysign(res, input);
}
inline float cubeCurve(float input) {
    float res = input * input * input;
    return res;
}
inline float squareCurve(float input) {
    float res = input * input;
    return copysign(res, input);
}
inline float atanCurve(float input) {
    float res = atan(fabs(input));
    return copysign(res, input);
}
inline float logCurve(float input) {
    float res = log(fabs(input) + 1.0);
    return copysign(res, input);
}
class ThrottleCurve {
    /**
     * Built from a cubic function (a*x^3 + b*x^2 + c*x + d) defined at
     * f(-1) = 0
     * f( 1) = 1
     * f( 0) = hoverPoint
     * with the last degree of freedom setting the x^3 vs x balance
     *     which corresponds to how "linear" the curve is
     */
  private:
  public:
    float a, b, c, d;
    /**
     * Construct a ThrottleCurve given a linearity (0, 1) and hoverPoint (0, 1)
     */
    ThrottleCurve(float linearity, float hoverPoint) {
        setLinearity(linearity);
        setHoverPoint(hoverPoint);
    }
    /**
     * How linear or curved the throttle curve is
     * A Value of 0.5 is as linear as possible
     * A Value near 0.0 is curved for finer control around the hover point
     * A value near 1.0 is curved for finer control at the extremes
     * Only defined for values between 0.0 and 1.0 exclusive
     * @param linearity How "linear" the final curve should be
     */
    void setLinearity(float linearity) {
        a = 0.5 - linearity;
        c = linearity;
    }
    /**
     * Set the value that the curve outputs when the input throttle value is
     *     at 50%
     * Only defined for values between 0.0 and 1.0 exclusive
     * @param hoverPoint The output value that becomes half throttle
     */
    void setHoverPoint(float hoverPoint) {
        b = 0.5 - hoverPoint;
        d = hoverPoint;
    }
    /**
     * Get a curved throttle value
     *     Minimum throttle when value = 0.0
     *     Maximum throttle when value = 1.0
     * @param  value Input to be curved
     * @return       The curved value
     */
    float get(float value) const {
        const float x = value * 2.0f - 1.0f;
        return d + x * (c + x * (b + x * a));
    }
};
#endif
