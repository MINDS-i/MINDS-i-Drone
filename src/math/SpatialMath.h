#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include <math.h>

#ifndef INVSQRTHACK
#define INVSQRTHACK false
#endif

namespace Units {
enum Rotation { RADIANS, DEGREES };
// earth's radius in miles
const float EARTH_RAD = 3958.761;
// one revolution in radians
const float twoPI = 2.0 * M_PI;
// earth's average circumference in miles
const float EARTH_CIRC = EARTH_RAD * twoPI;
// number of feet in a mile
const float FEET_PER_MILE = 5280.0f;
} // namespace Units

/**
 * Convert input from radians to degrees
 */
float inline toRad(float degrees) { return degrees * (M_PI / 180.0); }
/**
 * Convert input from degrees to radians
 */
float inline toDeg(float radians) { return radians * (180.0 / M_PI); }
/**
 * Calculate the 1/sqrt(x)
 */
float inline invSqrt(float x) {
#if INVSQRTHACK
    int32_t i = *(int32_t*)&x; // evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1); // what the ....?
    return *(float*)&i;
#else
    return 1. / sqrt(x);
#endif
}
/**
 * Calculate the sine of `t` applying the small angle optimization for
 * small enough angles
 */
float inline saSin(float t) {
    if (fabs(t) >= .22) {
        return sin(t);
    }
    return t;
}
/**
 * Calculate the cosine of `t` applying the small angle optimization for
 * small enough angles
 */
float inline saCos(float t) {
    if (fabs(t) >= .64) {
        return cos(t);
    }
    return 1.l - ((t * t) / 2);
}
/**
 * Return a radian equivalent to `val` in the range between
 * one half turn ahead or behind `ref`
 */
float inline simplifyRadian(float ref, float val) {
    ref += M_PI;
    float diff = ref - val;
    if (diff < 0.0f) {
        diff += Units::twoPI * ceil(diff / -Units::twoPI);
    }
    return ref - fmod(diff, Units::twoPI);
}
/**
 * Return a degree equivalent to `val` in the range between
 * one half turn ahead or behind `ref`
 */
float inline simplifyDegree(float ref, float val) {
    ref += 180.0f;
    float diff = ref - val;
    if (diff < 0.0f) {
        diff += 360.0f * ceil(diff / -360.0f);
    }
    return ref - fmod(diff, 360.0f);
}
/**
 * Return the simplified distance `b`-`a` in radians in the inverval +/- PI
 */
float inline distanceRadian(float a, float b) { return simplifyRadian(0, b - a); }
/**
 * Return the simplified distance `b`-`a` in degrees in the inverval +/- 180
 */
float inline distanceDegree(float a, float b) { return simplifyDegree(0, b - a); }
/**
 * Return the radian equivalent of `val` in the inverval +/- PI
 */
float inline truncateRadian(float val) { return simplifyRadian(0, val); }
/**
 * Return the degree equivalent of `val` in the inverval +/- 180
 */
float inline truncateDegree(float val) { return simplifyDegree(0, val); }

#endif
