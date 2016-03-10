#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include <math.h>

float inline toRad(float degrees) {
    return degrees * (M_PI/180.0);
}
float inline toDeg(float radians) {
    return radians * (180.0/M_PI);
}
float invSqrt(float x);
float saSin(float t);
float saCos(float t);

#endif
