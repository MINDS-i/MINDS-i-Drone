#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include <math.h>

float toRad(float degrees);
float toDeg(float radians);
float invSqrt(float x);
float saSin(float t); //was inlined
float saCos(float t); //was inlined

#endif
