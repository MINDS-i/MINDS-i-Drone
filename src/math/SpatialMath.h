#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include "math.h"
#include "quaternion.h"
#include "vector.h"

//which functions benefit from inlining determined empirically

void Qexp(math::quaternion &q, float a);
void normalize(math::vector3d &v);
void inline normalize(math::quaternion &q);
float toRad(float degrees);
float toDeg(float radians);
float invSqrt(float x);
float inline saSin(float t);
float inline saCos(float t);
math::quaternion inline fromEuler(float pitch, float roll, float yaw);
math::vector3d lerp(math::vector3d a, math::vector3d b, float p);
math::quaternion lerp(math::quaternion const &a, math::quaternion const &b, float p);
math::quaternion nlerp(math::quaternion const &a, math::quaternion const &b, float p);

#endif
