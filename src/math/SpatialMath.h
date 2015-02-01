#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include <math.h>
#include "math/quaternion.h"
#include "math/vector.h"

//which functions benefit from inlining determined empirically

void Qexp(math::quaternion &q, float a);
void normalize(math::vector3d &v);
void normalize(math::quaternion &q); //was inlined
float toRad(float degrees);
float toDeg(float radians);
float invSqrt(float x);
float saSin(float t); //was inlined
float saCos(float t); //was inlined
math::quaternion fromEuler(float pitch, float roll, float yaw); //was inlined
math::vector3d lerp(math::vector3d a, math::vector3d b, float p);
math::quaternion lerp(math::quaternion const &a, math::quaternion const &b, float p);
math::quaternion nlerp(math::quaternion const &a, math::quaternion const &b, float p);

#endif
