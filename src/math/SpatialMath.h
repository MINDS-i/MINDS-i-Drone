#ifndef SPATIAL_MATH_H
#define SPATIAL_MATH_H

#include "math/quaternion.h"
#include "math/vector.h"
#include "math/SpatialMath.h"

#define INVSQRTHACK true

float
invSqrt(float x){
#if INVSQRTHACK
	float xhalf = 0.5f * x;     // accurate within 3%, 12 times faster
	int32_t i = *(int32_t*)&x;  // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);  // what the ....?
	return *(float*)&i;
#else
	return 1./sqrt(x);
#endif
}
void inline
normalize(math::quaternion &q){
	q *= invSqrt( q.Scalar()  *q.Scalar()   +
				  q.Vector().x*q.Vector().x +
				  q.Vector().y*q.Vector().y +
				  q.Vector().z*q.Vector().z   );
}
void
normalize(math::vector3d &v){
	v *= invSqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
float inline
saSin(float t){
	if(fabs(t) >= .22) return sin(t);
	return t;
}
float inline
saCos(float t){
	if(fabs(t) >= .64) return cos(t);
	return 1.l-((t*t)/2);
}
math::quaternion inline
fromEuler(float pitch, float roll, float yaw) throw() {
	float c1 = saCos(pitch/2.);
	float s1 = saSin(pitch/2.);
	float c2 = saCos(  yaw/2.);
	float s2 = saSin(  yaw/2.);
	float c3 = saCos( roll/2.);
	float s3 = saSin( roll/2.);
	float c1c2 = c1*c2;
	float s1s2 = s1*s2;
	float w = c1c2*c3 -  s1s2*s3;
	float x = c1c2*s3 +  s1s2*c3;
	float y =s1*c2*c3 + c1*s2*s3;
	float z =c1*s2*c3 - s1*c2*s3;
	math::quaternion rtn(w, x, y, z);
	return rtn;
}
void
Qexp(math::quaternion &q, float a){
	//assumes unit quaternion; small angles
	float theta = a*acos(q._s);
	q._s = saCos(theta);
	math::vector3d tmp = q._v;
	tmp /= sqrt( tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z );
	tmp *= saSin(theta);
	q._v = tmp;
	normalize(q);
}
math::vector3d
lerp(math::vector3d a, math::vector3d b, float p) { //faster without using refs
	math::vector3d ans = math::vector3d(p*(a.x-b.x)+b.x,
										p*(a.y-b.y)+b.y,
										p*(a.z-b.z)+b.z );
	return ans;
}
math::quaternion
lerp(math::quaternion const &a, math::quaternion const &b, float p) {
	math::quaternion ans = math::quaternion(p*(  a._s-b._s  ) + b._s,
											p*(a._v.x-b._v.x) + b._v.x,
											p*(a._v.y-b._v.y) + b._v.y,
											p*(a._v.z-b._v.z) + b._v.z );
	return ans;
}
math::quaternion
nlerp(math::quaternion const &a, math::quaternion const &b, float p) {
	math::quaternion ans = lerp(a, b, p);
	normalize(ans);
	return ans;
}

#endif
