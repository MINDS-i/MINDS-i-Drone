#include "SpatialMath.h"

#define INVSQRTHACK false

float
toRad(float degrees){
	degrees /= 180.l;
	degrees *= M_PI;
	return degrees;
}
float
toDeg(float radians){
	radians /= M_PI;
	radians *= 180.l;
	return radians;
}
float
invSqrt(float x){
#if INVSQRTHACK
	float xhalf = 0.5f * x;     // accurate within 3%, 12 times faster
	long i = *(long*)&x;  // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);  // what the ....?
	return *(float*)&i;
#else
	return 1./sqrt(x);
#endif
}
float //was inlined
saSin(float t){
	if(fabs(t) >= .22) return sin(t);
	return t;
}
float //was inlined
saCos(float t){
	if(fabs(t) >= .64) return cos(t);
	return 1.l-((t*t)/2);
}

