#include "SpatialMath.h"

#define INVSQRTHACK false

float
invSqrt(float x){
#if INVSQRTHACK
	int32_t i = *(int32_t*)&x;  // evil floating point bit level hacking
	i = 0x5f3759df - (i >> 1);  // what the ....?
	return *(float*)&i;
#else
	return 1./sqrt(x);
#endif
}
float
saSin(float t){
	if(fabs(t) >= .22) return sin(t);
	return t;
}
float
saCos(float t){
	if(fabs(t) >= .64) return cos(t);
	return 1.l-((t*t)/2);
}

