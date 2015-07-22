#ifndef GREATCIRCLE_H
#define GREATCIRCLE_H
#include "math.h"
#include "Waypoint.h"
#ifdef STAND_ALONE_MATH
	#include <stdint.h>
#endif

//return val in a +/- half turn range to ref
float  simplifyRadian(float ref, float val);
float  simplifyDegree(float ref, float val);
//return the angle that turns a onto b
float  distanceRadian(float   a, float   b);
float  distanceDegree(float   a, float   b);
//retrun the angle in a standard +/- half turn from 0 range
float  truncateRadian(float val);
float  truncateDegree(float val);
//deprecated, use truncateDegree instead
float  trunkAngle(float	angle);
//calculate heading from a to b in standard degrees
float  calcHeading (Waypoint a, Waypoint b);
//calculate distance in miles from a to b
float  calcDistance(Waypoint a, Waypoint b);
//calculate gps waypoint after traveling `distance` on `bearing` from `position`
Waypoint extrapPosition(Waypoint position, float bearing, float distance);

#endif
