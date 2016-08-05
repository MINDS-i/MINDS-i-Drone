#ifndef GREATCIRCLE_H
#define GREATCIRCLE_H
#include "math.h"
#include "Waypoint.h"

//return val in a +/- half turn range to ref
float simplifyRadian(float ref, float val);
float simplifyDegree(float ref, float val);
//return the angle that turns a onto b
inline float distanceRadian(float a, float b){ return simplifyRadian(0, b-a); }
inline float distanceDegree(float a, float b){ return simplifyDegree(0, b-a); }
//return the angle in a standard +/- half turn from 0 range
inline float truncateRadian(float val){ return simplifyRadian(0, val); }
inline float truncateDegree(float val){ return simplifyDegree(0, val); }
//deprecated, use truncateDegree instead
inline float trunkAngle(float angle)  { return simplifyDegree(0, angle); }
//calculate heading from a to b, North = 0.0, counter clockwise positive
float calcHeading (Waypoint a, Waypoint b);
//calculate distance in miles from a to b
float calcDistance(Waypoint a, Waypoint b);
//calculate gps waypoint after traveling `distance` on `bearing` from `position`
Waypoint extrapPosition(Waypoint position, float bearing, float distance);


#endif
