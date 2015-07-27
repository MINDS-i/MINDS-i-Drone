#include "GreatCircle.h"

namespace{ //keep these constants to this file
	const float earthRad = 3958.761; //radius in miles
	const float twoPI    = 6.283185307179586476925; // one revolution in radians
}

float simplifyRadian(float ref, float val){
	ref += M_PI;
	float diff = ref - val;
	if(diff < 0.0f) diff += twoPI * ceil(diff / -twoPI);
	return ref - fmod(diff, twoPI);
}
float simplifyDegree(float ref, float val){
	ref += 180.0f;
	float diff = ref - val;
	if(diff < 0.0f) diff += 360.0f * ceil(diff / -360.0f);
	return ref - fmod(diff, 360.0f);
}
inline float distanceRadian(float   a, float   b){ return simplifyRadian(0, b-a); }
inline float distanceDegree(float   a, float   b){ return simplifyDegree(0, b-a); }
inline float truncateRadian(float val){ return simplifyRadian(0, val); }
inline float truncateDegree(float val){ return simplifyDegree(0, val); }
inline float trunkAngle(float angle)  { return simplifyDegree(0, angle); }

float
calcHeading(Waypoint a, Waypoint b){
	float aRlat = a.radLatitude();
	float aRlng = a.radLongitude();
	float bRlat = b.radLatitude();
	float bRlng = b.radLongitude();
	float y = sin(bRlng - aRlng) * cos(bRlat);
	float x = cos(aRlat)*sin(bRlat)
					- sin(aRlat)*cos(bRlat)*cos(bRlng - aRlng);
	return toDeg(  atan2(y,x)  );
}
float
calcDistance(Waypoint a, Waypoint b){
	float aRlat = a.radLatitude();
	float aRlng = a.radLongitude();
	float bRlat = b.radLatitude();
	float bRlng = b.radLongitude();
	float sinlat = sin((aRlat - bRlat)/2.);
	float sinlng = sin((aRlng - bRlng)/2.);
	float chord = sinlat*sinlat + sinlng*sinlng*cos(aRlat)*cos(bRlat);
	return 2. * earthRad * atan2( sqrt(chord), sqrt(1.-chord) );
}

Waypoint
extrapPosition(Waypoint position, float bearing, float distance){ //degrees,miles
	float rlat, rlng;
	float pRlat = position.radLatitude();
	float pRlng = position.radLongitude();
	rlat = asin(sin(pRlat)*cos(distance/earthRad) +
					 cos(pRlat)*sin(distance/earthRad)*cos(toRad(bearing)));
	rlng = pRlng+
		atan2(	(sin(toRad(bearing))*sin(distance/earthRad)*cos(pRlat)),
				(cos(distance/earthRad)-sin(pRlat)*sin(rlat)) );
	Waypoint destination(rlat, rlng, true);
	return destination;
}


