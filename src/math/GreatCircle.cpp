#include "GreatCircle.h"

const float _eRad = 3963.1676; //earth's radius in miles

#define TEST(a) Serial.print(#a);Serial.print(": ");Serial.print(a);Serial.print("\t");

/*make integer versions*/
#define TWO_M_PI 6.283185307179586476925
float simplifyRadian(float ref, float val){
	ref += M_PI;
	float diff = ref - val;
	if(diff < 0.0f) diff += TWO_M_PI * ceil(diff / -TWO_M_PI);
	return ref - fmod(diff, TWO_M_PI);
}
float simplifyDegree(float ref, float val){
	ref += 180.0f;
	float diff = ref - val;
	if(diff < 0.0f) diff += 360.0f * ceil(diff / -360.0f);
	return ref - fmod(diff, 360.0f);
}
float distanceRadian(float   a, float   b){ return simplifyRadian(0, b-a); }
float distanceDegree(float   a, float   b){ return simplifyDegree(0, b-a); }
float truncateRadian(float val){ return simplifyRadian(0, val); }
float truncateDegree(float val){ return simplifyDegree(0, val); }
float trunkAngle(float angle)  { return simplifyDegree(0, angle); }
// deprecated name

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
	return 2. * _eRad * atan2( sqrt(chord), sqrt(1.-chord) );
}

Waypoint
extrapPosition(Waypoint position, float bearing, float distance){ //degrees,miles
	float rlat, rlng;
	float pRlat = position.radLatitude();
	float pRlng = position.radLongitude();
	rlat = asin(sin(pRlat)*cos(distance/_eRad) +
					 cos(pRlat)*sin(distance/_eRad)*cos(toRad(bearing)));
	rlng = pRlng+
		atan2(	(sin(toRad(bearing))*sin(distance/_eRad)*cos(pRlat)),
				(cos(distance/_eRad)-sin(pRlat)*sin(rlat)) );
	Waypoint destination(rlat, rlng, true);
	return destination;
}


