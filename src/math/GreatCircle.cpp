#include "GreatCircle.h"

const float _eRad = 3963.1676; //earth's radius in miles

float trunkAngle(float angle){
	return trunkAngle(double(angle));//float and double are the same on arduino
}
double
trunkAngle(double angle){
	angle += 180.l;
	while(angle < 0.l) angle += 360.l;
	return fmod(angle,360.l)-180.l;
}
int
trunkAngle(int 	angle){
	angle += 180;
	while(angle < 0) angle += 360;
	return (angle%360)-180;
}
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


