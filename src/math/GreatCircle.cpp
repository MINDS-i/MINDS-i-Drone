#include "GreatCircle.h"

double _eRad = 3963.1676; //earth's radius in miles

Point::Point(): Rlat(0), Rlng(0), altitude(0){
}

Point::Point(double latitude,double longitude){
	Rlat = toRad(latitude);
	Rlng = toRad(longitude);
}

Point::Point(double latitude, double longitude, bool rad){
	if(rad){
		Rlat = latitude;
		Rlng = longitude;
	}else{
		Rlat = toRad(latitude);
		Rlng = toRad(longitude);
	}
}

Point::Point(double latitude, double longitude, uint16_t alt){
	Rlat = toRad(latitude);
	Rlng = toRad(longitude);
	altitude = alt;
}

void
Point::update(double latitude,double longitude){
	Rlat = toRad(latitude);
	Rlng = toRad(longitude);
}
void
Point::update(double latitude, double longitude, bool rad){
	if(rad){
		Rlat = latitude;
		Rlng = longitude;
	}else{
		Rlat = toRad(latitude);
		Rlng = toRad(longitude);
	}
}
double
Point::radLatitude(){
	return Rlat;
}
double
Point::radLongitude(){
	return Rlng;
}
double
Point::degLatitude(){
	return toDeg(Rlat);
}
double
Point::degLongitude(){
	return toDeg(Rlng);
}
uint16_t
Point::getAltitude(){
	return altitude;
}
void
Point::setAltitude(uint16_t alt){
	altitude = alt;
}

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
double
toRad(double degrees){
	degrees /= 180.l;
	degrees *= PI;
	return degrees;
}
double
toDeg(double radians){
	radians /= PI;
	radians *= 180.l;
	return radians;
}
double
calcHeading(Point a, Point b){
	double y = sin(b.Rlng - a.Rlng) * cos(b.Rlat);
	double x = cos(a.Rlat)*sin(b.Rlat)
								 - sin(a.Rlat)*cos(b.Rlat)*cos(b.Rlng - a.Rlng);
	return toDeg(  atan2(y,x)  );
}
double
calcDistance(Point a, Point b){
	double sinlat = sin((a.Rlat - b.Rlat)/2.);
	double sinlng = sin((a.Rlng - b.Rlng)/2.);
	double chord = sinlat*sinlat + sinlng*sinlng*cos(a.Rlat)*cos(b.Rlat);
	return 2. * _eRad * atan2( sqrt(chord), sqrt(1.-chord) );
}

Point
extrapPosition(Point position, double bearing, double distance){ //degrees,miles
	Point destination(0,0);

	destination.Rlat = asin(sin(position.Rlat)*cos(distance/_eRad) +
					 cos(position.Rlat)*sin(distance/_eRad)*cos(toRad(bearing)));
	destination.Rlng = position.Rlng+
		atan2(	(sin(toRad(bearing))*sin(distance/_eRad)*cos(position.Rlat)),
				(cos(distance/_eRad)-sin(position.Rlat)*sin(destination.Rlat)) );
	return destination;
}


