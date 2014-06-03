#include "GreatCircle.h"

Point::Point(): Rlat(0), Rlng(0){
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

double
toRad(double degrees){
	degrees /= 180.;
	degrees *= PI;
	return degrees;
}
double
toDeg(double radians){
	radians /= PI;
	radians *= 180.;
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
calcDistance(Point a, Point b){  //3959 = radius of earth in miles
	double sinlat = sin((a.Rlat - b.Rlat)/2.);
	double sinlng = sin((a.Rlng - b.Rlng)/2.);
	double chord = sinlat*sinlat + sinlng*sinlng*cos(a.Rlat)*cos(b.Rlat);
	return 2.* 3959. * atan2( sqrt(chord), sqrt(1.-chord) );
}
Point
extrapPosition(Point position, double bearing, double distance){ //radians
	Point destination(0,0);
	destination.Rlat = asin(sin(position.Rlat)*cos(distance/3959) +
					 cos(position.Rlat)*sin(distance/3959)*cos(toRad(bearing)));
	destination.Rlng = position.Rlng+
		atan2(	(sin(toRad(bearing))*sin(distance/3959)*cos(position.Rlat)),
				(cos(distance/3959)-sin(position.Rlat)*sin(destination.Rlat)) );
	return destination;
}
