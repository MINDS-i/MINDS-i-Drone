#ifndef GREATCIRCLE_H
#define GREATCIRCLE_H
#include "math.h"
#include "Arduino.h"

class Point{
public:
	double Rlat, Rlng;
	uint16_t altitude;
	Point();
	Point(double, double); //degrees
	Point(double, double, bool); //set true for radian entry
	Point(double, double, uint16_t); //set true for radian entry
	void update(double, double);
	void update(double latitude, double longitude, bool rad);
	double radLatitude();
	double radLongitude();
	double degLatitude();
	double degLongitude();
	uint16_t getAltitude();
	void setAltitude(uint16_t alt);
};

float 	trunkAngle(float	 angle);
double 	trunkAngle(double angle);
int		trunkAngle(int 	 angle);
double 	toRad(double degrees);
double	toDeg(double radians);
double 	calcHeading(Point a, Point b);
double 	calcDistance(Point a, Point b); //all distances are in Miles
Point 	extrapPosition(Point position, double bearing, double distance);

#endif
