#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <MINDSi.h>
#include "Compass.h"
#include "MPU.h"
#include "DroneUtils.h"
#include "GreatCircle.h"

#define DEBUG
#define LEARN_COMPASS_ERROR
#define UBLOX_GPS_FIX

const double FILTER_PARAM = .988; //closer to 1 is slower "learning"
const double POINT_RADIUS = .01; //in miles, margin for error in rover location
const int BLUE = 25;
const int YLLW = 26;
const int RED  = 27;
const int drivePin = 11; //output 2
const int steerPin = 12; //output 1
const int backSPin = 5;  //output 8
const int STEERTHROW = 70; //distance in degrees from far left to far right turn
const int driveSpeed = 120;

short XSHIFT = 50;
short YSHIFT = 280;
short XSCALE = 600;
short YSCALE = 725;

NMEA nmea(Serial1);
Servo drive,steer,backSteer;

double pathHeading; //all headings are Clockwise=+, -179 to 180, 0=north
double trueHeading;
double gpsFoundShift=-70;
double distance;
double gpsHeading;
double pitch=0, roll=0; //stores the observed angle from horizontal in x, y axis
double eigenPitch, eigenRoll; //stores the rotations to get to our current state
Point location(0,0);
unsigned long ttime = 0;
HardwareSerial *CommSerial = &Serial;
CommManager manager(CommSerial);

int angularError;
float outputAngle; //used in calculations
long x,y,z;

void setup() {
	beginCompass();
	drive.attach(drivePin);
	steer.attach(steerPin);
	backSteer.attach(backSPin);
	stop();
	Serial1.begin(38400);
	CommSerial->begin(57600);
	InitMPU();

	pinMode(40, OUTPUT);
	digitalWrite(40, HIGH);
	pinMode(RED , OUTPUT);
	pinMode(BLUE, OUTPUT);

	delay(1000);//provides 2 second delay for arming
	#ifndef DEBUG
		manager.requestResync();
		calibrateCompass();
	#endif

	#ifdef UBLOX_GPS_FIX
		Serial1.write(GPS_SETUP, sizeof(GPS_SETUP)); //setup 3DR LEA6 GPS module
	#endif
}

void loop() {
	manager.update();
#ifndef DEBUG
	updateGPS();
#endif
	if(ttime < millis()){
		ttime = millis()+200;
		observeHeading();
		if(manager.numWaypoints() >= 1 && !nmea.getWarning()){
			distance = calcDistance(manager.getTargetWaypoint(), location);
			if(distance > POINT_RADIUS)
				navigate(manager.getTargetWaypoint());
			else if(manager.getTargetIndex() < manager.numWaypoints()-1)
				manager.advanceTargetIndex();
			else if (manager.loopWaypoints())
				manager.setTargetIndex(0);
			else
				stop();
		}
		readAccelerometer();
		reportLocation();
	}
}

void navigate(Point target){
	pathHeading = calcHeading(location, target);
	angularError = (pathHeading - trueHeading);
	angularError = ((angularError+540)%360)-180;
	outputAngle = atan( float(angularError)*PI/180 )*(STEERTHROW/PI) + 90;

#ifdef DEBUG
	trueHeading += atan( float(angularError)*PI/180 )*(20);
	location = extrapPosition(location, trueHeading, POINT_RADIUS);
#else
	steer.write( outputAngle );
	backSteer.write( 180-outputAngle );
	drive.write( driveSpeed );
#endif
}

void stop(){
	drive.write(90);
	steer.write(90);
	backSteer.write(90);
}

void updateGPS(){
	nmea.update();
	if(nmea.newData()){
		location = nmea.getLocation();
		location.update(location.degLatitude(),location.degLongitude());
		digitalWrite(BLUE, nmea.getWarning()); //blue on when signal is found
		digitalWrite(RED , LOW);
	}
}

void observeHeading(){
	double CompassHeading = getHeadingTiltComp(XSHIFT, XSCALE,
											   YSHIFT, YSCALE,
											   eigenPitch, eigenRoll);
	if(!nmea.getWarning()) CompassHeading-=nmea.getMagVar();

	if(nmea.getCourse() != 0){
		gpsHeading = nmea.getCourse();
		gpsHeading = ((int(gpsHeading)+540)%360)-180;

		int error = CompassHeading-gpsHeading;
		error = ((error+540)%360)-180;

		#ifdef LEARN_COMPASS_ERROR
			//calculate high pass filter for compass, low sig from GPS
			if(fabs(pitch)-.18<0 && fabs(roll)-.18<0){  // .18 ~ 10 degrees
				gpsFoundShift = gpsFoundShift*FILTER_PARAM +
									float(error)*(1-FILTER_PARAM);
			}
		#endif
	}
#ifndef DEBUG
	trueHeading = CompassHeading+gpsFoundShift;
#endif
}

void readAccelerometer(){
	x = MPU_Ax();
	y = MPU_Ay();
	z = MPU_Az();

	pitch = atan2(sqrt(x*x+z*z), y)*.4+pitch*.6;
	roll  = atan2(sqrt(y*y+z*z), -x)*.4+ roll*.6;
	eigenPitch = atan2(x, z);
	eigenRoll = atan2( float(y), float(x)/sin(eigenPitch) );
}

void reportLocation(){
	manager.sendDataMessage(Protocol::DATA_LATITUDE, location.degLatitude());
	manager.sendDataMessage(Protocol::DATA_LONGITUDE, location.degLongitude());
	manager.sendDataMessage(Protocol::DATA_DISTANCE, distance);
	manager.sendDataMessage(Protocol::DATA_HEADING, trueHeading);

	manager.sendDataMessage(Protocol::DATA_ROLL, roll*180/PI);
	manager.sendDataMessage(Protocol::DATA_PITCH, pitch*180/PI);
	manager.sendDataMessage(Protocol::DATA_SPEED, gpsFoundShift);
	// manager.sendDataMessage(Protocol::DATA_SPEED, nmea.getGroundSpeed());
}

void calibrateCompass(){
	long endTime = millis()+15000;
	int maxX = -0xfff;
	int maxY = -0xfff;
	int minX =  0xfff;
	int minY =  0xfff;
	int x,y,z;

	steer.write( 90-STEERTHROW/2 );
	backSteer.write( 90+STEERTHROW/2 );
	drive.write(driveSpeed);
	while( millis()<endTime){
		manager.update();
		rawCompass(&x,&y,&z);
		if(y < minY) minY = y;
		if(x < minX) minX = x;
		if(y > maxY) maxY = y;
		if(x > maxX) maxX = x;
	}
	stop();

	XSCALE = (maxX-minX)/2;
	YSCALE = (maxY-minY)/2;
	XSHIFT = -(maxX+minX)/2;
	YSHIFT = -(maxY+minY)/2;
}
