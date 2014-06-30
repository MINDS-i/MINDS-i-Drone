#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <MINDSi.h>
#include "Compass.h"
#include "MPU.h"
#include "DroneUtils.h"
#include "GreatCircle.h"

//#define DEBUG
#define UBLOX_GPS_FIX

//clean up now GPS comm code
//Test with interGPS point extrapolation
//Test with strong form compass Shift
//Make template function for wrapping directions

//Ping sensors
//front = A0; front left = A1; front right = A2; Right = A3; left = A4

const double POINT_RADIUS = .001; //in miles, margin for error in rover location

const double FILTER  = .995;

const int BLUE = 25;
const int YLLW = 26;
const int RED  = 27;
const int drivePin = 12; //output 1
const int steerPin = 11; //output 2
const int backSPin = 8;  //output 3
const int STEERTHROW = 70; //distance in degrees from far left to far right turn
const int driveSpeed = 116;

const int driveInput = 2;
const int steerInput = 3;

short XSHIFT = 50;
short YSHIFT = 280;
short XSCALE = 600;
short YSCALE = 725;

NMEA nmea(Serial1);
Servo drive,steer,backSteer;

double pathHeading; //all headings are Clockwise=+, -179 to 180, 0=north
double trueHeading;
double gpsFoundShift=-90;
double compassHeading;
double distance;
double gpsHeading;
double pitch=0, roll=0; //stores the observed angle from horizontal in x, y axis
double eulerPitch, eulerRoll; //stores the rotations to get to our current state
Point location(0,0);
unsigned long ttime = 0, navTime = 0;
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
	CommSerial->begin(9600);
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
		Serial1.write(GPS_SETUP, sizeof(GPS_SETUP));
		sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);
		//sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
		//sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
		//sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
	#endif

	getRadio(driveInput);
}

void loop() {
	manager.update();
	updateGPS(); //off in debug
	if(!isRadioOn(driveInput)){


		if(ttime < millis()){
			ttime = millis()+150;
			readAccelerometer();
			observeHeading();//off in debug
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
			reportLocation();
		}


	} else {

		if(ttime < millis()){
			ttime = millis()+200;
			observeHeading();
			readAccelerometer();
			reportLocation();
		}

		int tmp = getRadio(steerInput);
		steer.write( tmp );
		backSteer.write( 180-tmp );
		drive.write( getRadio(driveInput) );

	}
}

void navigate(Point target){
	pathHeading = calcHeading(location, target);
	angularError = (pathHeading - trueHeading);
	angularError = trunkAngle(angularError);
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
		digitalWrite(BLUE, nmea.getWarning()); //blue on when signal is found
		digitalWrite(RED , LOW);

		location = nmea.getLocation();

		if(nmea.getCourse()!=0) gpsHeading = nmea.getCourse();
		gpsHeading = trunkAngle(gpsHeading);
		gpsFoundShift = gpsHeading - compassHeading;
		navTime = millis();
	}
}

void observeHeading(){
/*	float tmp = getHeadingTiltComp(XSHIFT, XSCALE, YSHIFT, YSCALE,
										eulerPitch, eulerRoll);
*/
	trueHeading = gpsHeading;

	float dT = millis()-navTime;
	if(dT < 1000){ //ignore irrational values
		//3600000 = milliseconds per hour
		float dTraveled = nmea.getGroundSpeed()*dT/3600000.f;
		location = extrapPosition(location, trueHeading, dTraveled);
	}
	navTime = millis();
}

void readAccelerometer(){
	x = MPU_Ax();
	y = MPU_Ay();
	z = MPU_Az();

	pitch = atan2(sqrt(x*x+z*z), y)*.4+pitch*.6;
	roll  = atan2(sqrt(y*y+z*z), -x)*.4+ roll*.6;
	eulerPitch = atan2(x, z);
	eulerRoll = atan2( float(y), float(x)/sin(eulerPitch) );
}

void reportLocation(){
	manager.sendDataMessage(Protocol::DATA_LATITUDE, location.degLatitude());
	manager.sendDataMessage(Protocol::DATA_LONGITUDE, location.degLongitude());
	manager.sendDataMessage(Protocol::DATA_ROLL, roll*180/PI);
	manager.sendDataMessage(Protocol::DATA_PITCH, pitch*180/PI);
	manager.sendDataMessage(Protocol::DATA_HEADING, trueHeading);
	manager.sendDataMessage(Protocol::DATA_DISTANCE, distance);
	manager.sendDataMessage(Protocol::DATA_SPEED, nmea.getGroundSpeed());
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
