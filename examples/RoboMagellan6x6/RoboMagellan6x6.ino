#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <MINDSi.h>
#include "Compass.h"
#include "MPU.h"
#include "DroneUtils.h"
#include "GreatCircle.h"
#include "HLAverage.h"
#include "wiring_private.h"

//#define DEBUG

//assume the right direction until we know from GPS

const uint8_t LEDpin[]   = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]  = {A4, A1, A0, A2, A3}; //left to right
const uint8_t ServoPin[] = {12, 11, 8};//drive, steer, backs, outputs 1,2,3 resp
const uint8_t RadioPin[] = {2, 3}; //drive, steer

const double POINT_RADIUS = .001; //in miles, margin for error in rover location
const int SCHEDULE_DELAY = 25;
const int STEERTHROW  = 50; //distance in deg from far left to far right turn
const int REVTHROW    = 20;
const int FWDSPEED    = 116;
const int REVSPEED    = 75;
const int STOP_TIME   = 1500;//time to coast to stop
const double PING_CALIB = 1350.l;
const int DANGER_TIME = STOP_TIME+1000; //time to backup after last sighting
const uint16_t warn[] = {650, 850, 1500, 850, 650};
const double pAngle[5]={ 79.27l, 36.83l, 0.l, -36.83l, -79.27l};
const double dPlsb = 4.f/float(0xffff); //dps Per leastSigBit for gyro

HardwareSerial *CommSerial = &Serial;
NMEA nmea(Serial1);
CommManager manager(CommSerial);
Point location(0,0);
HLA lowFilter(60000*10, 0);
HLA highFilter(10, 0);
Servo servo[3]; //drive, steer, backSteer
uint16_t ping[5]={20000,20000,20000,20000,20000};
//scheduler, navigation, obstacle, stop
uint32_t uTime = 0, nTime = 0, oTime = 0, sTime = 0;
double pathHeading; //all headings are Clockwise=+, -179 to 180, 0=north
double trueHeading;
double gyroHeading[2]={0,0}; //Gyro Heading Full and Half
double distance;
double pitch=0, roll=0; //stores the observed angle from horizontal in x, y axis
double eulerPitch, eulerRoll; //stores the rotations to get to our current state
uint8_t sIter, pIter; //iterators for scheduler and ping

voidFuncPtr schedule[] = {
							extrapPosition,
							checkPing,
							readAccelerometer,
							reportLocation,
							};

int16_t x,y,z; //used in calculations
int angularError, backDir;
float outputAngle;

void setup() {
	Serial1.begin(38400);
	CommSerial->begin(9600);
	InitMPU();

	pinMode(40, OUTPUT); digitalWrite(40, HIGH);
	for(int i=0; i<3; i++) pinMode(LEDpin[i], OUTPUT);
	for(int i=0; i<3; i++) servo[i].attach(ServoPin[i]);
	output(90,90);

	sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
	sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
	sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
	sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);

	delay(1000);
	calibrateGyro(); //this also takes one second

	getRadio(RadioPin[0]);//start radio interrupts
	manager.requestResync();
	uTime = millis();
}

void loop(){
	manager.update();
	updateGPS();
	updateGyro();
	if(uTime <= millis()){
		uTime += SCHEDULE_DELAY;
		schedule[sIter]();
		sIter++;
		sIter = sIter%(sizeof(schedule)/sizeof(*schedule));
		navigate();
	}
}

void navigate(){
	if (isRadioOn(RadioPin[0])) {
		output(getRadio(RadioPin[0]), getRadio(RadioPin[1]));
	} else if (oTime != 0) {
		//Back Up
		if(sTime == 0){
			output(90, 90);
			sTime = millis();
			backDir = (ping[0]<ping[4])? 90-REVTHROW : 90+REVTHROW;
		} else if(sTime+STOP_TIME < millis()){
			output(REVSPEED, backDir);
		}

		if(oTime+DANGER_TIME < millis()){
			sTime = 0;
			oTime = 0;
		}
	} else {
		//drive based on pathHeading and side ping sensors
		angularError = trunkAngle(pathHeading - trueHeading);
		outputAngle = atan( float(angularError)*PI/180 )*(STEERTHROW/PI) + 90;

		x = cos(toRad(outputAngle));
		y = sin(toRad(outputAngle));
		for(int i=0; i<5; i++){
			double tmp = ping[i]/PING_CALIB;
			tmp *= tmp;
			x += cos(toRad(pAngle[i]))/tmp;
			y += sin(toRad(pAngle[i]))/tmp;
		}

		outputAngle = toDeg(atan2(y,x))+90;
		outputAngle = max(min(outputAngle, 90+STEERTHROW),90-STEERTHROW);
		output(FWDSPEED, outputAngle);
	}
}

void updateGPS(){
	nmea.update();
	if(nmea.newData()){
		location = nmea.getLocation();
		updateWaypoint();
		syncHeading();
		positionChanged();
	}
}

void updateWaypoint(){
	if(manager.numWaypoints() >= 1 && !nmea.getWarning()){
		distance = calcDistance(manager.getTargetWaypoint(), location);
		if(distance > POINT_RADIUS) return;

		if(manager.getTargetIndex() < manager.numWaypoints()-1)
			manager.advanceTargetIndex();
		else if (manager.loopWaypoints())
			manager.setTargetIndex(0);
	}
}

void updateGyro(){
	double dt = lowFilter.millisSinceUpdate();
	if(dt < 4.l) return;

	uint16_t Gz = MPU_Gz();
	lowFilter.update(Gz);
	highFilter.update(Gz-lowFilter.get());
	trueHeading += dt*(highFilter.get())*dPlsb;
}

void syncHeading(){
	//make the gyro useful
	trueHeading = trunkAngle(nmea.getCourse());
}

void checkPing(){
	ping[pIter] = getPing(PingPin[pIter]);
	pIter++;
	pIter = pIter%5;
	if(ping[pIter] < warn[pIter]) oTime = millis();
}

void extrapPosition(){
	float dT = millis()-nTime;
	if(dT < 1000){ //ignore irrational values
		//3600000 = milliseconds per hour
		float dTraveled = nmea.getGroundSpeed()*dT/3600000.f;
		location = extrapPosition(location, trueHeading, dTraveled);
		positionChanged();
	}
}

void positionChanged(){
	if(manager.numWaypoints() < 1 || nmea.getWarning()) return;
	nTime = millis();
	pathHeading = calcHeading(location, manager.getTargetWaypoint());
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

void calibrateGyro(){ //takes one second
	double tmp = 0;
	for(int i=0; i<100; i++){
		uint16_t Gz = MPU_Gz();
		tmp += double(Gz)/100;
		delay(10);
	}
	lowFilter.set(tmp);
}

void output(uint8_t drive, uint8_t steer){
	servo[0].write(drive);
	servo[1].write(steer);
	servo[2].write(180-steer);
}
