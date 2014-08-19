#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <MINDSi.h>
#include "APM/Compass.h"
#include "APM/MPU.h"
#include "DroneUtils.h"
#include "GreatCircle.h"
#include "HLAverage.h"
#include "wiring_private.h"

//Constants that should never change during driving and never/rarely tuned
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A4, A1, A0, A2, A3}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {2, 3}; //drive, steer
const double  pAngle[5]   = { 79.27l, 36.83l, 0.l, -36.83l, -79.27l};
const double  dPlsb       = 4.f/float(0xffff); //dps Per leastSigBit for gyro
const int ScheduleDelay   = 22;
const uint16_t warn[]     = {1000, 1600, 2500, 1600, 1000};
const double PointRadius  = .001; //in miles, margin for error in rover location

//Global variables used throught the program
HardwareSerial *CommSerial = &Serial;
CommManager		manager(CommSerial);
NMEA			nmea(Serial1);
Point			location(0,0);
Point			backWaypoint(0,0);
HLA				lowFilter (600000, 0);//10 minutes
HLA				highFilter(    10, 0);//10 milliseconds
HLA 			pitch( 100, 0);
HLA 			roll ( 100, 0);
Servo			servo[3]; //drive, steer, backSteer
		//scheduler, navigation, obstacle, stop times
uint32_t uTime = 0, nTime = 0, oTime = 0, sTime = 0;
uint32_t gpsHalfTime = 0, gpsTime = 0;
 int32_t Ax,Ay,Az; //used in accelerometer calculations
uint16_t ping[5] = {20000,20000,20000,20000,20000};
uint8_t  sIter,pIter; //iterators for scheduler and ping
double   pathHeading; //All Headings are Clockwise+, -179 to 180, 0=north
double   trueHeading;
double   gyroHalf; //Store Gyro Heading halfway between gps points
double   distance;
boolean  stop = true;
boolean  backDir;

voidFuncPtr schedule[] = {
							extrapPosition,
							checkPing,
							readAccelerometer,
							reportLocation,
							};

//These parameters are synced with the dashboard in real time
//The defaults are set during setup and sent to the dash when a connection is
//made; after that they can be changed in the dash and changes will
//be sent to the robot.
#define LINEGRAVITY  manager.getFloat(10)
#define STEERTHROW   manager.getInt  (11)
#define STEER_STYLE  manager.getInt  (12)
#define STEER_FACTOR manager.getFloat(13)
#define MIN_FWD      manager.getInt  (14)
#define MAX_FWD      manager.getInt  (15)
#define REVTHROW     manager.getInt  (16)
#define REVSPEED     manager.getInt  (17)
#define PING_WEIGHT  manager.getFloat(18)
#define COAST_TIME   manager.getInt  (19)
#define DANGER_TIME  manager.getInt  (20)+manager.getInt(19)
void setTuningDefaults(){
	manager.setData(10, .50);  //LINEGRAVITY
	manager.setData(11, 45);   //STEERTHROW
	manager.setData(12, 1);    //STEER_STYLE
	manager.setData(13, 1.0);  //STEER_FACTOR
	manager.setData(14, 107);  //MIN_FWD
	manager.setData(15, 115);  //MAX_FWD
	manager.setData(16, 20);   //REVERSE_STEER_THROW
	manager.setData(17, 75);   //REVERSE_SPEED
	manager.setData(18, 1400.);//PING_WEIGHT
	manager.setData(19, 1500); //COAST_TIME
	manager.setData(20, 800);  //Minumum Backup Time
}

void setup() {
	Serial1.begin(38400);
	CommSerial->begin(Protocol::BAUD_RATE);
	InitMPU();
	pinMode(40, OUTPUT); digitalWrite(40, HIGH); //SPI select pin
	for(int i=0; i<3; i++) pinMode(LEDpin[i], OUTPUT);
	for(int i=0; i<3; i++) servo[i].attach(ServoPin[i]);
	output(90,90);

	sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
	sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
	sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
	sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);

	delay(2000);
	calibrateGyro(); //this also takes one second

	getRadio(RadioPin[0]);//start radio interrupts
	manager.requestResync();
	uTime = millis();

	setTuningDefaults();
}

void loop(){
	manager.update();
	updateGPS();
	updateGyro();
	if(uTime <= millis()){
		uTime += ScheduleDelay;
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
			backDir = ping[0]<ping[4];
		} else if(sTime+COAST_TIME < millis()){
			if(backDir) output(REVSPEED, 90-REVTHROW);
			else 		output(REVSPEED, 90+REVTHROW);
		}

		if(oTime+DANGER_TIME < millis()){
			sTime = 0;
			oTime = 0;
		}
	} else {
		//drive based on pathHeading and side ping sensors
		double x,y;
		double angularError = trunkAngle(pathHeading - trueHeading);
		double outputAngle;
		switch(STEER_STYLE){
			case 0:
				outputAngle = atan( angularError*PI/180.l )*(2*STEERTHROW/PI);
				break;
			case 1:
				outputAngle = ((angularError/180.l)*(angularError/180.l));
				outputAngle *=(2.l*STEERTHROW);
				if(angularError < 0) outputAngle *= -1;
				break;
			default:
				outputAngle = (angularError/180.l)*(2.l*STEERTHROW);
				break;
		}
		outputAngle *= STEER_FACTOR;

		x = cos(toRad(outputAngle));
		y = sin(toRad(outputAngle));
		for(int i=0; i<5; i++){
			double tmp = ping[i]/PING_WEIGHT;
			tmp *= tmp;
			x += cos(toRad(pAngle[i]))/tmp;
			y += sin(toRad(pAngle[i]))/tmp;
		}

		outputAngle = toDeg(atan2(y,x))+90;
		bound(double(90-STEERTHROW), outputAngle, double(90+STEERTHROW));

		//try slowing down on steep turns
		int disp = STEERTHROW - abs(90-outputAngle);
		int speed = (distance*5280.l);
		speed = min(speed, disp);
		speed += 90;
		bound(MIN_FWD, speed, MAX_FWD);

		if(stop) output(90 ,90);
		else     output(speed, outputAngle);
	}
}

void updateGPS(){
	nmea.update();
	if(nmea.newData()){
		location = nmea.getLocation();
		waypointUpdated();
		syncHeading();
		positionChanged();
		gpsTime = millis();
	}
}

void waypointUpdated(){
	if(manager.numWaypoints() > 0 && !nmea.getWarning()){
		stop = false;
 		distance = calcDistance(manager.getTargetWaypoint(), location);
		if(distance > PointRadius)  return;

		if(manager.getTargetIndex() < manager.numWaypoints()-1){
			backWaypoint = manager.getTargetWaypoint();
			manager.advanceTargetIndex();
		}
		else if (manager.loopWaypoints()){
			backWaypoint = manager.getTargetWaypoint();
			manager.setTargetIndex(0);
		}
		else{
			stop = true;
		}
	}
}

void updateGyro(){
	double dt = lowFilter.millisSinceUpdate();
	double Gz = MPU_Gz();
	lowFilter.update(Gz);
	highFilter.update(Gz-lowFilter.get());
	trueHeading = trunkAngle(trueHeading + dt*(highFilter.get())*dPlsb);

	if(gpsHalfTime < millis() && gpsHalfTime!=0){
		gyroHalf = trueHeading;
		gpsHalfTime = 0;
	}
}

void syncHeading(){
	if(!nmea.getWarning() && nmea.getCourse()!=0){
		if(millis() - gpsTime < 1500) //dont use gyrohalf if it is too old
			trueHeading = trunkAngle(nmea.getCourse() + trueHeading - gyroHalf);
		else
			trueHeading = trunkAngle(nmea.getCourse());
		gpsHalfTime = millis()+(millis()-gpsTime)/2;
	}
	else if(stop) trueHeading = pathHeading;
}

void checkPing(){
	ping[pIter] = getPing(PingPin[pIter]);
	pIter+=2;
	pIter = pIter%5;
	if(ping[pIter] < warn[pIter]) oTime = millis();
}

void extrapPosition(){
	float dT = millis()-nTime;
	if(dT < 1000 && !nmea.getWarning()){ //ignore irrational values
		//3600000 = milliseconds per hour
		float dTraveled = nmea.getGroundSpeed()*dT/3600000.f;
		dTraveled *= (2.l/3.l);//purposly undershoot
		location = extrapPosition(location, trueHeading, dTraveled);
	}
	positionChanged();
}

void positionChanged(){
	nTime = millis();
	if(manager.numWaypoints() <= 0) return;

	distance = calcDistance(manager.getTargetWaypoint(), location);
	if(backWaypoint.radLongitude() == 0 || distance*5280.l < 25){
		pathHeading = calcHeading(location, manager.getTargetWaypoint());
	} else {
		double full  = calcDistance(backWaypoint, manager.getTargetWaypoint());
		double AB    = calcHeading(backWaypoint, manager.getTargetWaypoint());
		double AL    = calcHeading(backWaypoint, location);
		double d     = cos(toRad(AL-AB))*calcDistance(backWaypoint, location);
		double D     = d + (full-d)*(1.l-LINEGRAVITY);
		Point target = extrapPosition(backWaypoint, AB, D);
		pathHeading  = calcHeading(location, target);
	}
}

void readAccelerometer(){
	Ax = MPU_Ax();
	Ay = MPU_Ay();
	Az = MPU_Az();
	pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
	roll .update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
}

void reportLocation(){
	float voltage = float(analogRead(67)/1024.l*5.l*10.1l);
	manager.setData(Protocol::DATA_LATITUDE,   location.degLatitude());
	manager.setData(Protocol::DATA_LONGITUDE,  location.degLongitude());
	manager.setData(Protocol::DATA_HEADING,	   trueHeading);
	manager.setData(Protocol::DATA_PITCH,      pitch.get()*180/PI);
	manager.setData(Protocol::DATA_ROLL,       roll.get()*180/PI);
	manager.setData(Protocol::DATA_SPEED,      nmea.getGroundSpeed());
	manager.setData(Protocol::DATA_VOLTAGE,    voltage);
	manager.setData(20, manager.getTargetWaypoint().degLatitude () );
	manager.setData(21, manager.getTargetWaypoint().degLongitude() );
	manager.setData(22, manager.numWaypoints());
}

void calibrateGyro(){ //takes one second
	double tmp = 0;
	for(int i=0; i<100; i++){
		double Gz = MPU_Gz();
		tmp += Gz/100;
		delay(10);
	}
	lowFilter.set(tmp);
}

void output(uint8_t drive, uint8_t steer){
	servo[0].write(drive);
	servo[1].write(steer);
	servo[2].write(180-steer);
}
