#include "Servo.h"
#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "DroneLibs.h"
#include "util/callbackTemplate.h"

#define useEncoder true

const uint8_t STORAGE_VER_IDX	= 31;
const uint8_t STORAGE_VER		= 17;

//Constants that should never change during driving and never/rarely tuned
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {0, 7, 6}; //auto switch, drive, steer
const uint8_t EncoderPin[]= {2, 3};
const double  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
const double  dplsb       = 4.f/float(0xffff); //dps Per leastSigBit for gyro
const int ScheduleDelay   = 22;
const uint16_t warn[]     = {1000, 1600, 2500, 1600, 1000};
const double PointRadius  = .001; //in miles, margin for error in rover location
							//tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
							//hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);

//Global variables used throught the program
HardwareSerial *commSerial	= &Serial;
Storage<float> *settings	= eeStorage::getInstance();
CommManager		manager(commSerial, settings);
NMEA			nmea(Serial1);
Waypoint		location(0,0);
Waypoint		backWaypoint(0,0);
HLA				lowFilter (600000, 0);//10 minutes
HLA				highFilter(    10, 0);//10 milliseconds
HLA 			pitch( 100, 0);
HLA 			roll ( 100, 0);
Servo			servo[3]; //drive, steer, backSteer
PIDcontroller   cruise;
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

void (*schedule[])(void) = {	extrapPosition,
								checkPing,
								readAccelerometer,
								reportLocation,
								};

//These parameters are loaded from eeprom in STORAGE_VER is valid
//the dashboard can update records in the storage passed into manager
//The callbacks keep them up to date while the code is running
float	lineGravity;
int		steerThrow, steerStyle;
float	steerFactor;
float	minFwd, maxFwd;
int		revThrow;
float	revSpeed;
float	pingWeight;
int		coastTime, dangerTime; //danger should include coastTime
float	tireDiameter;
int		steerCenter;

void writeDefaults(){
	settings->updateRecord( 0, .50);  //lineGravity
	settings->updateRecord( 1, 45);   //steerThrow
	settings->updateRecord( 2, 1);    //steerStyle
	settings->updateRecord( 3, 1.0);  //steerFactor
	settings->updateRecord( 4, 1.0);  //minFwd
	settings->updateRecord( 5, 6.0);  //maxFwd
	settings->updateRecord( 6, 20);   //REVERSE_STEER_THROW
	settings->updateRecord( 7, -1.5); //REVERSE_SPEED
	settings->updateRecord( 8, 1400.);//pingWeight
	settings->updateRecord( 9, 1500); //coastTime
	settings->updateRecord(10, 800);  //Minumum Backup Time
	settings->updateRecord(11, 0.05); //Cruise control P
	settings->updateRecord(12, 0.1);  //Cruise control I
	settings->updateRecord(13, 0.0);  //Cruine control D
	settings->updateRecord(14, 5.85); //tire Diameter
	settings->updateRecord(15, 90);   //steer Center
	settings->updateRecord(STORAGE_VER_IDX, STORAGE_VER);
}
void dangerTimeCallback(float in){ dangerTime = coastTime+in; }
void newPIDparam(float x){
	PIDparameters newPID = PIDparameters(settings->getRecord(11),
										 settings->getRecord(12),
										 settings->getRecord(13) );
	cruise.tune(newPID);
}
void setCallbacks(){
	settings->attachCallback( 0, callback<float, &lineGravity>	);
	settings->attachCallback( 1, callback<int  , &steerThrow>	);
	settings->attachCallback( 2, callback<int  , &steerStyle>	);
	settings->attachCallback( 3, callback<float, &steerFactor>	);
	settings->attachCallback( 4, callback<float, &minFwd>		);
	settings->attachCallback( 5, callback<float, &maxFwd>		);
	settings->attachCallback( 6, callback<int  , &revThrow>		);
	settings->attachCallback( 7, callback<float, &revSpeed>		);
	settings->attachCallback( 8, callback<float, &pingWeight>	);
	settings->attachCallback( 9, callback<int  , &coastTime>	);
	settings->attachCallback(10, &dangerTimeCallback);
	settings->attachCallback(11, &newPIDparam);
	settings->attachCallback(12, &newPIDparam);
	settings->attachCallback(13, &newPIDparam);
	settings->attachCallback(14, callback<float , &tireDiameter>);
	settings->attachCallback(15, callback<int   , &steerCenter> );
}

void setup() {
	if(settings->getRecord(STORAGE_VER_IDX) != STORAGE_VER)
		writeDefaults();
	setCallbacks();

	Serial1.begin(38400);
	commSerial->begin(Protocol::BAUD_RATE);
	InitMPU();
	pinMode(40, OUTPUT); digitalWrite(40, HIGH); //SPI select pin
	for(int i=0; i<3; i++) pinMode(LEDpin[i], OUTPUT);
	for(int i=0; i<3; i++) servo[i].attach(ServoPin[i]);
	output(0.0f,steerCenter);

	sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
	sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
	sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
	sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);

	delay(2000);
	calibrateGyro(); //this also takes one second

	setupAPM2radio();
	encoder::begin(EncoderPin[0], EncoderPin[1]);
	manager.requestResync();
	uTime = millis();
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
	float   mph = ((getAPM2Radio(RadioPin[1])-90) / 90.f)*maxFwd;
	uint8_t steer = getAPM2Radio(RadioPin[2]);
	if (abs(steer-steerCenter) > 5 || fabs(mph)>0.8f ) {
			//(getAPM2Radio(RadioPin[0]) > 120) {
		output(mph, steer);
	} else if (oTime != 0) {
		//Back Up
		if(sTime == 0){
			output(0, steerCenter);
			sTime = millis();
			backDir = ping[0]<ping[4];
		} else if(sTime+coastTime < millis()){
			if(backDir) output(revSpeed, steerCenter-revThrow);
			else 		output(revSpeed, steerCenter+revThrow);
		}

		if(oTime+dangerTime < millis()){
			sTime = 0;
			oTime = 0;
		}
	} else {
		//drive based on pathHeading and side ping sensors
		double x,y;
		double angularError = trunkAngle(pathHeading - trueHeading);
		double outputAngle;
		switch(steerStyle){
			case 0:
				outputAngle = atan( angularError*PI/180.l )*(2*steerThrow/PI);
				break;
			case 1:
				outputAngle = ((angularError/180.l)*(angularError/180.l));
				outputAngle *=(2.l*steerThrow);
				if(angularError < 0) outputAngle *= -1;
				break;
			default:
				outputAngle = (angularError/180.l)*(2.l*steerThrow);
				break;
		}
		outputAngle *= steerFactor;

		x = cos(toRad(outputAngle));
		y = sin(toRad(outputAngle));
		for(int i=0; i<5; i++){
			double tmp = ping[i]/pingWeight;
			tmp *= tmp;
			x += cos(toRad(pAngle[i]))/tmp;
			y += sin(toRad(pAngle[i]))/tmp;
		}

		outputAngle = toDeg(atan2(y,x))+steerCenter;
		bound(double(steerCenter-steerThrow),
							outputAngle,
			  double(steerCenter+steerThrow));

		float disp  = steerThrow - abs(steerCenter-outputAngle);
		float speed = (distance*5280.l);
		speed = min(speed, disp)/6.f; //logical speed clamps
		float targetApproachSpeed = manager.getTargetWaypoint().getExtra();
		speed = min(speed, targetApproachSpeed); //put in target approach speed
		bound(minFwd, speed, maxFwd);

		if(stop) output(0 , steerCenter);
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
	trueHeading = trunkAngle(trueHeading + dt*(highFilter.get())*dplsb);

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
		double D     = d + (full-d)*(1.l-lineGravity);
		Waypoint target = extrapPosition(backWaypoint, AB, D);
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
	manager.sendTelem(Protocol::telemetryType(LATITUDE),  location.degLatitude());
	manager.sendTelem(Protocol::telemetryType(LONGITUDE), location.degLongitude());
	manager.sendTelem(Protocol::telemetryType(HEADING),   trueHeading);
	manager.sendTelem(Protocol::telemetryType(PITCH),     pitch.get()*180/PI);
	manager.sendTelem(Protocol::telemetryType(ROLL),      roll.get()*180/PI);
	manager.sendTelem(Protocol::telemetryType(SPEED),     RPMtoMPH(encoder::getRPM()));
	manager.sendTelem(Protocol::telemetryType(VOLTAGE),   voltage);
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

void output(float mph, uint8_t steer){
#if useEncoder
	if(abs(mph)<0.5f){
		cruise.stop();
	} else {
		setMPH(mph);
	}
	float outputval = cruise.calc(encoder::getRPM());
	servo[0].write(90+outputval);
#else
	servo[0].write(90+mph*6);
#endif
	servo[1].write(steer);
	servo[2].write(180-steer);
}

void setMPH(float mph){
	cruise.set(MPHtoRPM(mph));
}
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }
