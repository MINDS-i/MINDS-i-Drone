#include "Servo.h"
#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "DroneLibs.h"
#include "util/callbackTemplate.h"

//Constants that should never change during driving and never/rarely tuned
#define useEncoder true
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {0, 7, 6}; //auto switch, drive, steer
const uint8_t EncoderPin[]= {2, 3};
const double  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
const double  dplsb       = 4.f/float(0xffff); //dps Per leastSigBit for gyro
const int ScheduleDelay   = 22;
const uint16_t warn[]     = {1000, 1600, 3000, 1600, 1000};
const double PointRadius  = .001; //in miles, margin for error in rover location
							//tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
							//hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);

//Global variables used throught the program
HardwareSerial *commSerial	= &Serial;
Storage<float> *storage	= eeStorage::getInstance();
CommManager		manager(commSerial, storage);
Settings		settings(storage);
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

//These parameters are loaded from eeprom if the code has not been reuploaded
//since when they were last set.
//the dashboard can update records in the storage instance passed into manager
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
void dangerTimeCallback(float in){ dangerTime = coastTime+in; }
void newPIDparam(float x){
	using namespace groundSettings;
	PIDparameters newPID = PIDparameters(settings.get(CRUISE_P),
										 settings.get(CRUISE_I),
										 settings.get(CRUISE_D) );
	cruise.tune(newPID);
}

void setupSettings(){
	using namespace groundSettings;
	settings.attach( LINE_GRAV	, .50  , callback<float, &lineGravity>	);
	settings.attach( STEER_THROW, 45   , callback<int  , &steerThrow>	);
	settings.attach( STEER_STYLE, 1    , callback<int  , &steerStyle>	);
	settings.attach( STEER_FAC	, 1.0  , callback<float, &steerFactor>	);
	settings.attach( MIN_FWD_SPD, 1.5  , callback<float, &minFwd>		);
	settings.attach( MAX_FWD_SPD, 6.0  , callback<float, &maxFwd>		);
	settings.attach( REV_STR_THR, 20   , callback<int  , &revThrow>		);
	settings.attach( MAX_REV_SPD, -1.5 , callback<float, &revSpeed>		);
	settings.attach( PING_WEIGHT, 1400 , callback<float, &pingWeight>	);
	settings.attach( COAST_TIME	, 1500 , callback<int  , &coastTime>	);
	settings.attach( MIN_REV_T	, 800  , &dangerTimeCallback			);
	settings.attach( CRUISE_P	, 0.05 , &newPIDparam					);
	settings.attach( CRUISE_I	, 0.1  , &newPIDparam					);
	settings.attach( CRUISE_D	, 0.0  , &newPIDparam					);
	settings.attach( TIRE_DIAM	, 5.85 , callback<float , &tireDiameter>);
	settings.attach( STR_CENTER	, 90   , callback<int   , &steerCenter> );
}

void setup() {
	setupSettings();

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
		constrain(outputAngle,
				  double(steerCenter-steerThrow),
			 	  double(steerCenter+steerThrow));

		float disp  = steerThrow - abs(steerCenter-outputAngle);
		float speed = (distance*5280.l);
		speed = min(speed, disp)/6.f; //logical speed clamps
		float approachSpeed = manager.getTargetWaypoint().getApproachSpeed();
		speed = min(speed, approachSpeed); //put in target approach speed
		constrain(speed, minFwd, maxFwd);

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
	//the one wire encoder would need to feed fabs(mph) to cruise
	//and the output's sign would need to be flipped based on sign(mph)
	if(abs(mph)<0.5f){
		cruise.stop();
	} else {
		setMPH(mph);
	}
	float outputval = cruise.calc(encoder::getRPM());
	servo[0].write(90+outputval);
#else
	servo[0].write(90+mph*(90.0f/maxFwd));
#endif
	servo[1].write(steer);
	servo[2].write(180-steer);
}

void setMPH(float mph){
	cruise.set(MPHtoRPM(mph));
}
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }
