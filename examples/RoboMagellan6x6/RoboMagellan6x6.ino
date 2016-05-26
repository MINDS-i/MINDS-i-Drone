#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "MINDS-i-Drone.h"
#include "util/callbackTemplate.h"

//Constants that should never change during driving and never/rarely tuned
#define useEncoder true
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {7, 0, 1}; //auto switch, drive, steer
const uint8_t EncoderPin[]= {2, 3};
const double  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
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
LEA6H			gps;
MPU6000			mpu;
Waypoint		location(0,0);
Waypoint		backWaypoint(0,0);
HLA				lowFilter (600000, 0);//10 minutes
HLA				highFilter(    10, 0);//10 milliseconds
HLA 			pitch( 100, 0);
HLA 			roll ( 100, 0);
PIDparameters   cruisePID(0,0,0,-90,90);
PIDcontroller   cruise(&cruisePID);
ServoGenerator::Servo servo[3]; //drive, steer, backSteer
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

void checkPing();
void readAccelerometer();
void reportLocation();
void extrapPosition();
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
	cruisePID = PIDparameters(settings.get(CRUISE_P),
                              settings.get(CRUISE_I),
                              settings.get(CRUISE_D), -90, 90 );
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

inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }

void setup() {
	setupSettings();
    ServoGenerator::setup(20000);

	gps.begin();
	mpu.begin();
	commSerial->begin(Protocol::BAUD_RATE);
	for(int i=0; i<3; i++) pinMode(LEDpin[i], OUTPUT);
	for(int i=0; i<3; i++) servo[i].attach(ServoPin[i]);
	output(0.0f,steerCenter);

	delay(2000);
	calibrateGyro(); //this also takes one second

	setupAPM2radio();
	#if useEncoder
		encoder::begin(EncoderPin[0], EncoderPin[1]);
	#endif
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
		outputAngle = constrain(outputAngle,
				  				double(steerCenter-steerThrow),
			 	  				double(steerCenter+steerThrow));

		float disp  = steerThrow - abs(steerCenter-outputAngle);
		float speed = (distance*5280.l);
		speed = min(speed, disp)/6.f; //logical speed clamps
		float approachSpeed = manager.getTargetWaypoint().getApproachSpeed();
		speed = min(speed, approachSpeed); //put in target approach speed
		speed = constrain(speed, minFwd, maxFwd);

		if(stop) output(0 , steerCenter);
		else     output(speed, outputAngle);
	}
}

void updateGPS(){
	gps.update();
	if(gps.newData()){
		location = gps.getLocation();
		waypointUpdated();
		syncHeading();
		positionChanged();
		gpsTime = millis();
	}
}

void waypointUpdated(){
	if(manager.numWaypoints() > 0 && !gps.getWarning()){
		stop = false;
 		distance = calcDistance(manager.getTargetWaypoint(),  location);

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
	float dt = lowFilter.millisSinceUpdate();
	float Gz = -toDeg(mpu.gyroZ());
	lowFilter.update(Gz);
	highFilter.update(Gz-lowFilter.get());
	trueHeading = trunkAngle(trueHeading + dt*(highFilter.get()));

	if(gpsHalfTime < millis() && gpsHalfTime!=0){
		gyroHalf = trueHeading;
		gpsHalfTime = 0;
	}
}

void syncHeading(){
	if(!gps.getWarning() && gps.getCourse()!=0){
		trueHeading = gps.getCourse();
		/*
		if(millis() - gpsTime < 1500) //dont use gyrohalf if it is too old
			trueHeading = trunkAngle(gps.getCourse() + trueHeading - gyroHalf);
		else
			trueHeading = trunkAngle(gps.getCourse());
		gpsHalfTime = millis()+(millis()-gpsTime)/2;*/
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
	if(dT < 1000 && !gps.getWarning()){ //ignore irrational values
		//3600000 = milliseconds per hour
		float dTraveled = gps.getGroundSpeed()*dT/3600000.f;
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
	Ax = mpu.acclX();
	Ay = mpu.acclY();
	Az = mpu.acclZ();
	pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
	roll .update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
}

void reportLocation(){
	float voltage  = float(analogRead(67)/1024.l*5.l*10.1f);
	float amperage = float(analogRead(66)/1024.l*5.l*17.0f);
	manager.sendTelem(Protocol::telemetryType(LATITUDE),    location.degLatitude());
	manager.sendTelem(Protocol::telemetryType(LONGITUDE),   location.degLongitude());
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading);
	manager.sendTelem(Protocol::telemetryType(PITCH),       toDeg(pitch.get())-90);
	manager.sendTelem(Protocol::telemetryType(ROLL),        toDeg(roll.get())-90);
	manager.sendTelem(Protocol::telemetryType(GROUNDSPEED), RPMtoMPH(encoder::getRPM()));
	manager.sendTelem(Protocol::telemetryType(VOLTAGE),     voltage);
	manager.sendTelem(Protocol::telemetryType(VOLTAGE+1),   amperage);
}

void calibrateGyro(){ //takes one second
	double tmp = 0;
	for(int i=0; i<100; i++){
		double Gz = toDeg(mpu.gyroZ());
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
		cruise.set(MPHtoRPM(mph));
	}
	float outputval = cruise.calc(encoder::getRPM());
	servo[0].write(90+outputval);
#else
	servo[0].write(90+mph*(90.0f/maxFwd));
#endif
	servo[1].write(steer);
	servo[2].write(180-steer);
}
