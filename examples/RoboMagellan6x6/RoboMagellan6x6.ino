#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "MINDS-i-Drone.h"
#include "util/callbackTemplate.h"
#include "platforms/Ardupilot.h"
using namespace Platform;

//Constants that should never change during driving and never/rarely tuned
#define useEncoder true
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {7, 0, 1}; //auto switch, drive, steer
const uint8_t EncoderPin[]= {2/*APM pin 7*/, 3 /*APM pin 6*/};
const double  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
const uint16_t warn[]     = {1000, 1600, 3000, 1600, 1000};
const double PointRadius  = .001; //in miles, margin for error in rover location
							//tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
							//hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);

//Global variables used throught the program
Waypoint location(0,0);
Waypoint backWaypoint(0,0);
PIDparameters cruisePID(0,0,0,-90,90);
PIDcontroller cruise(&cruisePID);
ServoGenerator::Servo servo[3]; //drive, steer, backSteer
		//obstacle, stop times
uint32_t oTime = 0, sTime = 0;
uint16_t ping[5] = {20000,20000,20000,20000,20000};
float pathHeading; //All Headings are CCW+ around down, -179 to 180, 0=north
float trueHeading;
float headingLastGPSUpdate;
float distance;
boolean stop = true;
boolean backDir;
auto extrapolationTimer = Interval::timer();

void checkPing();
void reportLocation();
void extrapPosition();
void (*schedule[])(void) = { extrapPosition, checkPing, reportLocation };
auto ScheduleTimer = Interval::every(22); //milliseconds
const int ScheduleSize = sizeof(schedule)/sizeof(schedule[0]);

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
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }

void setupSettings();

RCFilter orientation(0.003,0.0015);
InertialVec* sens[2] = {&hmc, &mpu};
Translator   conv[2] = {Translators::APM, Translators::APM};
InertialManager imu(sens, conv, 2);

void isrCallback(uint16_t microseconds){
    float ms = ((float)microseconds)/1000.0;
    imu.update();
    orientation.update(imu, ms);
}

void setup() {
	Platform::beginAPM();
	setupSettings();

    ServoGenerator::setUpdateCallback(isrCallback);

	commSerial->begin(Protocol::BAUD_RATE);
	for(int i=0; i<3; i++) pinMode(LEDpin[i], OUTPUT);
	for(int i=0; i<3; i++) servo[i].attach(ServoPin[i]);
	output(0.0f,steerCenter);

	delay(2000);

	#if useEncoder
		encoder::begin(EncoderPin[0], EncoderPin[1]);
	#endif
	comms.requestResync();
}

void loop(){
	Platform::updateAPM();
	updatePath();
	navigate();

	if(ScheduleTimer()){
		static uint8_t sIter = 0;
		schedule[sIter]();
		sIter++;
		sIter = sIter%ScheduleSize;
	}
}

void navigate(){
	float   mph = ((APMRadio::get(RadioPin[1])-90) / 90.f)*maxFwd;
	uint8_t steer = APMRadio::get(RadioPin[2]);
	if (abs(steer-steerCenter) > 5 || fabs(mph)>0.8f ) {
			//(APMRadio::get(RadioPin[0]) > 120) {
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
		double angularError = truncateDegree(pathHeading - trueHeading);

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
		float approachSpeed = comms.getTargetWaypoint().getApproachSpeed();
		speed = min(speed, approachSpeed); //put in target approach speed
		speed = constrain(speed, minFwd, maxFwd);

		if(stop) output(0 , steerCenter);
		else     output(speed, outputAngle);
	}
}

void updatePath(){
	static auto readIdx = gps.dataIndex();
	if(gps.dataIndex() != readIdx){
		readIdx = gps.dataIndex();

		headingLastGPSUpdate = orientation.getYaw();
		location = gps.getLocation();
		extrapolationTimer.reset();
		waypointUpdated();
		positionChanged();
	}
	trueHeading = gps.getCourse() + (orientation.getYaw()-headingLastGPSUpdate);
}

void waypointUpdated(){
	if(comms.numWaypoints() > 0 && !gps.getWarning()){
		stop = false;
 		distance = comms.getTargetWaypoint().distanceTo(location);

		if(distance > PointRadius)  return;

		if(comms.getTargetIndex() < comms.numWaypoints()-1){
			backWaypoint = comms.getTargetWaypoint();
			comms.advanceTargetIndex();
		}
		else if (comms.loopWaypoints()){
			backWaypoint = comms.getTargetWaypoint();
			comms.setTargetIndex(0);
		}
		else{
			stop = true;
		}
	}
}

void checkPing(){
	static uint8_t pIter = 0;
	ping[pIter] = getPing(PingPin[pIter]);
	pIter+=2;
	pIter = pIter%5;
	if(ping[pIter] < warn[pIter]) oTime = millis();
}

void extrapPosition(){
	uint32_t dt = extrapolationTimer();
	if(dt > 1E5) {
		extrapolationTimer.reset();

		// convert to miles per hour
		float dTraveled = gps.getGroundSpeed()*((float)dt)/(60.f*60.f*1E6f);
		dTraveled *= (2.l/3.l); //intetionally undershoot

		comms.sendTelem(AMPERAGE+4, dTraveled);
		comms.sendTelem(AMPERAGE+5, trueHeading);

		if(dTraveled != 0){
			location = location.extrapolate(trueHeading, dTraveled);
		}
	}
	positionChanged();
}

void positionChanged(){
	if(comms.numWaypoints() <= 0) return;

	distance = comms.getTargetWaypoint().distanceTo(location);
	if(backWaypoint.radLongitude() == 0 || distance*5280.l < 25){
		pathHeading = location.headingTo(comms.getTargetWaypoint());
	} else {
		double full = backWaypoint.distanceTo(comms.getTargetWaypoint());
		double AB   = backWaypoint.headingTo(comms.getTargetWaypoint());
		double AL   = backWaypoint.headingTo(location);
		double d    = cos(toRad(AL-AB)) * backWaypoint.distanceTo(location);
		double D    = d + (full-d)*(1.l-lineGravity);
		Waypoint target = backWaypoint.extrapolate(AB, D);
		pathHeading  = location.headingTo(target);
	}
}

void reportLocation(){
	using namespace Protocol;
	comms.sendTelem(LATITUDE,    location.degLatitude());
	comms.sendTelem(LONGITUDE,   location.degLongitude());
	comms.sendTelem(HEADING,     trueHeading);
	comms.sendTelem(PITCH,       toDeg(orientation.getPitch()));
	comms.sendTelem(ROLL,        toDeg(orientation.getRoll()));
	comms.sendTelem(GROUNDSPEED, RPMtoMPH(encoder::getRPM()));
	comms.sendTelem(VOLTAGE,     power.getVoltage());
	comms.sendTelem(AMPERAGE,    power.getAmperage());

	comms.sendTelem(AMPERAGE+1,  gps.getLatitude());
	comms.sendTelem(AMPERAGE+2,  gps.getLongitude());
	comms.sendTelem(AMPERAGE+3,  gps.getGroundSpeed());
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
	float outputval = cruise.update(encoder::getRPM());
	servo[0].write(90+outputval);
#else
	servo[0].write(90+mph*(90.0f/maxFwd));
#endif
	servo[1].write(steer);
	servo[2].write(180-steer);
}

void setupSettings(){
	/*GROUNDSETTING index="0" name="line gravity " min="0" max="1" def="0.50"
	 *Defines how strongly the rover should attempt to return to the original
	 *course between waypoints, verses the direct path from its current location
	 * to the target<br>
	 */
	settings.attach(0, .50, callback<float, &lineGravity>);

	/*GROUNDSETTING index="1" name="steer throw" min="0" max="90" def="45"
	 *The number of degrees that rover will turn its wheels when it needs to
	 *to turn its most extreme amount
	 */
	settings.attach(1, 45, callback<int, &steerThrow>);

	/*GROUNDSETTING index="2" name="steer style" min="0" max="2" def="1"
	 *switches between arctangent of error steering (0) <br>
	 *square of error steering (1) <br>
	 *and proportional to error steering (2)
	 */
	settings.attach(2, 1, callback<int, &steerStyle>);

	/*GROUNDSETTING index="3" name="steer scalar" min="0" max="+inf" def="1"
	 *Multiplier that determines how aggressively to steer
	 */
	settings.attach(3, 1.0, callback<float, &steerFactor>);

	/*GROUNDSETTING index="4" name="min fwd speed" min="0" max="+inf" def="1.5"
	 *minimum forward driving speed in MPH
	 */
	settings.attach(4, 1.5, callback<float, &minFwd>);

	/*GROUNDSETTING index="5" name="max fwd speed" min="0" max="+inf" def="6.0"
	 *maximum forward driving speed in MPH
	 */
	settings.attach(5, 6.0, callback<float, &maxFwd>);

	/*GROUNDSETTING index="6" name="rev str throw" min="0" max="90" def="20"
	 *How far to turn the wheels when backing away from an obstacle
	 */
	settings.attach(6, 20, callback<int, &revThrow>);

	/*GROUNDSETTING index="7" name="reverse speed" min="-inf" max="0" def="-1.5"
	 *speed in MPH to drive in reverse
	 */
	settings.attach(7, -1.5, callback<float, &revSpeed>);

	/*GROUNDSETTING index="8" name="ping factor" min="1" max="+inf" def="1400"
	 *Factor to determine how strongly obstacles effect the rover's course <br>
	 *Larger numbers correspond to larger effects from obstacles
	 */
	settings.attach(8, 1400, callback<float, &pingWeight>);

	/*GROUNDSETTING index="9" name="coast time" min="0" max="+inf" def="1500"
	 *Time in milliseconds to coast before reversing when an obstacle is encountered
	 */
	settings.attach(9, 1500, callback<int, &coastTime>);

	/*GROUNDSETTING index="10" name="min rev time" min="0" max="+inf" def="800"
	 *minimum time in milliseconds to reverse away from an obstacle
	 */
	settings.attach(10, 800, [](float in){
		// The program logic has dangerTime start after coastTime completes
		// so the time when danger is over must be shifted by coastTime
		dangerTime = coastTime+in;
	});

	/*GROUNDSETTING index="11" name="Cruise P" min="0" max="+inf" def="0.05"
	 *P term in cruise control PID loop
	 */
	settings.attach(11, 0.05, [](float p){ cruisePID.setIdealP(p); });

	/*GROUNDSETTING index="12" name="Cruise I" min="0" max="+inf" def="0.1"
	 *I term in cruise control PID loop
	 */
	settings.attach(12, 0.1, [](float i){ cruisePID.setIdealI(i); });

	/*GROUNDSETTING index="13" name="Cruise D" min="0" max="+inf" def="0.0"
	 *D term in cruise control PID loop
	 */
	settings.attach(13, 0.0, [](float d){ cruisePID.setIdealD(d); });

	/*GROUNDSETTING index="14" name="Tire Diameter" min="0" max="+inf" def="5.85"
	 *Tire Diameter in inches, used to calculate MPH
	 */
	settings.attach(14, 5.85, callback<float, &tireDiameter>);

	/*GROUNDSETTING index="15" name="Steer Center" min="0" max="180" def="90"
	 *Center point in degrees corresponding to driving straight
	 */
	settings.attach(15, 90, callback<int, &steerCenter>);
}

