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
//const uint8_t EncoderPin[]= {2/*APM pin 7*/, 3 /*APM pin 6*/};
const float  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
//const int ScheduleDelay   = 22;
const uint16_t warn[]     = {1000, 1600, 3000, 1600, 1000};
const float PointRadius  = .001; //in miles, margin for error in rover location
							//tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
							//hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);

//Global variables used throught the program
#ifdef ptest
HardwareSerial *commSerial	= &Serial2;
#else
HardwareSerial *commSerial	= &Serial;
#endif
Storage<float> *storage	= eeStorage::getInstance();
CommManager		manager(commSerial, storage);
Settings		settings(storage);
LEA6H_sim		gps;
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
uint32_t uTime = 0, nTime = 0, oTime = 0, sTime = 0, simTime = 0;
uint32_t gpsHalfTime = 0, gpsTime = 0;
 int32_t Ax,Ay,Az; //used in accelerometer calculations
uint16_t ping[5] = {20000,20000,20000,20000,20000};
uint8_t  sIter,pIter; //iterators for scheduler and ping
float   pathHeading; //All Headings are Clockwise+, -179 to 180, 0=north
float   trueHeading;
float   gyroHalf; //Store Gyro Heading halfway between gps points
float   distance;
boolean  stop = true;
boolean  backDir;

void checkPing();
void readAccelerometer();
void reportLocation();
void extrapPosition();
void updateSIM();

//delay,last scheduled time
uint32_t scheduleDelay[][2] = 
{
	{40,0},
	{22,22},
	{22,44},
	{22,66},
	{500,88}
};

void (*schedule[])(void) = 
{	
	extrapPosition,
	readAccelerometer,
	reportLocation,
	checkPing,
	updateSIM,
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
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }

void setupSettings();



void setup() {
	setupSettings();

	gps.begin();
	mpu.begin();

	//for debugging.  If Serial == commSerial then will be overwritten with next call
	Serial.begin(115200);
	commSerial->begin(57600);


	for(int i=0; i<3; i++) 
		pinMode(LEDpin[i], OUTPUT);
	
	for(int i=0; i<3; i++) 
		servo[i].attach(ServoPin[i]);

	output(0.0f,steerCenter);

	delay(2000);
	calibrateGyro(); //this also takes one second


	manager.requestResync();

	uTime = millis();

	#ifdef ptest
	manager.addWaypoint(47.626919, -117.642860, Units::DEGREES, 0);
	#endif

	//ba testing
	pinMode(A5, OUTPUT);
    pinMode(A6, OUTPUT);

    pinMode(A7, OUTPUT);    
	pinMode(A8, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(45, OUTPUT);
}

void loop()
{
	int i;
	digitalWrite(A5, HIGH);

	digitalWrite(A6, HIGH);
	manager.update();
	digitalWrite(A6, LOW);

	digitalWrite(A6, HIGH);
	updateGPS();
	digitalWrite(A6, LOW);

	digitalWrite(A6, HIGH);
	updateGyro();
	digitalWrite(A6, LOW);

	
	for (i=0;i<sizeof(schedule)/sizeof(*schedule);i++)
	{
		digitalWrite(A6, HIGH);
		if(scheduleDelay[i][1] <= millis())
		{
			//digitalWrite(A5, HIGH);	
			scheduleDelay[i][1] = scheduleDelay[i][0]+millis();
			schedule[i]();
			navigate();
			//digitalWrite(A5, LOW);

		}
		digitalWrite(A6, LOW);			
	}

		// if(uTime <= millis())
		// {
		// 	digitalWrite(A6, HIGH);
		// 	uTime += ScheduleDelay;
		// 	schedule[sIter]();
		// 	sIter++;
		// 	sIter = sIter%(sizeof(schedule)/sizeof(*schedule));
		// 	navigate();
		// 	digitalWrite(A6, LOW);			
		// }

	
	digitalWrite(A5, LOW);
}

void updateSIM()
{
	//find point on path that is along trueheading with length of mph converted to distance travel in 1/5 a second.
	//todo Set some randomness to path?
	
	//extrapPosition basically does what we want.
	float dT = millis()-simTime;

	//Why less then 1second?  (from extrapPositions=)
	//if (dT < 1000 )
	//{ 
		gps.setUpdatedRMC();
		//3600000 = milliseconds per hour
		float dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		//float dTraveled = 5*dT/3600000.f;
		//Serial.println(dTraveled,8);
		//Serial.println(gps.getGroundSpeed(),8);
		//todo.  Add Some kind of randomness?
		Waypoint newLocation = location.extrapolate(trueHeading, dTraveled);

		gps.setLatitude(newLocation.degLatitude());
		//Serial.println(newLocation.degLatitude(),8);
		gps.setLongitude(newLocation.degLongitude());
		//Serial.println(newLocation.degLongitude(),8);
	//}

	simTime=millis();
}

void navigate()
{
	if (oTime != 0) 
	{
		//Back Up
		if(sTime == 0)
		{
			output(0, steerCenter);
			sTime = millis();
			backDir = ping[0]<ping[4];
		} 
		else if(sTime+coastTime < millis())
		{
			if(backDir) 
				output(revSpeed, steerCenter-revThrow);
			else 		
				output(revSpeed, steerCenter+revThrow);
		}

		if(oTime+dangerTime < millis())
		{
			sTime = 0;
			oTime = 0;
		}
	} 
	else 
	{
		//drive based on pathHeading and side ping sensors
		float x,y;
		//difference between pathheadgin (based on waypoints (previous and current))
		//error can only be as much as 180 degrees off (opposite directions).
		float angularError = truncateDegree(pathHeading - trueHeading);

		float outputAngle;
		switch(steerStyle){
			case 0:
				//not sure.  Ratio of opposite/adjacent multiplied by steering throw
				//not sure why divided by PI.
				outputAngle = atan( angularError*PI/180.l )*(2*steerThrow/PI);
				break;
			case 1:
				//get squared of percentage of error from 180
				outputAngle = ((angularError/180.l)*(angularError/180.l));
				//above percentage of the full steerthrow (example: 45 degree*.01)
				outputAngle *=(2.l*steerThrow);
				//negative angle if left turning vs right turning ( or opposite?)
				if(angularError < 0)
					outputAngle *= -1;
				break;
			default:
				//simplest. Percentage of error multipled by 2x the steerthow
				// if angularError is 90 degrees off we max out the steering (left/right)
				// this is constrained below
				// at this point we could be 180 degress off and have output angle of 90
				outputAngle = (angularError/180.l)*(2.l*steerThrow);
				break;
		}

		//scale angle (default of 1, no change)
		outputAngle *= steerFactor;

		//find x and y component of output angle
		x = cos(toRad(outputAngle));
		y = sin(toRad(outputAngle));

		//Not 100 sure:  Generally using pings sensor values to adjust output angle
		//modify x and y based on what pings are seeing		
		for(int i=0; i<5; i++)
		{
			//temp is some percentage (at ping of 1400 would mean tmp==1)
			float tmp = ping[i]/pingWeight;
			//value is squared?
			tmp *= tmp;
			//add to x,y based on set angle of sensor inverse scaled of percentage
			x += cos(toRad(pAngle[i]))/tmp;
			y += sin(toRad(pAngle[i]))/tmp;
		}

		//determine angle based on x,y then adjust from steering center ( 0 )
		outputAngle = toDeg(atan2(y,x))+steerCenter;
		//Can't steer more then throw
		outputAngle = constrain(outputAngle,
				  				double(steerCenter-steerThrow),
			 	  				double(steerCenter+steerThrow));


		//disp should with default values be [(-45)-45]?
		//output angle is contrainted between -45 and +45. 
		//90(default)-/+45 => [45-135]
		//90 - [45-135] => [45-(-45)]
		//why?
		float disp  = steerThrow - abs(steerCenter-outputAngle);
		//distance is in miles.  Not sure why we convert to feet?
		float speed = (distance*5280.l);

		//not sure why one would use disp vs speed?  Is this for turning distance?
		//speed is even a distance.  if we get Less the x feet away and it is smaller 
		//then approachSpeed and turning angle then we us it?

		speed = min(speed, disp)/6.f; //logical speed clamps
		float approachSpeed = manager.getTargetWaypoint().getApproachSpeed();
		speed = min(speed, approachSpeed); //put in target approach speed
		speed = constrain(speed, minFwd, maxFwd);
		
		if(stop) 
			output(0 , steerCenter);
		else     
			output(speed, outputAngle);
	}
}

void updateGPS()
{
	//sim mode doesn't need to update gps (nothing to read)
	//gps.update();

	if(gps.getUpdatedRMC())
	{
		digitalWrite(A8, HIGH);

		gps.clearUpdatedRMC();
		//readIdx = gps.dataIndex();
		location = gps.getLocation();
		waypointUpdated();
		syncHeading();
		positionChanged();
		gpsTime = millis();
		digitalWrite(A8, LOW);
	}
}

void waypointUpdated()
{
	if(manager.numWaypoints() > 0 && !gps.getWarning())
	{
		stop = false;

 		distance = manager.getTargetWaypoint().distanceTo(location);

		if(distance > PointRadius)  
			return;

		if(manager.getTargetIndex() < manager.numWaypoints()-1)
		{
			backWaypoint = manager.getTargetWaypoint();
			manager.advanceTargetIndex();
		}
		else if (manager.loopWaypoints())
		{
			backWaypoint = manager.getTargetWaypoint();
			manager.setTargetIndex(0);
		}
		else
		{
			stop = true;
		}
	}
}

void updateGyro()
{
	float dt = lowFilter.millisSinceUpdate();
	float Gz = -toDeg(mpu.gyroZ());
	lowFilter.update(Gz);
	highFilter.update(Gz-lowFilter.get());

	//cant use gyro for any heading in simulation

	// trueHeading = truncateDegree(trueHeading + dt*(highFilter.get()));

	// if(gpsHalfTime < millis() && gpsHalfTime!=0){
	// 	gyroHalf = trueHeading;
	// 	gpsHalfTime = 0;
	// }
}

void syncHeading()
{
	if(!gps.getWarning() && gps.getCourse()!=0)
	{
		trueHeading = gps.getCourse();
		/*
		if(millis() - gpsTime < 1500) //dont use gyrohalf if it is too old
			trueHeading = truncateDegree(gps.getCourse() + trueHeading - gyroHalf);
		else
			trueHeading = truncateDegree(gps.getCourse());
		gpsHalfTime = millis()+(millis()-gpsTime)/2;*/
	}
	else if(stop) 
	{
		trueHeading = pathHeading;
	}
}

void checkPing()
{

	// //digitalWrite(A8,HIGH);
	// ping[pIter] = getPing(PingPin[pIter]);
	// pIter+=2;
	// pIter = pIter%5;
	// if(ping[pIter] < warn[pIter]) 
	// 	oTime = millis();
	// //digitalWrite(A8,LOW);
}

void extrapPosition()
{
	float dTraveled;
	digitalWrite(A7,HIGH);

	float dT = millis()-nTime;
	if(dT < 1000 && !gps.getWarning()){ //ignore irrational values
		//3600000 = milliseconds per hour
		dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		dTraveled *= (2.l/3.l);//purposly undershoot
		location = location.extrapolate(trueHeading, dTraveled);
	}
	// Serial.println("/==================\\");
	// Serial.println(dTraveled,8);
	// Serial.println(location.degLatitude(),8);
	// Serial.println(location.degLongitude(),8);
	// Serial.println("\\==================\\");
	positionChanged();
	digitalWrite(A7,LOW);
}

void positionChanged()
{
	nTime = millis();
	if (manager.numWaypoints() <= 0) 
		return;

	distance = manager.getTargetWaypoint().distanceTo(location);
	if (backWaypoint.radLongitude() == 0 || distance*5280.l < 25)
	{
		pathHeading = location.headingTo(manager.getTargetWaypoint());	
	} 
	else 
	{
		// find a point along path from backwaypoint to targetwaypoint in which to drive toward (pathHeading)
		//  First we determine how much of our current vector "counts" toard the target (cos).
		//  Whatever is left of distance along that original (optimal) vector is scalled by lineGravity.
		//  LineGravity will scale the point to either heading directory orthogonal to original (optimal) vector or 
		//  directly toward the target waypoint

		//full distance from previous waypoint to target waypoint
		float full  = backWaypoint.distanceTo(manager.getTargetWaypoint());
		//heading (degrees) to the target waypoint from previous waypoint
		float AB    = backWaypoint.headingTo(manager.getTargetWaypoint());
		//heading from where we are at compared to previous waypoint
		float AL    = backWaypoint.headingTo(location);
		//percentage of our current vector. what amount of distance in in direction we should be going.
		float d     = cos(toRad(AL-AB)) * backWaypoint.distanceTo(location);
		//scale remaining distance by linegravity (0,1) then add the 'd' distance. 
		float D     = d + (full-d)*(1.l-lineGravity);
		//find waypoint for this new distance along the previous-to-target path
		Waypoint target = backWaypoint.extrapolate(AB, D);
		//Find the heading to get there from current location
		pathHeading  = location.headingTo(target);

	}

	//update gps sim with heading
	gps.setCourse(pathHeading);
}

void readAccelerometer()
{
	digitalWrite(13,HIGH);
	Ax = mpu.acclX();
	Ay = mpu.acclY();
	Az = mpu.acclZ();
	//atan2 gets angle of x and y vectors
	pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
	//atan2 gets angle of y and z vectors
	roll.update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
	digitalWrite(13,LOW);
}

void reportLocation()
{
	digitalWrite(45,HIGH);

	float voltage  = float(analogRead(67)/1024.l*5.l*10.1f);
	float amperage = float(analogRead(66)/1024.l*5.l*17.0f);
	manager.sendTelem(Protocol::telemetryType(LATITUDE),    location.degLatitude());
	manager.sendTelem(Protocol::telemetryType(LONGITUDE),   location.degLongitude());
	//ba add -90 just to make display correct.  No idea why this works
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading-90);
	manager.sendTelem(Protocol::telemetryType(PITCH),       toDeg(pitch.get())-90);
	manager.sendTelem(Protocol::telemetryType(ROLL),        toDeg(roll.get())-90);
	//manager.sendTelem(Protocol::telemetryType(GROUNDSPEED), RPMtoMPH(encoder::getRPM()));
	manager.sendTelem(Protocol::telemetryType(GROUNDSPEED), gps.getGroundSpeed());
	manager.sendTelem(Protocol::telemetryType(VOLTAGE),     voltage);
	manager.sendTelem(Protocol::telemetryType(VOLTAGE+1),   amperage);

	//TODO define num of ping sensors and loop?
	// manager.sendTelem(Protocol::telemetryType(OBJDETECT), ping[0] );
	// manager.sendTelem(Protocol::telemetryType(OBJDETECT+1), ping[1] );
	// manager.sendTelem(Protocol::telemetryType(OBJDETECT+2), ping[2] );
	// manager.sendTelem(Protocol::telemetryType(OBJDETECT+3), ping[3] );
	// manager.sendTelem(Protocol::telemetryType(OBJDETECT+4), ping[4] );

	digitalWrite(45,LOW);
}

void calibrateGyro(){ //takes one second
	float tmp = 0;
	for(int i=0; i<100; i++)
	{
		float Gz = toDeg(mpu.gyroZ());
		tmp += Gz/100;
		delay(10);
	}
	lowFilter.set(tmp);
}

void output(float mph, uint8_t steer)
{
	//set speed to gps simulator
	gps.setGroundSpeed(mph);

#if useEncoder
	//the one wire encoder would need to feed fabs(mph) to cruise
	//and the output's sign would need to be flipped based on sign(mph)
	if(abs(mph)<0.5f)
	{
		cruise.stop();
	} 
	else 
	{
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

void newPIDparam(float x)
{
	// indexes for cruise control PID settings defined below
	cruisePID = PIDparameters(settings.get(11),
                              settings.get(12),
                              settings.get(13), -90, 90 );
}


void setupSettings()
{
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
	settings.attach(10, 800, &dangerTimeCallback);

	/*GROUNDSETTING index="11" name="Cruise P" min="0" max="+inf" def="0.05"
	 *P term in cruise control PID loop
	 */
	settings.attach(11, 0.05, &newPIDparam);

	/*GROUNDSETTING index="12" name="Cruise I" min="0" max="+inf" def="0.1"
	 *I term in cruise control PID loop
	 */
	settings.attach(12, 0.1, &newPIDparam);

	/*GROUNDSETTING index="13" name="Cruise D" min="0" max="+inf" def="0.0"
	 *D term in cruise control PID loop
	 */
	settings.attach(13, 0.0, &newPIDparam);

	/*GROUNDSETTING index="14" name="Tire Diameter" min="0" max="+inf" def="5.85"
	 *Tire Diameter in inches, used to calculate MPH
	 */
	settings.attach(14, 5.85, callback<float, &tireDiameter>);

	/*GROUNDSETTING index="15" name="Steer Center" min="0" max="180" def="90"
	 *Center point in degrees corresponding to driving straight
	 */
	settings.attach(15, 90, callback<int, &steerCenter>);
}

