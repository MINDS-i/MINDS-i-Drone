#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "MINDS-i-Drone.h"
#include "util/callbackTemplate.h"
#include "version.h"


//=============================================//
//  Defines used to control compile options
//=============================================//

// == Sim mode ==
//#define simMode true

//== encoder ==
#define useEncoder true

//== external logger ==
//#define extLogger true

//=============================================//
//Constants that should never change during 
// driving and never/rarely tuned
//=============================================//

//== pin defines ==
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = {7, 0, 1}; //auto switch, drive, steer
const uint8_t EncoderPin[]= {2/*APM pin 7*/, 3 /*APM pin 6*/};

//== ping sensor vars ==

const float  pAngle[5]   = { 79.27, 36.83, 0.0, -36.83, -79.27};
//about 500 us per inch
int blockLevel[]     = {1000, 1600, 3000, 1600, 1000};
int warnLevel[]     = {2000, 3200, 6000, 3200, 2000};

//== Tire related vars/defines == 

//tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
//hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);


//== steering related ==

//All Headings are Clockwise+, -179 to 180, 0=north
double   pathHeading; 
double   trueHeading;

//test for correcting mechanically caused drift left-right
float steerSkew = 0;

//== hardware related ==

HardwareSerial *commSerial	= &Serial;
Storage<float> *storage	= eeStorage::getInstance();
#ifdef simMode
LEA6H_sim		gps;
#else
LEA6H			gps;
#endif
MPU6000			mpu;


CommManager		manager(commSerial, storage);
Settings		settings(storage);
Waypoint		location(0,0);
Waypoint		backWaypoint(0,0);
HLA				lowFilter (600000, 0);//10 minutes
HLA				highFilter(    10, 0);//10 milliseconds
HLA 			pitch( 100, 0);
HLA 			roll ( 100, 0);
PIDparameters   cruisePID(0,0,0,-90,90);
PIDcontroller   cruise(&cruisePID);
ServoGenerator::Servo servo[3]; //drive, steer, backSteer

//== scheduler, navigation, obstacle, stop times ==

uint32_t uTime = 0, nTime = 0, sTime = 0;
#ifdef simMode
uint32_t simTime = 0;
#endif
uint32_t gpsHalfTime = 0, gpsTime = 0;
//used in accelerometer calculations
int32_t Ax,Ay,Az; 

//== Ping ==

enum PING_HIST {
	PING_CUR = 0,
	PING_LAST,
};

uint16_t ping[5][2] = {20000,20000,20000,20000,20000};
uint8_t  sIter,pIter; //iterators for scheduler and ping


//== gyro ==

double   gyroHalf; //Store Gyro Heading halfway between gps points
double   distance;
boolean  stop = true;
boolean  backDir;

//todo testing
float lastOutputAngle=0;
float lastAngularError=0;


//====================================
// State related variables/enums
//====================================

enum APM_STATES { 
	APM_STATE_INVALID = 0,
	APM_STATE_INIT,
	APM_STATE_SELF_TEST,
	APM_STATE_DRIVE
};

enum DRIVE_STATES {
	DRIVE_STATE_INVALID = 0,
	DRIVE_STATE_STOP,
	DRIVE_STATE_AUTO,
	DRIVE_STATE_RADIO
};

enum AUTO_STATES {
	AUTO_STATE_INVALID = 0,
	AUTO_STATE_FULL,
	AUTO_STATE_AVOID,
	AUTO_STATE_STALLED,
};

//warning assign as flags. Unique bits 1,2,4,8,etc
enum AUTO_STATE_FLAGS {
	AUTO_STATE_FLAGS_NONE=0,
	AUTO_STATE_FLAG_CAUTION=1,
	AUTO_STATE_FLAG_APPROACH=2,
};


//global state vars (maybe move to class at some point?)
uint8_t apmState=APM_STATE_INVALID;
uint8_t driveState=DRIVE_STATE_INVALID;
uint8_t prevDriveState=DRIVE_STATE_INVALID;
uint8_t autoState=AUTO_STATE_INVALID;
uint8_t autoStateFlags=AUTO_STATE_FLAGS_NONE;



//==================================
// Function prototypes
//==================================

void checkPing();
void readAccelerometer();
void reportLocation();
void reportState();
void extrapPosition();
void navigate();
#ifdef simMode
void updateSIM();
#endif

void stateStop();
void stateStart();
void version();

void setupSettings();

//===================================
// Scheduler structures
//===================================

struct schedulerData 
{
	void (*function)(void);
	uint16_t delay;
	uint32_t lastTime;
	boolean enabled;
};

//delay,last scheduled time,enabled
struct schedulerData scheduler[] = 
{
	{extrapPosition,	110	,0		,1},
	{readAccelerometer,	110	,22		,1},
	{reportLocation,	110	,44		,1},
	{reportState,		110	,110	,1},
	{checkPing,			22	,88		,1},
	{navigate,			0	,66		,1},
	#ifdef simMode
	{updateSIM,			500	,100	,1}
	#endif
};

enum SCHEDULED_FUNCTIONS
{
	SCHD_FUNC_EXTRPPOS=0,
	SCHD_FUNC_RDACC,
	SCHD_FUNC_RPRTLOC,
	SCHD_FUNC_RPRTSTATE,
	SCHD_FUNC_CHKPING,
	SCHD_FUNC_NAVIGATE,
	#ifdef simMode	
	SCHD_FUNC_UPDATESIM,
	#endif
};



//==============================================
// Vars stored in Settings 
//
//  These parameters are loaded from eeprom if 
// the code has not been reuploaded since 
// when they were last set.  the dashboard 
// can update records in the storage instance 
// passed into manager.  The callbacks keep 
// them up to date while the code is running
//==============================================

float	lineGravity;
int		steerThrow, steerStyle;
float	steerFactor;
float	minFwd, maxFwd;
int		revThrow;
float	revSpeed;
float	pingWeight;
int		avoidCoastTime, avoidStraightBack, avoidSteerBack; 
float	tireDiameter;
int		steerCenter;
//in miles, margin for error in rover location
float PointRadius  = .0015; 
float approachRadius = .0038;


//===========================
//  Settings callbacks
//===========================

void dangerTimeCallback(float in){ 
	avoidStraightBack = avoidCoastTime+(in/2);
	avoidSteerBack = avoidCoastTime+in; 
}

void pingBlockLevelEdgesCallback(float in){
	blockLevel[0] = blockLevel[4] = (int)in;
}
void pingBlockLevelMiddlesCallback(float in){
	blockLevel[1] = blockLevel[3] = (int)in;
}
void pingBlockLevelCenterCallback(float in){
	blockLevel[2] = (int)in;
}
void pingWarnLevelEdgesCallback(float in){
	warnLevel[0] = warnLevel[4] = (int)in;
}
void pingWarnLevelMiddlesCallback(float in){
	warnLevel[1] = warnLevel[3] = (int)in;
}
void pingWarnLevelCenterCallback(float in){
	warnLevel[2] = (int)in;
}


//===========================
// Utility functions/inlines
//===========================

inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }


//============================
// external Logging functions
//============================


#ifdef extLogger
unsigned int extlogSeqNum=0;

void extLog(const char type[], float val, int format=6)
{
	Serial2.print(extlogSeqNum++);
	Serial2.print(" ");
	Serial2.print(millis());
	Serial2.print(" ");
	Serial2.print(type);
	Serial2.print(": ");
	Serial2.println(val,format);	
}
void extLog(const char type[], const char val[])
{
	Serial2.print(extlogSeqNum++);
	Serial2.print(" ");
	Serial2.print(millis());
	Serial2.print(" ");
	Serial2.print(type);
	Serial2.print(": ");
	Serial2.println(val);
}

void extLog(const char type[], int value)
{
	Serial2.print(extlogSeqNum++);
	Serial2.print(" ");
	Serial2.print(millis());
	Serial2.print(" ");
	Serial2.print(type);
	Serial2.print(": ");
	Serial2.println(value);
}


#else
void extLog(const char type[], float val, int format=6) { return; }
void extLog(const char type[], const char val[]) { return; }
void extLog(const char type[], int value) { return; }

#endif


//=======================================
//  State flag utilities
//=======================================

void setAutoStateFlag(uint8_t flag)
{
	String msg("Setting Flag " + String(flag));
	manager.sendString(msg.c_str());	

	extLog("Setting Flag",flag);	


	autoStateFlags |= flag;
}

void clearAutoStateFlag(uint8_t flag)
{
	String msg("Clearing Flag " + String(flag));
	manager.sendString(msg.c_str());	

	extLog("Clearing Flag",flag);	


	autoStateFlags &= ~flag;
}

bool isSetAutoStateFlag(uint8_t flag)
{
	return ( autoStateFlags & flag ) > 0 ? true : false;
}










//=====================================
//  Main Functions
//=====================================




void setup() 
{

	//commSerial->begin(Protocol::BAUD_RATE);	
	commSerial->begin(57600);

	#ifdef extLogger
	Serial2.begin(115200);
	#endif

	changeAPMState(APM_STATE_INIT);

	setupSettings();


	//initialize vars with sane values
	lastOutputAngle=steerCenter;
	lastAngularError=steerCenter;


	gps.begin();
	mpu.begin();
	
	for(int i=0; i<3; i++) 
		pinMode(LEDpin[i], OUTPUT);
	
	for(int i=0; i<3; i++) 
		servo[i].attach(ServoPin[i]);

	output(0.0f,steerCenter);

	delay(2000);
	calibrateGyro(); //this also takes one second

	APMRadio::setup();
	#if useEncoder
		encoder::begin(EncoderPin[0], EncoderPin[1]);
	#endif
	manager.requestResync();

	uTime = millis();

	
	//add state callbacks
	manager.setStateStopCallback(stateStop);
	manager.setStateStartCallback(stateStart);
	manager.setVersionCallback(version);

	//===   ba testing ===//
	pinMode(A5, OUTPUT);
    pinMode(A6, OUTPUT);

    pinMode(A7, OUTPUT);    
	pinMode(A8, OUTPUT);
	
	pinMode(13, OUTPUT);
	
	#ifdef extLogger
	pinMode(45, OUTPUT);
	#endif

	//=== end ba testing ===//


	changeAPMState(APM_STATE_SELF_TEST);

	//while (gps.getWarning())

	//todo loop while testing systems
	//Maybe do ping sensor check?
	//gps lock test?
	//drive test?
	// short move forward and read accel/gyro, pulse encoder
	// have to include manager.update() in loop
	//maybe move this all down into loop()
	changeAPMState(APM_STATE_DRIVE);
	changeDriveState(DRIVE_STATE_STOP);
	changeAutoState(AUTO_STATE_FULL);
}

void loop()
{
	int i;
	digitalWrite(A5, HIGH);

	//======================================================
	//Functions that don't directely affect the rover.
	//======================================================

	digitalWrite(A6, HIGH);
	manager.update();
	digitalWrite(A6, LOW);

	digitalWrite(A6, HIGH);
	updateGPS();
	digitalWrite(A6, LOW);

	// digitalWrite(A6, HIGH);
	// updateGyro();
	// digitalWrite(A6, LOW);

	//======================================================
	//Functions that are run in various states and at 
	//various freqency
	//======================================================
	
	//Was thinking of trying to only run one task per loop but
	//not sure if needed.  Keep this around for a bit to decide
	// //find the best task to run this time around
	// for (i=0;i<sizeof(scheduler)/sizeof(*scheduler);i++)
	// {
	// 	unsigned int offset=0;
	// 	int best_task=-1;

	// 	if (scheduler[i].enabled && scheduler[i].lastTime <= millis())
	// 	{
	// 		if (millis - scheduler[i].lastTime > offset)
	// 		{
	// 			offset=lastTime;
	// 			best_task=i;
	// 		}
	// 	}
	// }

	for (i=0;i<sizeof(scheduler)/sizeof(*scheduler);i++)
	{
		digitalWrite(A6,HIGH);
		if(scheduler[i].enabled && scheduler[i].lastTime <= millis())
		{
			scheduler[i].lastTime = scheduler[i].delay+millis();
			scheduler[i].function();			
			////digitalWrite(A5, LOW);

			//ba- think over.  Issue is that the same *faster* tasks may run without
			//letting others *slower* tasks run 
			////only allow one scheduler task per loop
			////so that schedulers don't significantly impact main loop freq
			//break;

		}
		digitalWrite(A6,LOW);

	}



	digitalWrite(A5,LOW);
}



void changeAPMState(uint8_t newState)
{
	//ignore if we are not changing state
	if (apmState == newState)
		return;

	switch (apmState)
	{
		case APM_STATE_INVALID:
			//only allow transition from invalid to init
			switch(newState)
			{
				case APM_STATE_INIT:
					manager.sendString("APM State: Init");

					extLog("APM State","Init");

					apmState=newState;				
					break;
				default:
					apmState=APM_STATE_INVALID;					
					break;
			}
			break;		
		case APM_STATE_INIT:
			//only allow transition from invalid to self-test
			switch(newState)
			{
				case APM_STATE_SELF_TEST:
					manager.sendString("APM State: Self Test");

					extLog("APM State","Self Test");

					apmState=newState;
					break;
				default:
					apmState=APM_STATE_INVALID;
					break;
			}
			break;
		case APM_STATE_SELF_TEST:
			//only allow transition from invalid to drive
			switch(newState)
			{
				case APM_STATE_DRIVE:
					manager.sendString("APM State: Drive");

					extLog("APM State","Drive");

					apmState=newState;
					break;
				default:
					apmState=APM_STATE_INVALID;
					break;
			}		
			break;			
		case APM_STATE_DRIVE:
			//no case so far for drive-> any other state
			apmState=APM_STATE_INVALID;			
			break;
	}

}


void changeDriveState(uint8_t newState)
{
	

	//ignore if we are not changing state
	if (driveState == newState)
		return;
	

	switch (newState)
	{
		case DRIVE_STATE_INVALID:
			//Ignore change to invalid state because it is invalid
			break;

		case DRIVE_STATE_STOP:

			switch(driveState)
			{

				case DRIVE_STATE_INVALID:
				case DRIVE_STATE_AUTO:
				case DRIVE_STATE_RADIO:
					manager.sendString("Drive State: Stop");

					extLog("Drive State","Stop");


					#ifdef simMode
					gps.setGroundSpeed(0);
					#else
					output(0,steerCenter);
					#endif

					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RDACC].enabled 		= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;
					#ifdef simMode
					scheduler[SCHD_FUNC_UPDATESIM].enabled 	= true;
					#endif 
					
					driveState=newState;


					if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION))
						clearAutoStateFlag(AUTO_STATE_FLAG_CAUTION);
					break;
			}

			break;

		case DRIVE_STATE_AUTO:
			switch(driveState)
			{
				case DRIVE_STATE_INVALID:
				case DRIVE_STATE_STOP:
				case DRIVE_STATE_RADIO:
					manager.sendString("Drive State: Auto");

					extLog("Drive State","Auto");


					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= true;
					scheduler[SCHD_FUNC_RDACC].enabled 		= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;
					
					#ifdef simMode
					scheduler[SCHD_FUNC_UPDATESIM].enabled 	= true;
					simTime=millis();
					#endif


					nTime = millis();

					
					driveState=newState;

					break;
			}
			break;

		#ifndef simMode
		case DRIVE_STATE_RADIO:
			switch(driveState)
			{
				case DRIVE_STATE_INVALID:
				case DRIVE_STATE_STOP:
				case DRIVE_STATE_AUTO:
					manager.sendString("Drive State: Radio");

					extLog("Drive State","Radio");


					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RDACC].enabled 		= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;

					//Only radio state sets previous drivestate
					//so it knows what to revert back to
					prevDriveState=driveState;
					driveState=newState;
					break;

			}
			break;
		#endif
	}
}



void changeAutoState(uint8_t newState)
{
	//ignore if we are not changing state
	if (autoState == newState)
		return;	


	switch (newState)
	{
		case AUTO_STATE_INVALID:
			//Ignore change to invalid state because it is invalid
			break;

		case AUTO_STATE_FULL:
			switch(autoState)
			{
				case AUTO_STATE_INVALID:
				case AUTO_STATE_AVOID:
					manager.sendString("Auto State: Full");

					extLog("Auto State","Full");


					sTime = 0;
					

					//change speed
					autoState=newState;
					break;
				case AUTO_STATE_STALLED:
					//don't transition from stalled
					//do we allow a way of getting out of stall other then power cycle?
					break;
			}
			break;

		case AUTO_STATE_AVOID:
			switch(autoState)
			{
				case AUTO_STATE_INVALID:
				case AUTO_STATE_FULL:
					manager.sendString("Auto State: Avoid");

					extLog("Auto State","Avoid");


					//If we had set caution flag then entered avoid we need to clear
					//might consider leaving in caution but for now...
					clearAutoStateFlag(AUTO_STATE_FLAG_CAUTION);

					//determine backup angle
					output(0, steerCenter);
					sTime = millis();

					//Maybe find weight of left vs right (rather then just edge)
					backDir = ping[0][PING_CUR]<ping[4][PING_CUR];
					//change speed 

					autoState=newState;
					break;
				case AUTO_STATE_STALLED:
					//don't transition from stalled
					//do we allow a way of getting out of stall other then power cycle?
					break;

			}

			break;
		case AUTO_STATE_STALLED:
			switch(autoState)
			{
				case AUTO_STATE_INVALID:
				case AUTO_STATE_FULL:
				case AUTO_STATE_AVOID:
					manager.sendString("Auto State: Stalled");

					extLog("Auto State","Stalled");

					autoState=newState;
					//stop motors
					break;
				
			}
			break;

	}

}



#ifdef simMode
void updateSIM()
{	
	//todo Set some randomness to path?
	float steering_error=0;


	extLog("updateSIM","=====");


	//in sim always act like we are still getting gps updates
	gps.setUpdatedRMC();	

	gps.setCourse(gps.getLocation().headingTo(manager.getTargetWaypoint())+steering_error);

	//Only move if in auto mode
	//Stalled is simulated but still handle
	if (driveState == DRIVE_STATE_AUTO && autoState != AUTO_STATE_STALLED)
	{
		float dT = millis()-simTime;

		//3600000 = milliseconds per hour
		float dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		
	
		//todo.  Add Some kind of randomness?
		Waypoint newLocation = location.extrapolate(gps.getCourse(), dTraveled);
	
		gps.setLatitude(newLocation.degLatitude());
		//Serial.println(newLocation.degLatitude(),8);
		gps.setLongitude(newLocation.degLongitude());
		//Serial.println(newLocation.degLongitude(),8);

		simTime=millis();
		
	}
	
	
}
#endif

void navigate()
{
	float   mph = ((APMRadio::get(RadioPin[1])-90) / 90.f)*maxFwd;
	uint8_t steer = APMRadio::get(RadioPin[2]);

		
	if (abs(steer-steerCenter) > 5 || fabs(mph)>0.8f ) 
	{						
		changeDriveState(DRIVE_STATE_RADIO);

		output(mph, steer);
	}
	else
	{	
		if (driveState == DRIVE_STATE_RADIO )
			changeDriveState(prevDriveState);
	}


	if (driveState == DRIVE_STATE_AUTO)
	{


		if ( autoState == AUTO_STATE_AVOID)
		{

			//The below is in a specific order to accommodate the flow
			//of time.  The first check is the furthest in time and working
			//toward the closest.		

			
			if ( millis() > sTime+avoidSteerBack)
			{
				changeAutoState(AUTO_STATE_FULL);
			}		
			else if ( millis() > sTime+avoidStraightBack )
			{
				if(backDir) 
					output(revSpeed, steerCenter-revThrow);
				else 		
					output(revSpeed, steerCenter+revThrow);
			}		
			else if( millis() > sTime+avoidCoastTime )
			{			
				output(revSpeed, steerCenter);
			}

		} 	
		else if (autoState == AUTO_STATE_FULL)
		{
			//drive based on pathHeading and side ping sensors

			float x,y;
			float approachSpeed;
			float angularError;
			float outputAngle;

			//####################//
			// Angle calculations //
			//####################//


			//difference between pathheading (based on waypoints (previous and current))
			//error can only be as much as 180 degrees off (opposite directions).
			angularError = truncateDegree(pathHeading - trueHeading);

			extLog("angularError",angularError,6);

			//angularCorrection = (lastAngularError - angularError) - lastOutputAngle - lastAngularCorrection;
			
			switch(steerStyle)
			{
				case 0:
					//not sure.  Ratio of opposite/adjacent multiplied by steering throw
					//convert from degrees to radians for use with atan
					outputAngle = atan( angularError*PI/180.l )*(2*steerThrow/PI);
					break;
				case 1:
					//get squared of percentage of error from 180.  x^2 function where 180=1.00
					outputAngle = ((angularError/180.l)*(angularError/180.l));

					//above percentage of the full steerthrow 
					if (isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH) == false )
						outputAngle *=(2.l*steerThrow);
					else
						outputAngle *=(6.l*steerThrow);

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


			//scale angle (default of 1.5) This is to account for the 45 deg steer really is 30 deg
			outputAngle *= steerFactor;

			//possible correction in steering (pulling left or right)
			outputAngle += steerSkew;

			extLog("nav outputAngle post err correct",outputAngle);

			//find x and y component of output angle
			x = cos(toRad(outputAngle));
			y = sin(toRad(outputAngle));

			//Not 100 sure:  Generally using pings sensor values to adjust output angle
			//modify x and y based on what pings are seeing		
			for(int i=0; i<5; i++)
			{
				//temp is some percentage (at ping of 1400 would mean tmp==1)
				float tmp = ping[i][PING_CUR]/pingWeight;
				//value is squared?
				tmp *= tmp;
				//add to x,y based on set angle of sensor inverse scaled of percentage
				x += cos(toRad(pAngle[i]))/tmp;
				y += sin(toRad(pAngle[i]))/tmp;
			}


			extLog("nav ping X adjust",x,6);
			extLog("nav ping Y adjust",y,6);
			extLog("nav outputAngle ping",toDeg(atan2(y,x)),6);


			//determine angle based on x,y then adjust from steering center ( 90 )
			outputAngle = toDeg(atan2(y,x))+steerCenter;
			//Can't steer more then throw
			outputAngle = constrain(outputAngle,
					  				double(steerCenter-steerThrow),
				 	  				double(steerCenter+steerThrow));

			//outputAngle is [45,135] steerCenter == 90 and steerThrow == 45

			//####################//
			// Speed calculations //
			//####################//

			//90-45 => 45
			//90-135 => 45
			//45 - [0,45]
			//disp is between 45 and 0
			//disp is largest the closest to steerCenter
			//If we are turning hard with disp of 43,44 that could be low enough
			//to set speed to less the maxFwd (assuming defaults)
			//why?  Basically if we turn too much or get within a few feet or target
			//the speed is reduced.
			//turn hard (45) means disp is close to 0 
			//get within a couple of feet from target the speed is reduced

			float disp  = steerThrow - abs(steerCenter-outputAngle);
			//distance is in miles.  Not sure why we convert to feet?
			//this isn't really speed
			//get slower and slower once we are less then maxfwd away?
			float speed = (distance*5280.l);

			//not sure why one would use disp vs speed?  Is this for turning distance?
			//speed is even a distance.  if we get Less the x feet away and it is smaller 
			//then approachSpeed and turning angle then we us it?

			speed = min(speed, disp)/6.f; //logical speed clamps
			approachSpeed = manager.getTargetWaypoint().getApproachSpeed();
			speed = min(speed, approachSpeed); //put in target approach speed

			speed = constrain(speed, minFwd, maxFwd);

			//deal with speed reduction for caution and approach
			if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION) || isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH))
			{
				speed = speed/2;
			}
			
			output(speed, outputAngle);

			lastOutputAngle=outputAngle;
			lastAngularError=angularError;
		}
	}//end drive state auto check

}

void extrapPosition()
{
	float dTraveled;
	//digitalWrite(A7,HIGH);

	float dT = millis()-nTime;
	if(dT < 1000 && !gps.getWarning()){ //ignore irrational values
		//3600000 = milliseconds per hour
		dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		dTraveled *= (2.l/3.l);//purposly undershoot
		location = location.extrapolate(trueHeading, dTraveled);


		extLog("extrap lat",location.degLatitude(),6);
		extLog("extrap long",location.degLongitude(),6);


	}

	positionChanged();
	//digitalWrite(A7,LOW);

}


void updateGPS()
{
	
	gps.update();

	if(gps.getUpdatedRMC())
	{
		digitalWrite(A8, HIGH);

		//todo is this not used in sim?
		gps.clearUpdatedRMC();
		
		location = gps.getLocation();


		extLog("GPS lat",location.degLatitude(),6);
		extLog("GPS long",location.degLongitude(),6);


		if (driveState == DRIVE_STATE_AUTO)
		{
			waypointUpdated();
			syncHeading();
			positionChanged();
			// currently not used
			//gpsTime = millis();
		}
		//digitalWrite(A8, LOW);
	}
}

void waypointUpdated()
{

	distance = manager.getTargetWaypoint().distanceTo(location);

	extLog("Target lat", manager.getTargetWaypoint().degLatitude(),6);
	extLog("Target long", manager.getTargetWaypoint().degLongitude(),6);

	//approach flag gets locked in until we we move to another waypoint or stop.
	if (distance < approachRadius && isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH) == false )
		setAutoStateFlag(AUTO_STATE_FLAG_APPROACH);

	if(distance <= PointRadius)
	{  

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
			changeDriveState(DRIVE_STATE_STOP);
		}

		clearAutoStateFlag(AUTO_STATE_FLAG_APPROACH);
	}
}

void syncHeading()
{
	if(!gps.getWarning() && gps.getCourse() != 0)
	{
		trueHeading = gps.getCourse();

		extLog("SH trueHeading", trueHeading,6);

		/*
		if(millis() - gpsTime < 1500) //dont use gyrohalf if it is too old
			trueHeading = truncateDegree(gps.getCourse() + trueHeading - gyroHalf);
		else
			trueHeading = truncateDegree(gps.getCourse());
		gpsHalfTime = millis()+(millis()-gpsTime)/2;*/
	}
	
}




void positionChanged()
{
	nTime = millis();


	distance = manager.getTargetWaypoint().distanceTo(location);

	extLog("PC distance",distance,6);
	extLog("PC lat",location.degLatitude(),6);
	extLog("PC long",location.degLongitude(),6);

	if (backWaypoint.radLongitude() == 0 || distance*5280.l < 25)
	{
		pathHeading = location.headingTo(manager.getTargetWaypoint());	
	} 
	else 
	{
		// find a point along path from backwaypoint to targetwaypoint in which to drive toward (pathHeading)
		//  First we determine how much of our current vector "counts" toward the target (cos).
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

	extLog("PC PathHeading",pathHeading,6);

}

void checkPing()
{

	//digitalWrite(A8,HIGH);

	//copy cur value into last
	ping[pIter][PING_LAST] = ping[pIter][PING_CUR];

	#ifndef simMode
    ping[pIter][PING_CUR] = getPing(PingPin[pIter]);
	#else
	pinMode(PingPin[pIter], INPUT);    
	ping[pIter][PING_CUR] = map(analogRead(PingPin[pIter]),0,1023,0,10000);
	#endif

	//iteration goes 0 2 4 1 3 <repeats>
	pIter+=2;
	pIter = pIter%5;

	
	if ( driveState == DRIVE_STATE_AUTO && autoState == AUTO_STATE_FULL)
	{	

		if(ping[pIter][PING_CUR] < blockLevel[pIter] && ping[pIter][PING_LAST] < blockLevel[pIter]) 
		{
			changeAutoState(AUTO_STATE_AVOID);
		}
		else
		{
			
			if (ping[pIter][PING_CUR] < warnLevel[pIter] && ping[pIter][PING_LAST] < warnLevel[pIter])
			{
				if ( !isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION))
				{
					setAutoStateFlag(AUTO_STATE_FLAG_CAUTION);	

					//extLog("Caution flag: ","1");

				}
			}
			else
			{		
				if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION))
				{

					clearAutoStateFlag(AUTO_STATE_FLAG_CAUTION);

					//extLog("Caution flag: ","0");

				}
			}
		}
	}


	//digitalWrite(A8,LOW);
}

void output(float mph, uint8_t steer)
{
	#ifdef simMode
	//set speed to gps simulator (simulator only)
	gps.setGroundSpeed(mph);
	#endif


	extLog("mph",mph,6);
	extLog("steer",steer,6);


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

void updateGyro()
{
	float dt = lowFilter.millisSinceUpdate();
	float Gz = -toDeg(mpu.gyroZ());
	lowFilter.update(Gz);
	highFilter.update(Gz-lowFilter.get());

	//todo think about simulating the gyro?
	#ifndef simMode	
	trueHeading = truncateDegree(trueHeading + dt*(highFilter.get()));


	extLog("UG trueHeading", trueHeading,6);


	if(gpsHalfTime < millis() && gpsHalfTime!=0){
		gyroHalf = trueHeading;
		gpsHalfTime = 0;
	}
	#endif
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
	
	#ifdef simMode
	//ba add -90 just to make display correct.  No idea why this works
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading-90);	
	#else	
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading);
	#endif

	manager.sendTelem(Protocol::telemetryType(PITCH),       toDeg(pitch.get())-90);
	manager.sendTelem(Protocol::telemetryType(ROLL),        toDeg(roll.get())-90);

	#ifdef simMode
	manager.sendTelem(Protocol::telemetryType(GROUNDSPEED), gps.getGroundSpeed());
	#else
	manager.sendTelem(Protocol::telemetryType(GROUNDSPEED), RPMtoMPH(encoder::getRPM()));
	#endif
	manager.sendTelem(Protocol::telemetryType(VOLTAGE),     voltage);
	manager.sendTelem(Protocol::telemetryType(VOLTAGE+1),   amperage);

	//TODO define num of ping sensors and loop?
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_SONIC),0, ping[0][PING_CUR] );
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_SONIC),1, ping[1][PING_CUR] );
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_SONIC),2, ping[2][PING_CUR] );
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_SONIC),3, ping[3][PING_CUR] );
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_SONIC),4, ping[4][PING_CUR] );

	digitalWrite(45,LOW);
}

void reportState()
{
	digitalWrite(45,HIGH);
	
	manager.sendState(Protocol::stateType(APM_STATE),apmState);
	manager.sendState(Protocol::stateType(DRIVE_STATE),driveState);
	manager.sendState(Protocol::stateType(AUTO_STATE),autoState);
	manager.sendState(Protocol::stateType(AUTO_FLAGS),autoStateFlags);
	manager.sendState(Protocol::stateType(GPS_STATE),gps.getWarning());


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



void newPIDparam(float x)
{
	// indexes for cruise control PID settings defined below
	cruisePID = PIDparameters(settings.get(11),
                              settings.get(12),
                              settings.get(13), -90, 90 );
}

void stateStop()
{	
	changeDriveState(DRIVE_STATE_STOP);
}

void stateStart()
{
	changeDriveState(DRIVE_STATE_AUTO);
}

void version()
{
	manager.sendVersion(version_major, version_minor, version_rev);
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

	/*GROUNDSETTING index="3" name="steer scalar" min="0" max="+inf" def="1.5"
	 *Multiplier that determines how aggressively to steer
	 */
	settings.attach(3, 1.5, callback<float, &steerFactor>);

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
	settings.attach(9, 1500, callback<int, &avoidCoastTime>);

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

//Target radi settings


	/*GROUNDSETTING index="16" name="Waypoint acheived radius in miles" min="0" max=".003" def=".0015"
	 * Radius centered at waypoint where target is determined to be meet
	 */
	settings.attach(16, .0015, callback<float,&PointRadius>);

	/*GROUNDSETTING index="17" name="Approach radius" min="0" max=".0076" def=".0038"
	 * Radius cneter at waypoint where the approach flag is set
	 */
	settings.attach(17, .0038, callback<float,&approachRadius>);


//skew

	/*GROUNDSETTING index="57" name="steer skew" min="-45" max="45" def="0"
	 *
	 */
	settings.attach(57, 0, callback<float,&steerSkew>);

//ping 

	/*GROUNDSETTING index="58" name="Avoid Ping value Edges" min="500" max="10000" def="1000"
	 *
	 */
	settings.attach(58, 1000, &pingBlockLevelEdgesCallback);

	/*GROUNDSETTING index="59" name="Avoid Ping value Middles" min="500" max="10000" def="1600"
	 *
	 */
	settings.attach(59, 1600, &pingBlockLevelMiddlesCallback);

	/*GROUNDSETTING index="60" name="Avoid Ping value center" min="500" max="10000" def="3000"
	 *
	 */
	settings.attach(60, 3000, &pingBlockLevelCenterCallback);

	/*GROUNDSETTING index="61" name="Warn Ping value Edges" min="500" max="10000" def="2000"
	 *
	 */
	settings.attach(61, 2000, &pingWarnLevelEdgesCallback);

	/*GROUNDSETTING index="62" name="Warn Ping value Middles" min="500" max="10000" def="3200"
	 *
	 */
	settings.attach(62, 3200, &pingWarnLevelMiddlesCallback);

	/*GROUNDSETTING index="63" name="Warn Ping value center" min="500" max="10000" def="6000"
	 *
	 */
	settings.attach(63, 6000, &pingWarnLevelCenterCallback);


	

}

