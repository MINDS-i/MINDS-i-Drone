#include <StateMsgs.h>
#include <Util.h>
#include <MINDSiDebugger.h>
#include <RadioMsgs.h>
#include <SensorMsgs.h>
#include <NavigationMsgs.h>
#include <PositionMsgs.h>
#include <OrientationMsgs.h>
#include <AsciiMsgs.h>
#include <VersionMsgs.h>

#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "MINDS-i-Drone.h"
#include "util/callbackTemplate.h"
#include "version.h"

#define M_DEBUG2  //comment out to disable debugger

#ifdef M_DEBUG2
  #include "MINDSiDebugger.h"
  MINDSiDebugger debugger;
#endif

#define MIN_GOOD_HEADINGS 5 
#define HEADING_LOCK_RANGE 10
bool kalman_heading = true;
float k_heading = 0;
bool heading_lock = false;
bool new_gps = false;
bool driving_straight = false;
double ratio_adjustment = 0.1;
//=============================================//
//  Defines used to control compile options
//=============================================//

// == Sim mode ==
//#define simMode true

//== encoder ==
#define useEncoder true

//==== Peripheral HW vars ====/
bool isBumperEnabled = false;

//=============================================//
//Constants that should never change during 
// driving and never/rarely tuned
//=============================================//

//== pin defines ==
const uint8_t VoltagePin  = 67;
const uint8_t LEDpin[]    = {25, 26, 27}; //blue, yellow, red
const uint8_t PingPin[]	  = {A0, A1, A2, A3, A4}; //left to right
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
uint8_t RadioChannel[]  = {1, 0, 2}; //see enum 
const uint8_t EncoderPin[]= {2/*APM pin 7*/, 3 /*APM pin 6*/};



//== radio controller enums ==//
enum RadioChannelTypes{  RADIO_FAILSAFE = 0, RADIO_STEER = 1, RADIO_THROTTLE = 2};
enum RadioControllerTypes { RC_TGY_IA6B = 0, RC_HOBBY_KING_GT2B = 1 };


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

#define MAN_MAX_FWD (5) 

//== steering related ==

//All Headings are Clockwise+, -179 to 180, 0=north
double   pathHeading; 
double   headingError; 
double   crosstrackError; 
double   trueHeading;
double 	 scOutputAngle;
double 	 trueOutputAngle;

//test for correcting mechanically caused drift left-right
int8_t steerSkew = 0;

int8_t turnAroundDir=0;

//== hardware related ==

HardwareSerial *commSerial	= &Serial;
Storage<float> *storage	= eeStorage::getInstance();
#ifdef simMode
LEA6H_sim		gps;
#else
LEA6H			gps;
#endif

//MPU6000			mpu;
MPU6000_DMP		mpudmp;

bumper bumperSensor;

CommManager		manager(commSerial, storage);
Settings		settings(storage);
Waypoint		location(0,0);
Waypoint		backWaypoint(0,0);
PIDparameters   cruisePID(0,0,0,-90,90);
PIDcontroller   cruise(&cruisePID);
ServoGenerator::Servo servo[3]; //drive, steer, backSteer

//== scheduler, navigation, obstacle, stop times ==

uint32_t nTime = 0, sTime = 0;
#ifdef simMode
uint32_t simTime = 0;
#endif

//== Ping ==
enum PING_HIST {
	PING_CUR = 0,
	PING_LAST,
};

uint16_t ping[5][2] = {20000,20000,20000,20000,20000};
uint8_t  pIter; //iterators for scheduler and ping

//== Power ==
//LOW_VOLTAGE_CUTOFF should be 6.1 or greater as some power
//modules may not correctly report values below 6.1V
#define LOW_VOLTAGE_CUTOFF 6.1 //volts; should always be >= 6.1 to assure 
#define LOW_VOLTAGE_TIME 5 //seconds
#define RESET_VOLTAGE_THRESHOLD 7.2 //volts
#define RESET_VOLTAGE_TIME 1 //seconds
float voltage  = -1.0;
float amperage = -1.0;

//== mpu6000 ==//
float euler_z_offset=0;
float last_euler_z=0;
float cur_euler_z=0;

//== gyro ==
double   gyroHalf; //Store Gyro Heading halfway between gps points
double   distance;
boolean  stop = true;
int8_t  backDir;

//== radio tx/rx
uint8_t radioControllerDev = RC_TGY_IA6B;
bool radioFailsafeEnabled = false;

float k_cross_track = 0.05;
float k_yaw = -0.3;
float max_cross_track_error = 2.0;

uint8_t radioFailsafeCount=0;
#define RADIO_FAILURE_COUNT_MAX 10

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
	DRIVE_STATE_RADIO,
	DRIVE_STATE_LOW_VOLTAGE_STOP,
	DRIVE_STATE_LOW_VOLTAGE_RESTART,
	DRIVE_STATE_RADIO_FAILSAFE
};

enum AUTO_STATES {
	AUTO_STATE_INVALID = 0,
	AUTO_STATE_FULL,
	AUTO_STATE_AVOID,
	AUTO_STATE_STALLED
};

enum AVOID_STATES {
	AVOID_STATE_ENTER = 0,
	AVOID_STATE_FWD_BRAKE,
	AVOID_STATE_STRAIGHTBACK,
	AVOID_STATE_STEER,
	AVOID_STATE_REV_BRAKE,
	AVOID_STATE_DONE,
	AVOID_STATE_NUM_STATES
};

const char AVOID_STATES_STRING[AVOID_STATE_NUM_STATES][32] = 
{
	"Avoid State Enter",
	"Avoid State Forward Brake",	
	"Avoid State Straightback",
	"Avoid State Steer",
	"Avoid State Reverse Brake",
	"Avoid State Done"
};

//warning assign as flags. Unique bits 1,2,4,8,etc
enum AUTO_STATE_FLAGS {
	AUTO_STATE_FLAGS_NONE		=0x00,
	AUTO_STATE_FLAG_CAUTION		=0x01,
	AUTO_STATE_FLAG_APPROACH	=0x02,
	AUTO_STATE_FLAG_TURNAROUND	=0x04,
};


//global state vars (maybe move to class at some point?)
uint8_t apmState=APM_STATE_INVALID;
uint8_t driveState=DRIVE_STATE_INVALID;
uint8_t prevDriveState=DRIVE_STATE_INVALID;
uint8_t autoState=AUTO_STATE_INVALID;
uint8_t autoStateFlags=AUTO_STATE_FLAGS_NONE;
uint8_t avoidState=AVOID_STATE_DONE;



//==================================
// Function prototypes
//==================================

void checkPing();
void checkBumperSensor();
void checkVoltage();
void reportLocation();
void reportState();
void extrapPosition();
void navigate();
#ifdef simMode
void updateSIM();
#endif

void stateStop();
void stateStart();
void bumperDisable();
void bumperEnable();
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
	{reportLocation,	110	,44		,1},
	{reportState,		110	,110	,1},
	{checkPing,			22	,88		,1},
	{checkBumperSensor, 80	,55		,1},
	{checkVoltage, 		100 ,66		,1},
	//{navigate,			0	,66		,1},
	#ifdef simMode
	{updateSIM,			500	,100	,1}
	#endif
};

enum SCHEDULED_FUNCTIONS
{
	SCHD_FUNC_EXTRPPOS=0,
	SCHD_FUNC_RPRTLOC,
	SCHD_FUNC_RPRTSTATE,
	SCHD_FUNC_CHKPING,
	SCHD_FUNC_BUMPSENSOR,
	SCHD_FUNC_CHKVOLTAGE,
	//SCHD_FUNC_NAVIGATE,
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
int   lookAheadDist = 25; //feet
int		steerThrow = 45;
int   steerStyle = 2;
float	steerFactor = 1.5;
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
	avoidStraightBack = in;
	avoidSteerBack = in; 
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


	//todo? Do we allow disable of failsafe after entering failsafe to clear it?
void radioFailsafe(float in)
{
	String msg("Radio Failsafe " + String(in));
	manager.sendString(msg.c_str());
  if ((int)in != 0)
  {
  	radioFailsafeEnabled=true;
		String msg("Radio Failsafe: Enabled");
		manager.sendString(msg.c_str());
	}
	else
	{
  	radioFailsafeEnabled=false;
		String msg("Radio Failsafe: Disabled");
		manager.sendString(msg.c_str());  	
	}

}

void radioController(float in){
	radioControllerDev=(uint8_t)in;
	switch((int)in)
	{
	  case RC_TGY_IA6B: 
	    RadioChannel[RADIO_THROTTLE] = 1;
	    RadioChannel[RADIO_STEER] = 0;
	    RadioChannel[RADIO_FAILSAFE] = 2; 
	    break;
	  case RC_HOBBY_KING_GT2B:
	    RadioChannel[RADIO_THROTTLE] = 0;
	    RadioChannel[RADIO_STEER] = 1;
	    RadioChannel[RADIO_FAILSAFE] = 2;
	    break;
	  default:
	    //error.  Ignore?
	    break;
	}

}

//testing
void triggerGyroSyncNorth(float in){
	euler_z_offset = mpudmp.getEulerZ();
}


//===========================
// Utility functions/inlines
//===========================

inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }



//=======================================
//  State flag utilities
//=======================================

void setAutoStateFlag(uint8_t flag)
{
	autoStateFlags |= flag;
}

void clearAutoStateFlag(uint8_t flag)
{
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

	//setup pins for debugging
	pinMode(13, OUTPUT);
	pinMode(A7,OUTPUT);
	pinMode(A8,OUTPUT);
	pinMode(45,OUTPUT);

	
	commSerial->begin(Protocol::BAUD_RATE);	
	//commSerial->begin(57600);


	changeAPMState(APM_STATE_INIT);
	setupSettings();

	gps.begin();

	bumperSensor.begin(A6,A5);

	//*important* disable cs on Pressure sensor
  	// that is on same spi bus 
  	// (mpu6000 driver did this in the drivers begin())
  	pinMode(40, OUTPUT);
  	digitalWrite(40, HIGH);   
	mpudmp.begin();
	
	for(int i=0; i<3; i++) 
		pinMode(LEDpin[i], OUTPUT);
	
	for(int i=0; i<3; i++) 
		servo[i].attach(ServoPin[i]);

	output(0.0f,steerCenter);

	delay(2000);
	//calibrateGyro(); //this also takes one second

	APMRadio::setup();
	#if useEncoder
		encoder::begin(EncoderPin[0], EncoderPin[1]);
	#endif
	manager.requestResync();


	//add state callbacks
	manager.setStateStopCallback(stateStop);
	manager.setStateStartCallback(stateStart);
  manager.setBumperDisableCallback(bumperDisable);
  manager.setBumperEnableCallback(bumperEnable);
  manager.setSettingsResetCallback(setDefaultSettings);
	manager.setVersionCallback(version);


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

//this all needs to be cleaned up after it works (hopefully)
	for (uint8_t i=0;i<10;i++)
	{
		updateGyro();
		manager.update();
		delay(10);
	}

	euler_z_offset = mpudmp.getEulerZ();


	changeAPMState(APM_STATE_SELF_TEST);

	checkVoltage();
}



void loop()
{
	int i;


	digitalWrite(45,HIGH);



	//======================================================
	//Functions that don't directely affect the rover.
	//======================================================

	manager.update();

	checkRadioFailsafe();

	updateGPS();
	updateGyro();
	bumperSensor.update();

	//======================================================
	//Functions that are run in various states and at 
	//various freqency
	//======================================================
	
	///TODO: Think about a more robust scheduler.  More then one task per main loop.

	for (i=0;i<sizeof(scheduler)/sizeof(*scheduler);i++)
	{
		digitalWrite(A7,HIGH);
		if(scheduler[i].enabled && scheduler[i].lastTime <= millis())
		{
			scheduler[i].lastTime = scheduler[i].delay+millis();
			scheduler[i].function();			

			//ba- think over.  Issue is that the same *faster* tasks may run without
			//letting others *slower* tasks run 
			////only allow one scheduler task per loop
			////so that schedulers don't significantly impact main loop freq
			//break;

		}
		digitalWrite(A7,LOW);

	}

	//run navigate all the time
	navigate();

	digitalWrite(45,LOW);

}

void checkRadioFailsafe()
{
	if (radioFailsafeEnabled == false)
		return;

	if (driveState != DRIVE_STATE_RADIO_FAILSAFE )
	{
		
		if(APMRadio::get(RadioChannel[RADIO_FAILSAFE]) == 0) 
	  {
			// radio disconnect failsafe
			radioFailsafeCount++;

			if (radioFailsafeCount >= RADIO_FAILURE_COUNT_MAX)
			{
				changeDriveState(DRIVE_STATE_RADIO_FAILSAFE);
			}
		}
		else
		{
			radioFailsafeCount=0;
		}
	}
	else //DRIVE_STATE_RADIO_FAILSAFE
	{
		if(APMRadio::get(RadioChannel[RADIO_FAILSAFE]) != 0) 
	  {
			// radio reconnected
			radioFailsafeCount++;

			if (radioFailsafeCount >= RADIO_FAILURE_COUNT_MAX)
			{
				changeDriveState(DRIVE_STATE_STOP);
			}
		}
		else
		{
			radioFailsafeCount=0;
		}

	}
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

	//ignore if drive state is low battery
	if (driveState == DRIVE_STATE_LOW_VOLTAGE_STOP && newState != DRIVE_STATE_LOW_VOLTAGE_RESTART) {
		return;
	}

	switch (newState)
	{
		case DRIVE_STATE_INVALID:
			//Ignore change to invalid state because it is invalid
			break;

		case DRIVE_STATE_LOW_VOLTAGE_RESTART:
		case DRIVE_STATE_STOP:

			switch(driveState)
			{

				case DRIVE_STATE_LOW_VOLTAGE_STOP:
				case DRIVE_STATE_INVALID:
				case DRIVE_STATE_AUTO:
				case DRIVE_STATE_RADIO:
				case DRIVE_STATE_RADIO_FAILSAFE:
					manager.sendString("Drive State: Stop");


					#ifdef simMode
					gps.setGroundSpeed(0);
					#else
					output(0,steerCenter);
					#endif

					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
					scheduler[SCHD_FUNC_CHKVOLTAGE].enabled 		= true;
					
					//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;
					#ifdef simMode
					scheduler[SCHD_FUNC_UPDATESIM].enabled 	= true;
					#endif 

					//ensure we don't end up in driveState of 
					//DRIVE_STATE_LOW_VOLTAGE_RESTART
					driveState=DRIVE_STATE_STOP;

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

//					backWaypoint = location;

					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
					scheduler[SCHD_FUNC_CHKVOLTAGE].enabled 		= true;
					//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;
					
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


					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
					scheduler[SCHD_FUNC_CHKVOLTAGE].enabled 		= true;
					//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;

					prevDriveState=driveState;

					driveState=newState;
					break;
				
				case DRIVE_STATE_RADIO_FAILSAFE:	
					//should not be able to go into DRIVE_STATE_RADIO from Failsafe (protected in navigate())
					break;				
				default:
					manager.sendString("Drive State: Invalid state change. Stopping");
					prevDriveState=DRIVE_STATE_STOP;
					driveState=DRIVE_STATE_STOP;
					output(0,steerCenter);
					break;
			}
			break;
		#endif

		case DRIVE_STATE_LOW_VOLTAGE_STOP:
			manager.sendString("Drive State: Low Voltage (Stopped). Please Replace Batteries");

			#ifdef simMode
			gps.setGroundSpeed(0);
			#else
			output(0,steerCenter);
			#endif

			scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
			scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
			scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
			scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
			scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
			scheduler[SCHD_FUNC_CHKVOLTAGE].enabled 		= true;
			//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;

			//Invalidate previous drive state and force a STOP state if battery replaced
			prevDriveState=DRIVE_STATE_INVALID;
			driveState=newState;
			break;


		#ifndef simMode	
		case DRIVE_STATE_RADIO_FAILSAFE:
			switch(driveState)
			{
				case DRIVE_STATE_INVALID:
				case DRIVE_STATE_STOP:
				case DRIVE_STATE_AUTO:
				case DRIVE_STATE_RADIO:
					manager.sendString("Drive State: Radio failsafe");
					driveState=newState;
					
					radioFailsafeCount=0;

					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;

					#ifdef simMode
					gps.setGroundSpeed(0);
					#else
					output(0,steerCenter);
					#endif

					break;
				default:
					manager.sendString("Drive State: Invalid state change. Stopping");
					prevDriveState=DRIVE_STATE_STOP;
					driveState=DRIVE_STATE_STOP;
					output(0,steerCenter);
					break;
			}
		  break;
		#endif
	}

	#ifdef M_DEBUG
	// Logger Msg
	StateMsg_t msg;
	msg.apmState = apmState;
	msg.driveState = driveState;
	msg.autoState = autoState;
	msg.autoFlag = autoStateFlags;
	msg.voltage = voltage*10;
	msg.amperage = amperage*10;
	msg.groundSpeed = RPMtoMPH(encoder::getRPM())*10;
	debugger.send(msg);
	#endif

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


					//If we had set caution flag then entered avoid we need to clear
					//might consider leaving in caution but for now...
					clearAutoStateFlag(AUTO_STATE_FLAG_CAUTION);

					
					avoidState=AVOID_STATE_ENTER;
					

					int8_t bumperBackDir;

					if (bumperSensor.leftButtonState()==bumperSensor.rightButtonState())
					{
						//Caution bumperBackDir of 0 doesn't mean bumper sensor wasn't triggered
						//It is either both sensors or triggered or neither
						bumperBackDir = 0;						
					}
					else if (bumperSensor.leftButtonState() > bumperSensor.rightButtonState())
						bumperBackDir = 1;
					else
						bumperBackDir = -1;


					//bumper always takes precidence over ultrasonic
					//except when both bumper sensors are triggered.  Then use altrasonic to decide
					if (bumperBackDir!=0)
					{						
						backDir=bumperBackDir;
					}
					else
					{
						//if bumper isn't set (or both were triggered) then use ultrasonic to decide
						if (ping[0][PING_CUR] < ping[4][PING_CUR])
							backDir = 1;
						else
							backDir = -1;

					}



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


	//in sim always act like we are still getting gps updates
	gps.setUpdatedRMC();	

	gps.setCourse(gps.getLocation().headingTo(manager.getTargetWaypoint())+steering_error);

	String msg("SetCourseSim " + String(gps.getCourse()));
	manager.sendString(msg.c_str());	


	//Only move if in auto mode
	//Stalled is simulated but still handle
	if (driveState == DRIVE_STATE_AUTO && autoState != AUTO_STATE_STALLED)
	{
		float dT = millis()-simTime;

		//3600000 = milliseconds per hour
		float dTraveled = gps.getGroundSpeed()*dT/3600000.f;


	String msg("sim ground speed " + String(gps.getGroundSpeed()));
	manager.sendString(msg.c_str());


	char fbuff[32];
	char sbuff[32];
	dtostrf(dTraveled, 6, 6, fbuff);	
	sprintf(sbuff,"%s %s\n","Distance traveled",fbuff);

	//String msg1("sim traveled " + String(dTraveled));
	manager.sendString(sbuff);
		
	
		//todo.  Add Some kind of randomness?
		Waypoint newLocation = location.extrapolate(gps.getCourse(), dTraveled);

		gps.setLocation(newLocation);
	
		//gps.setLatitude(newLocation.degLatitude());
		////Serial.println(newLocation.degLatitude(),8);
		//gps.setLongitude(newLocation.degLongitude());
		////Serial.println(newLocation.degLongitude(),8);

		simTime=millis();
		
	}
	
	
}
#endif


void handleAvoidState(int8_t speed, int8_t steer, uint8_t nextState, uint32_t timeout)
{
	output(speed,steer);
	
	if (millis() > sTime+timeout)
	{
		//reset timer
		sTime=millis();
		//setup for next state
		avoidState=nextState;
		manager.sendString(AVOID_STATES_STRING[nextState]);

	}
}

void navigate()
{
	float throttle = APMRadio::get(RadioChannel[RADIO_THROTTLE]);
	float   mph = ((throttle-90) / 90.f)*MAN_MAX_FWD;
	uint8_t steer = APMRadio::get(RadioChannel[RADIO_STEER]);


	if (driveState == DRIVE_STATE_LOW_VOLTAGE_STOP)
	{
		output(0,steerCenter);
		return;						
	}

	if (driveState != DRIVE_STATE_RADIO_FAILSAFE) {
		if (abs(steer-steerCenter) > 5 || fabs(mph)>0.8f ) 
		{						
			changeDriveState(DRIVE_STATE_RADIO);
			output(mph, steer);
		}
		else
		{	
			if (driveState == DRIVE_STATE_RADIO ) {
				changeDriveState(prevDriveState);
			}
		}
	}
	


	if (driveState == DRIVE_STATE_AUTO  )
	{
		if ( autoState == AUTO_STATE_AVOID)
		{
			//TODO: could be simplified into function calls
			//inputs: speed,steer,nextstate,timeout
			switch (avoidState)
			{
				case AVOID_STATE_ENTER:					
					//set timer
					sTime=millis();
					//setup for next state
					avoidState=AVOID_STATE_FWD_BRAKE;
					manager.sendString(AVOID_STATES_STRING[AVOID_STATE_FWD_BRAKE]);			
					break;
				case AVOID_STATE_FWD_BRAKE:
					{	
						//todo consider consolitating foward and reverse into single function

						float current_speed = RPMtoMPH(encoder::getRPM());

						//If new speed is low enough move on to new state
						//TODO timeout as well?
						if (current_speed < .5)
						{
							avoidState=AVOID_STATE_STRAIGHTBACK;
							manager.sendString(AVOID_STATES_STRING[AVOID_STATE_STRAIGHTBACK]);
						}
						else
						{
							output(0,steerCenter);						
						}
						sTime = millis();
					}
					break;	
				case AVOID_STATE_STRAIGHTBACK:
					handleAvoidState(revSpeed,steerCenter,AVOID_STATE_STEER,avoidStraightBack);
					break;
				case AVOID_STATE_STEER:
					int8_t temp_steer;
					if(backDir == 0)
					{
						temp_steer=steerCenter;
					}
					else if (backDir < 0) 
					{
						temp_steer=steerCenter+revThrow;
					}
					else
					{
						temp_steer=steerCenter-revThrow;						
					}
					handleAvoidState(revSpeed,temp_steer,AVOID_STATE_REV_BRAKE,avoidSteerBack);
					break;
				case AVOID_STATE_REV_BRAKE:
					{
						//todo consider consolitating foward and reverse into single function

						float current_speed = RPMtoMPH(encoder::getRPM());
						//calculate and set a reduce speed (maybe use non-linear in future?)
						//current_speed=current_speed/2.0;
						//If new speed is slow enough move on to new state
						if (current_speed > -.5)
						{
							avoidState=AVOID_STATE_DONE;
							manager.sendString(AVOID_STATES_STRING[AVOID_STATE_DONE]);
						}
						else
						{
							output(0,steerCenter);						
						}
						sTime=millis();
					}
					break;
				case AVOID_STATE_DONE:
					changeAutoState(AUTO_STATE_FULL);
					break;

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

			if (!isSetAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND))
			{
				//turn around if next waypoint is behind the rover
				if (angularError > 65)
				{				
					setAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND);
					turnAroundDir=1;
				}
				else if (angularError < -65)
				{
					setAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND);
					turnAroundDir=-1;
				}			
			}
			else
			{
				//stop turning around when we are close to the right
				//direction				
				if (angularError <= 35 && angularError >= -35)
				{
						clearAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND);
						turnAroundDir=0;
				}
			}




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
						outputAngle *=(4.l*steerThrow);
					else
						outputAngle *=(8.l*steerThrow);

					//negative angle if left turning vs right turning ( or opposite?)
					if(angularError < 0)
						outputAngle *= -1;
					break;
				case 2:
					outputAngle = steering_control_lat_lon(
						backWaypoint.m_gpsCoord,
						manager.getTargetWaypoint().m_gpsCoord,
						location.m_gpsCoord,
						(float) trueHeading);
					break;
				default:
					//simplest. Percentage of error multipled by 2x the steerthow
					// if angularError is 90 degrees off we max out the steering (left/right)
					// this is constrained below
					// at this point we could be 180 degress off and have output angle of 90
					outputAngle = (angularError/180.l)*(2.l*steerThrow);
					break;
			}
			scOutputAngle = outputAngle;


			if (isSetAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND))
			{
				//If the sign is swapped it is because the rover crossed over center line
				//which means angularError is not 100% correct (i.e. 182 vs 178) but is close enough 
				//given that the outputAngle will be reduced to steerThrow

				//if we are forcing right turn but calculations says left; Force right.
				//else if we are forcing left turn but calculations says right; Force left.
				//else do nothing as we are turning in correct direction.
				if (turnAroundDir == 1 && angularError < 0)
					outputAngle *= -1.0;
				else if (turnAroundDir == -1 && angularError > 0)
					outputAngle *= -1.0;
			}


			//scale angle (default of 1.5) This is to account for the 45 deg steer really is 30 deg
			outputAngle *= steerFactor;

			//Keep angle somewhat sensable to avoid issues with values getting larger the +/-180
			//arbitrarily using +/- 90 as it still give some "room" while still clamping.
			outputAngle = constrain(outputAngle,double(-90.0),double(90.0));

			//only adjust steering using the ping sensors if CAUTION flag is active 
			if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION) && false) {
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

				//determine angle based on x,y then adjust from steering center ( 90 )
				outputAngle = toDeg(atan2(y,x))+steerCenter;
			}
			else {
				outputAngle += steerCenter;    
			}
			//Can't steer more then throw
			outputAngle = constrain(outputAngle,
					  				double(steerCenter-steerThrow),
				 	  				double(steerCenter+steerThrow));
			trueOutputAngle = outputAngle;

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

      //start with the fastest speed and decrease as appropriate
			float speed = maxFwd;

			//not sure why one would use disp vs speed?  Is this for turning distance?
			//speed is even a distance.  if we get Less the x feet away and it is smaller 
			//then approachSpeed and turning angle then we us it?

//			speed = min(speed, disp)/6.f; //logical speed clamps
      
			approachSpeed = manager.getTargetWaypoint().getApproachSpeed();

      //speed reduction based on steering angle
      float max_full_speed_steering = 15; //probably should be a UI setable value or moved to top
      if (abs(steerCenter-outputAngle) > max_full_speed_steering)
      {
        speed = approachSpeed * ((steerThrow-abs(steerCenter-outputAngle))/steerThrow);
        //sanity check
        speed = max(speed, 0.0); //put in target approach speed
      }

			speed = min(speed, approachSpeed); //put in target approach speed

			speed = constrain(speed, minFwd, maxFwd);

			//deal with speed reduction for caution and approach
			if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION) || isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH))
			{
				speed = speed/2;
			}

			output(speed, outputAngle);
		}
	}//end drive state auto check

}

void wgs84_to_cartesian(float& x, float& y, const GPS_COORD & coord, const GPS_COORD & ref_coord, float alt0)
{
  const double eccentricity = 0.08181919084261;
  const double equator_radius = 6378137.0;

  double rlat0 = gps_angle_to_float(&ref_coord.latitude) * M_PI / 180.0;
  double depth = -alt0;
  double p = eccentricity * sin(rlat0);
  p = 1.0 - p * p;

  double rho_e = equator_radius *
    (1.0 - eccentricity * eccentricity) / (sqrt(p) * p);
  double rho_n = equator_radius / sqrt(p);
  double rho_lat = rho_e - depth;
  double rho_lon = (rho_n - depth) * cos(rlat0);

  DELTA_GPS delta_gps;
  calc_delta_gps(&ref_coord, &coord, &delta_gps);
  y = (delta_gps.latitude * M_PI / 180.0) * rho_lat;
  x = (delta_gps.longitude * M_PI / 180.0) * rho_lon;
}


float steering_control_lat_lon(GPS_COORD & wp1, const GPS_COORD & wp2, const GPS_COORD & bot_location, float heading_bot)
{
  float x1 = 0.0;
  float y1 = 0.0;
  float x2, y2;
  float xbot, ybot;
  wgs84_to_cartesian(x2, y2, wp2, wp1, 0.0);
  wgs84_to_cartesian(xbot, ybot, bot_location, wp1, 0.0);

  float yaw_bot = heading_bot * M_PI / 180.0;

  return steering_control(x1, y1, x2, y2, xbot, ybot, yaw_bot) * 180.0 / M_PI;      
}

float steering_control(float x1, float y1, float x2, float y2, float xbot, float ybot, float yaw_bot)
{
  float path_yaw = atan2(x2 - x1, y2 - y1);
  float yaw_error = yaw_bot - path_yaw;

  if (yaw_error > M_PI) {
    yaw_error -= 2 * M_PI;
  }
  else if (yaw_error < -M_PI) {
    yaw_error += 2 * M_PI;
  }

  float vx_path = x2 - x1;
  float vy_path = y2 - y1;
  float vx_bot = xbot - x1;
  float vy_bot = ybot - y1;
  float len_path = sqrt(vx_path*vx_path + vy_path*vy_path);
  float evx_path = vx_path / len_path;
  float evy_path = vy_path / len_path;

  // cross_track error (positive is to the left of the path, negative to the right)
  float cross_track_error = -vx_bot * evy_path + vy_bot * evx_path;

  cross_track_error = max(-max_cross_track_error, min(max_cross_track_error, cross_track_error));

  crosstrackError = cross_track_error;
  headingError = yaw_error * 180.0 / M_PI;

  return (k_cross_track * cross_track_error + k_yaw * yaw_error);
}

void extrapPosition()
{
	float dTraveled;

	float dT = millis()-nTime;
	//ignore irrational values
	if(dT < 1000 && !gps.getWarning())
	{ 
		//3600000 = milliseconds per hour
		dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		dTraveled *= (2.l/3.l);//purposly undershoot
		location = location.extrapolate(trueHeading, dTraveled);

    #ifdef M_DEBUG
		// Logger Msg
		ExtrapolatedPositionMsg_t msg;
		GPS_ANGLE loc_lat = location.angLatitude();
		GPS_ANGLE loc_lon = location.angLongitude();
		msg.latitude.minutes = int16_t(loc_lat.minutes);
		msg.latitude.frac = debugger.frac_float_to_int32(loc_lat.frac);  
		msg.longitude.minutes = int16_t(loc_lon.minutes);
		msg.longitude.frac = debugger.frac_float_to_int32(loc_lon.frac);    
		msg.altitude = debugger.alt_float_to_uint16(0);
		debugger.send(msg);
    #endif
	}

	positionChanged();
}


void updateGPS()
{
	
	gps.update();

	if(gps.getUpdatedRMC())
	{
		new_gps = true;
		digitalWrite(A8, HIGH);

		//todo is this not used in sim?
		gps.clearUpdatedRMC();
		
		location = gps.getLocation();


    #ifdef M_DEBUG2
		// Logger Msg
		RawPositionMsg_t msg;
		GPS_ANGLE loc_lat = location.angLatitude();
		GPS_ANGLE loc_lon = location.angLongitude();
		msg.latitude.minutes = int16_t(loc_lat.minutes);
		msg.latitude.frac = debugger.frac_float_to_int32(loc_lat.frac);  
		msg.longitude.minutes = int16_t(loc_lon.minutes);
		msg.longitude.frac = debugger.frac_float_to_int32(loc_lon.frac);    
		msg.altitude = debugger.alt_float_to_uint16(0);
		debugger.send(msg);
    #endif

		if (driveState == DRIVE_STATE_AUTO)
		{
			waypointUpdated();
			positionChanged();
		}
	}
}

bool finish_line_reached(GPS_COORD & wp1, const GPS_COORD & wp2, const GPS_COORD & rover)
{
  float x1 = 0.0;
  float y1 = 0.0;
  float x2, y2;
  float dx, dy;
  float fx, fy;
  float xbot, ybot;

  //convert lat/lon to cartersian with wp1 at (0,0)
  wgs84_to_cartesian(x2, y2, wp2, wp1, 0.0);
  wgs84_to_cartesian(xbot, ybot, rover, wp1, 0.0);

  dx = x2-x1;
  dy = y2-y1;

  float dx_bot = xbot - x2;
  float dy_bot = ybot - y2;
  if ((dx_bot * dx + dy_bot * dy) >= 0.0) {
    return true;
  }
  return false;
}

void waypointUpdated()
{

	distance = manager.getTargetWaypoint().distanceTo(location);

	//approach flag gets locked in until we we move to another waypoint or stop.
	if (distance < approachRadius && isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH) == false )
		setAutoStateFlag(AUTO_STATE_FLAG_APPROACH);

  bool finished_line = finish_line_reached(backWaypoint.m_gpsCoord, manager.getTargetWaypoint().m_gpsCoord, location.m_gpsCoord);

	if(distance <= PointRadius || finished_line)
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
			manager.setTargetIndex(0);
		}

		clearAutoStateFlag(AUTO_STATE_FLAG_APPROACH);
	}
}

void positionChanged()
{
	nTime = millis();


	distance = manager.getTargetWaypoint().distanceTo(location);

  #ifdef M_DEBUG2
	// Logger Msg
	WaypointMsg_t msg;

	GPS_ANGLE backWpLat = backWaypoint.angLatitude();
	GPS_ANGLE backWpLon = backWaypoint.angLongitude();
	msg.latStart.minutes = int16_t(backWpLat.minutes);
	msg.latStart.frac = debugger.frac_float_to_int32(backWpLat.frac);  
	msg.lonStart.minutes = int16_t(backWpLon.minutes);
	msg.lonStart.frac = debugger.frac_float_to_int32(backWpLon.frac);

	Waypoint end_target = manager.getTargetWaypoint();
	GPS_ANGLE endTargetWpLat = end_target.angLatitude();
	GPS_ANGLE endTargetWpLon = end_target.angLongitude();
	msg.latTarget.minutes = int16_t(endTargetWpLat.minutes);
	msg.latTarget.frac = debugger.frac_float_to_int32(endTargetWpLat.frac);  
	msg.lonTarget.minutes = int16_t(endTargetWpLon.minutes);
	msg.lonTarget.frac = debugger.frac_float_to_int32(endTargetWpLon.frac);
  #endif
    #ifdef M_DEBUG2
		msg.latIntermediate.minutes = 0;
		msg.latIntermediate.frac = 0;  
		msg.lonIntermediate.minutes = 0;
		msg.lonIntermediate.frac = 0;
    #endif
  
  
	if (kalman_heading && !heading_lock) {
		pathHeading = trueHeading; //drive straight
    #ifdef M_DEBUG2
    msg.pathHeading = (int16_t)(truncateDegree(pathHeading)*100.0);
    debugger.send(msg);
    #endif
		return;
	}
 else
 {
    pathHeading = backWaypoint.headingTo(manager.getTargetWaypoint());
    #ifdef M_DEBUG2
    msg.pathHeading = (int16_t)(truncateDegree(pathHeading)*100.0);
    debugger.send(msg);
    #endif
 }

/*
	if (backWaypoint.radLongitude() == 0 || distance*5280.l < lookAheadDist*lineGravity)
	{
		pathHeading = location.headingTo(manager.getTargetWaypoint());
	} 
	else 
	{
		// find a point along path from backwaypoint to targetwaypoint in which to drive toward (pathHeading)
		//  First we determine how much of our current vector "counts" toward the target (cos).
		//  From there determine an intermediary waypoint along that original (optimal) vector is a lookAheadDis scalled by lineGravity.
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
		//scale the look-ahead distance (feet) by linegravity (0.25,5.00) then add the 'd' distance. 
		//make sure to convert look-ahead distance to miles before adding to 'd' (miles)
		float D     = d + ((lookAheadDist/5280.l)*lineGravity);
		//find waypoint for this new distance along the previous-to-target path
		Waypoint intermediate = backWaypoint.extrapolate(AB, D);
		//Find the heading to get there from current location
		pathHeading  = location.headingTo(intermediate);

	}

*/
}

void checkBumperSensor()
{
  //If bumper is disabled, do nothing.
  if(!isBumperEnabled) return;
  
	uint8_t leftButtonState=bumperSensor.leftButtonState();
	uint8_t rightButtonState=bumperSensor.rightButtonState();

	//String msg("Left: " + String(leftButtonState) + " Right: " + String(rightButtonState));
	//	manager.sendString(msg.c_str());

	if ( driveState == DRIVE_STATE_AUTO && autoState == AUTO_STATE_FULL && false)
	{
		if (leftButtonState || rightButtonState)
		{		
	
			changeAutoState(AUTO_STATE_AVOID);	
		}
	}	
}

void checkVoltage()
{
	static uint32_t timer = 0;

	voltage  = float(analogRead(67)/1024.l*5.l*10.1f);
	amperage = float(analogRead(66)/1024.l*5.l*17.0f);

	if (driveState != DRIVE_STATE_LOW_VOLTAGE_STOP)
	{
		// check to see if the battery voltage has dropped below the low voltage threshhold
		if (voltage < LOW_VOLTAGE_CUTOFF)
		{
			//is this the first consecutive drop below the low voltage threshold 
			if (timer == 0)
			{
				timer = millis(); //store current time
			}

			//check if voltage has been below the threshold for the required time
			//go straight to a low voltage state if the system boots with a low battery
			if ( timer+(LOW_VOLTAGE_TIME*1000) < millis())
			{
				// halt all vehilce operations
				changeDriveState(DRIVE_STATE_LOW_VOLTAGE_STOP);
				timer = 0;
			}
		}
		else
		{
			//reset voltage timer
			timer = 0;
		}
	}
	else
	{
		if (voltage >= RESET_VOLTAGE_THRESHOLD)
		{
			//is this the first consecutive increase to the reset voltage threshold 
			if (timer == 0)
			{
				timer = millis(); //store current time
			}

			//check if voltage has been above the threshold for the required time
			if ( timer+(RESET_VOLTAGE_TIME*1000) < millis() )
			{
				// enable all vehilce operations
				changeDriveState(DRIVE_STATE_LOW_VOLTAGE_RESTART);
				timer = 0;
			}
		}
		else
		{
			//reset voltage timer
			timer = 0;
		}
	}
	
	#ifdef M_DEBUG
	AsciiMsg_t msg;
	String tst = String(millis()) + ":" + String(timer) + ":" + String(uint8_t(voltage*10));
	msg.ascii.len = tst.length();
	tst.toCharArray(msg.ascii.data,tst.length()+1);
	debugger.send(msg);
	#endif
}

void checkPing()
{
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

	
	if ( driveState == DRIVE_STATE_AUTO && autoState == AUTO_STATE_FULL && false)
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
				}
			}
			else
			{		
				if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION))
				{
					//check if all sonars before clearing flag
					int clear_caution = 0;
					for(int i=0;i<5;i++) {
						if (ping[i][PING_CUR] >= warnLevel[i]) {
							clear_caution++;
						}
					}

					if (clear_caution >= 5) {
						clearAutoStateFlag(AUTO_STATE_FLAG_CAUTION);          
					}

				}
			}
		}
	}

  #ifdef M_DEBUG
	if (pIter == 0) //only log when all ping values have been updated
	{
		// Logger Msg
		SonarMsg_t msg;
		msg.ping1 = ping[0][PING_CUR];
		msg.ping2 = ping[1][PING_CUR];
		msg.ping3 = ping[2][PING_CUR];
		msg.ping4 = ping[3][PING_CUR];
		msg.ping5 = ping[4][PING_CUR];
		debugger.send(msg);
	}
  #endif
}


void output(float mph, uint8_t steer)
{
	static uint8_t straight_ctr = 0;
  static int ctr = 0;
  
	if (abs(steer - steerCenter) > 5) {
		driving_straight = false;
		straight_ctr = 0;
	}
	else {
		straight_ctr++;
		if (straight_ctr >= 5) {
			driving_straight = true;
			straight_ctr = 5;  //don't let overflow    
		}
	}
	steer += steerSkew;
  
	#ifdef simMode
	//set speed to gps simulator (simulator only)
	gps.setGroundSpeed(mph);
	#endif

  #ifdef M_DEBUG2
	if (ctr >= 10) {
	// Logger Msg
	ControlMsg_t msg;
	msg.speed = (int16_t)(mph*100);
	msg.steering = steer;
	msg.sc_steering = (int16_t)(scOutputAngle * 100.0);
	msg.true_steering = (int16_t)(trueOutputAngle * 100.0);
	msg.k_crosstrack = (int16_t)(k_cross_track * 1000.0);
	msg.k_yaw = (int16_t)(k_yaw * 1000.0);
	msg.heading_error = (int16_t)(headingError * 100.0);
	msg.crosstrack_error = (int16_t)(crosstrackError * 100.0);
	debugger.send(msg);
 ctr = 0;
	}
  else {
    ctr++;
  }
  #endif
  
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

void updateHeading()
{
  static bool gps_heading_init = false; //5 good GPS course msgs

  //Tuning parameters
  float gps_heading_error = 57.2958; // 1 rad in deg
  //float error_propagation_rate = 0.5729578; // rad/s in deg 
  float error_propagation_rate = gps_heading_error * ratio_adjustment; // rad/s in deg 
  float speed_thresh = 0.5592341; // 0.25 m/s in mph-- GPS heading will be bad at very low speeds

  //The main estimate (initialized to 0)
  static float cur_heading_est = 0.0;
  static float cur_heading_variance = 1000000;

  // To be populated with actual data from the system
  float cur_wheel_speed = RPMtoMPH(encoder::getRPM());

  static int last_time = mpudmp.lastUpdateTime();

  static int heading_good_counter = 0;

  static float offset = 0;
  int cur_time = mpudmp.lastUpdateTime();
  int dt = cur_time - last_time;
  last_time = cur_time;

  // prediction:
  float euler_z = toDeg(mpudmp.getEulerZ());
  cur_heading_est = truncateDegree(euler_z - offset);
  float old_heading_est = truncateDegree(euler_z - offset);
  cur_heading_variance += pow((error_propagation_rate * dt/1000.0),2);

	#ifdef M_DEBUG
	{
		AsciiMsg_t msg;
		String tst = String(millis()) + ": HEADING_MSG : heading: " + String(trueHeading, 4) + " imu_yaw: " + String(mpudmp.getEulerZ() * 180.0 / M_PI, 4) + " ratio: " + String(ratio_adjustment, 10);
		msg.ascii.len = tst.length();
		tst.toCharArray(msg.ascii.data,tst.length()+1);
		debugger.send(msg);
	}
	{
		AsciiMsg_t msg;
		String tst = String(millis()) + ": IMU_MSG_RPY:  r: " + String(mpudmp.getEulerX()) + "  p: " + String(mpudmp.getEulerY()) + "  y: " + String(mpudmp.getEulerZ()) + '\n' +
		             String(millis()) + ": IMU_MSG_ACC: ax: " + String(mpudmp.get_ax()) + " ay: " + String(mpudmp.get_ay()) + " az: " + String(mpudmp.get_az()) + '\n' +
                     String(millis()) + ": IMU_MSG_ANG: wx: " + String(mpudmp.get_wx()) + " wy: " + String(mpudmp.get_wy()) + " wz: " + String(mpudmp.get_wz());
		msg.ascii.len = tst.length();
		tst.toCharArray(msg.ascii.data,tst.length()+1);
		debugger.send(msg);
	}
	#endif

  if (new_gps) {
    float cur_gps_heading_meas = truncateDegree(gps.getCourse());  // this will update only once a second?
    new_gps = false;

    //ensure that we have gotten at least one valid gps course
    if (!gps_heading_init && fabs(cur_wheel_speed) > speed_thresh) {
      if (cur_gps_heading_meas > 0.0001 || cur_gps_heading_meas < -0.0001) {
        gps_heading_init = true;
      }
    }
        
    // correction:
    // only apply correction if:
    // - vehicle speed is above speed threshold
    // - a valid GPS course is available (start using course before turning allowed)
    // - we are not currently in a hard turn (turn around) mode
    // - we are driving relativly straight
    if (fabs(cur_wheel_speed) > speed_thresh && gps_heading_init && !isSetAutoStateFlag(AUTO_STATE_FLAG_TURNAROUND) && driving_straight) {
      float update_heading = cur_gps_heading_meas;
      if (cur_wheel_speed < 0) {
        // driving backwards, flip the heading measurement
        update_heading = truncateDegree(update_heading + 180.0);
      }
      float gps_heading_var = pow(gps_heading_error,2); 
      float heading_diff = truncateDegree(update_heading - cur_heading_est);
      float var_denominator = (1.0 / cur_heading_variance + 1.0 / gps_heading_var);
      cur_heading_est = truncateDegree((cur_heading_est / cur_heading_variance + (cur_heading_est + heading_diff) / gps_heading_var)  / var_denominator);
      cur_heading_variance = 1.0 / var_denominator;

      offset -= truncateDegree(cur_heading_est - old_heading_est);

      //keep variance high until we have a stable solution
      if (heading_lock == false)
      {
        if (abs(heading_diff) < HEADING_LOCK_RANGE) {
          heading_good_counter++;
          if (heading_good_counter >= MIN_GOOD_HEADINGS) {
            heading_lock = true;
          }
          else {
            cur_heading_variance *= 1000;            
          }
        }
        else {
          cur_heading_variance *= 1000;            
          heading_good_counter = 0;
        }
      }
    }
  }
  
  k_heading = truncateDegree(euler_z - offset);
}

void updateGyro()
{
  static int ctr = 0;
	//update gyro and accel here (mpu6000_dmp)
	mpudmp.update();

 	if (mpudmp.updateReady())
  	{
		updateHeading();
  		float euler_x, euler_y, euler_z;

	    mpudmp.getEuler(euler_x,euler_z,euler_z);
	    
		last_euler_z = cur_euler_z;
		cur_euler_z = euler_z;
	    trueHeading =  cur_euler_z  - euler_z_offset;
	    
	    //correct for any wraps out of the -3.14 to 3.14
		if (trueHeading < -PI)
		trueHeading += 2*PI;

		if (trueHeading > PI)
		trueHeading -= 2*PI;	    

		trueHeading = toDeg(trueHeading);

	    mpudmp.clearUpdateReady();
	}

	if (mpudmp.lastUpdateTime() > (millis() + 2000 ))
	{
		if ( mpudmp.lastUpdateTime() > (millis() + 4000))
		{
			manager.sendString("Gyro: error");
			mpudmp.setLastUpdateTime(millis());
		}

	}

	if (kalman_heading) {
		trueHeading = k_heading;
	}
	
	#ifdef M_DEBUG2
  if (ctr >= 10) {
	// Logger Msg
	OrientationMsg_t msg;
	msg.heading = (uint16_t)(trueHeading*100);
	msg.roll = (uint16_t)(toDeg(mpudmp.getEulerY())*100);
	msg.pitch = (uint16_t)(toDeg(mpudmp.getEulerX())*100);
	debugger.send(msg);
  ctr = 0;
  }
  else {
    ctr++;
  }
	#endif
}

void reportLocation()
{
	//digitalWrite(45,HIGH);

	manager.sendTelem(Protocol::telemetryType(LATITUDE),    location.degLatitude());
	manager.sendTelem(Protocol::telemetryType(LONGITUDE),   location.degLongitude());
	
	#ifdef simMode
	manager.sendTelem(Protocol::telemetryType(HEADING),     gps.getCourse());	
	#else	
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading);
	#endif

	manager.sendTelem(Protocol::telemetryType(PITCH),       toDeg(mpudmp.getEulerX()));
	manager.sendTelem(Protocol::telemetryType(ROLL),        toDeg(mpudmp.getEulerY()));

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

	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_BUMPER),BUMPER_BUTTON_LEFT, bumperSensor.leftButtonState());
	manager.sendSensor(Protocol::dataSubtype(OBJDETECT_BUMPER),BUMPER_BUTTON_RIGHT, bumperSensor.rightButtonState());

	//extra gps info
	manager.sendTelem(Protocol::telemetryType(GPSNUMSAT), gps.getNumSat());
	manager.sendTelem(Protocol::telemetryType(GPSHDOP), gps.getHDOP());

	//extra gps info
	manager.sendTelem(Protocol::telemetryType(HEADING_LOCK), ( heading_lock ) == true ? 1 : 0);

	//digitalWrite(45,LOW);
}

void reportState()
{
	//digitalWrite(45,HIGH);
	
	manager.sendState(Protocol::stateType(APM_STATE),apmState);
	manager.sendState(Protocol::stateType(DRIVE_STATE),driveState);
	manager.sendState(Protocol::stateType(AUTO_STATE),autoState);
	manager.sendState(Protocol::stateType(AUTO_FLAGS),autoStateFlags);
	manager.sendState(Protocol::stateType(GPS_STATE),gps.getWarning());


	//digitalWrite(45,LOW);
}

void updateSteerSkew(float s)
{
  steerSkew = int8_t(round(s));
//  steerSkew = -4;
}

void newPIDparam(float x)
{
	// indexes for cruise control PID settings defined below
	// Note: low and upper of [-90,90] exceed range of some ESCs
	//       reducing to [-60,60] changed output range to 
	//		 [900,2100] PWM 
	cruisePID = PIDparameters(settings.get(11),
                              settings.get(12),
                              settings.get(13), -60, 60 );
}

void stateStop()
{	
	changeDriveState(DRIVE_STATE_STOP);
}

void stateStart()
{
  //just now starting a mission - set backwaypoint to current location
  backWaypoint = location;

	changeDriveState(DRIVE_STATE_AUTO);
}

void bumperEnable() 
{
  isBumperEnabled = true;
}

void bumperDisable()
{
  isBumperEnabled = false;
}

void version()
{
	manager.sendVersion(version_major, version_minor, version_rev);
}


//todo: add string and make array of structs?
//update setupSettings to use array too
//hard to mantain currently but another branch is updating the setupsettings()
//function and don't want to cause conflicts

enum SETTINGS_INDEX_ID
{
	SETTINGS_DATA_MIN=0,
	SETTINGS_DATA_MAX=1,
	SETTINGS_DATA_DEF=2,
};

const float settingsData[][3] PROGMEM = { 
															{.25,5.0,1.0},			//Line Gravity
															{0,90,45},					//Steer Throw
															{0,2,1},						//Steer Style
															{0,8,1.5},					//Steer Scalar (steerFactor)
															{1,3,1.5},					//Min Fwd Speed
															{1.5,3,2.0},				//Max Fwd Speed
															{0,90,20},					//Rev Str Throw
															{-2,-1,-1},					//Reverse Speed
															{1,20000,1400},			//Ping Factor
															{2000,8000, 2000},  //Coast Time
															{500,2000, 1000},   //Min Rev Time
															{0,1, 0.05},				//Cruise P
															{0,10, 0.1},				//Cruise I
															{0,10, 0.0},				//Cruise D
															{0,12, 5.85},			  //Tire Diameter
															{0,180, 90},				//Steer Center
															{0,.003, .0015},		//Waypoint acheived radius in miles
															{0,.0076, .0038},	  //Approach radius
															{0,1, 0},					  //old GryoSync...removed
															{0,1,0},				//???...missing index 19
															{-10,10, 0},				//Steer Skew
															{500,10000, 1000},	//Avoid Ping value Edges
															{500,10000, 1600},	//Avoid Ping value Middles
															{500,10000, 3000},	//Avoid Ping value center
															{500,10000, 2000},	//Warn Ping value Edges
															{500,10000, 3200},	//Warn Ping value Middles
															{500,10000, 6000},	  //Warn Ping value center
															{0,1,0},						//Radio failsafe enabled
															{0,1,0},						//Radio Dev type


														};

void setDefaultSettings()
{
	for (byte i=0;i<sizeof(settingsData)/sizeof(settingsData[0]);i++)
	{		
		storage->updateRecord(i,pgm_read_float_near(&settingsData[i][SETTINGS_DATA_DEF]));
	}
}


void setupSettings()
{
	uint8_t index;

   
	/*GROUNDSETTING index="0" name="Line Gravity" min="0.25" max="5.00" def="1.00"
	 *Defines how strongly the rover should attempt to return to the original
	 *course between waypoints, verses the direct path from its current location
	 * to the target<br>
	 */
	index = 0;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &lineGravity>);

	/*GROUNDSETTING index="1" name="Steer Throw" min="0" max="90" def="45"
	 *The number of degrees that rover will turn its wheels when it needs to
	 *to turn its most extreme amount
	 */
  index = 1;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<int, &steerThrow>);

	/*GROUNDSETTING index="2" name="Steer Style" min="0" max="2" def="1"
	 *Switches between arctangent of error steering (0) <br>
	 *square of error steering (1) <br>
	 *and proportional to error steering (2)
	 */
	index = 2;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<int, &steerStyle>);

	/*GROUNDSETTING index="3" name="Steer Scalar" min="0" max="8" def="1.5"
	 *Multiplier that determines how aggressively to steer
	 */
	index = 3;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &steerFactor>);


	/*GROUNDSETTING index="4" name="Min Fwd Speed" min="1" max="3" def="1.5"
	 *Minimum forward driving speed in MPH
	 */
	index = 4;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &minFwd>);

	/*GROUNDSETTING index="5" name="Max Fwd Speed" min="1.5" max="3" def="2.0"
	 *Maximum forward driving speed in MPH
	 */
	index = 5;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &maxFwd>);

	/*GROUNDSETTING index="6" name="Rev Str Throw" min="0" max="90" def="20"
	 *How far to turn the wheels when backing away from an obstacle
	 */
	index = 6;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<int, &revThrow>);

	/*GROUNDSETTING index="7" name="Reverse Speed" min="-2" max="-1" def="-1.0"
	 *Speed in MPH to drive in reverse
	 */
	index = 7;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &revSpeed>);

	/*GROUNDSETTING index="8" name="Ping Factor" min="1" max="20000" def="1400"
	 *Factor to determine how strongly obstacles effect the rover's course <br>
	 *Larger numbers correspond to larger effects from obstacles
	 */
	index = 8;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &pingWeight>);

	/*GROUNDSETTING index="9" name="Coast Time" min="2000" max="8000" def="2000"
	 *Time in milliseconds to coast before reversing when an obstacle is encountered
	 */
	index = 9;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<int, &avoidCoastTime>);

	/*GROUNDSETTING index="10" name="Min Rev Time" min="500" max="2000" def="1000"
	 *Minimum time in seconds to reverse away from an obstacle
	 */
	index = 10;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &dangerTimeCallback);

	/*GROUNDSETTING index="11" name="Cruise P" min="0" max="1" def="0.05"
	 *P term in cruise control PID loop
	 */
	index = 11;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &newPIDparam);

	/*GROUNDSETTING index="12" name="Cruise I" min="0" max="10" def="0.1"
	 *I term in cruise control PID loop
	 */
	index = 12;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &newPIDparam);

	/*GROUNDSETTING index="13" name="Cruise D" min="0" max="10" def="0.0"
	 *D term in cruise control PID loop
	 */
	index = 13;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &newPIDparam);

	/*GROUNDSETTING index="14" name="Tire Diameter" min="0" max="12" def="5.85"
	 *Tire Diameter in inches, used to calculate MPH
	 */
	index = 14;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float, &tireDiameter>);

	/*GROUNDSETTING index="15" name="Steer Center" min="0" max="180" def="90"
	 *Center point in degrees corresponding to driving straight
	 */
	index = 15;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<int, &steerCenter>);

//Target radi settings


	/*GROUNDSETTING index="16" name="Waypoint acheived radius in miles" min="0" max=".003" def=".0015"
	 * Radius centered at waypoint where target is determined to be meet
	 */
	index = 16;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float,&PointRadius>);

	/*GROUNDSETTING index="17" name="Approach radius" min="0" max=".0076" def=".0038"
	 * Radius cneter at waypoint where the approach flag is set
	 */

	index = 17;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), callback<float,&approachRadius>);

//skew

	/*GROUNDSETTING index="20" name="Steer Skew" min="-45" max="45" def="0"
	 *
	 */
	index = 20;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &updateSteerSkew);

//ping 

	/*GROUNDSETTING index="21" name="Avoid Ping value Edges" min="500" max="10000" def="1000"
	 *
	 */
	index = 21;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingBlockLevelEdgesCallback);

	/*GROUNDSETTING index="22" name="Avoid Ping value Middles" min="500" max="10000" def="1600"
	 *
	 */
	index = 22;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingBlockLevelMiddlesCallback);

  /*GROUNDSETTING index="23" name="Avoid Ping value center" min="500" max="10000" def="3000"
	 *
	 */
	index = 23;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingBlockLevelCenterCallback);

	/*GROUNDSETTING index="24" name="Warn Ping value Edges" min="500" max="10000" def="2500"
	 *
	 */
	index = 24;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingWarnLevelEdgesCallback);

	/*GROUNDSETTING index="25" name="Warn Ping value Middles" min="500" max="10000" def="5000"
	 *
	 */
	index = 25;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingWarnLevelMiddlesCallback);

	/*GROUNDSETTING index="26" name="Warn Ping value center" min="500" max="10000" def="8000"
	 *
	 */
	index = 26;
	settings.attach(index, pgm_read_float_near(&settingsData[index][0]), pgm_read_float_near(&settingsData[index][1]), pgm_read_float_near(&settingsData[index][2]), &pingWarnLevelCenterCallback);
	
	/*GROUNDSETTING index="27" name="Radio Failsafe" min="0" max="1" def="0"
	 * Enable = 1
	 * Disable = 0
	 */
	settings.attach(27, 0, &radioFailsafe);

	/*GROUNDSETTING index="28" name="Radio Controller" min="0" max="1" def="0"
	 * 6 Channel - TGY = 0
	 * 3 Channel - HobbyKing = 1
	 */
	settings.attach(28, 0, &radioController);

}


 