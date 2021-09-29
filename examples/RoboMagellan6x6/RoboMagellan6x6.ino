#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "MINDS-i-Drone.h"
#include "util/callbackTemplate.h"
#include "version.h"

//#define M_DEBUG  //comment out to disable debugger

#ifdef M_DEBUG
  #include "MINDSiDebugger.h"
  MINDSiDebugger debugger;
#endif

#define MIN_GOOD_HEADINGS 5 
#define HEADING_LOCK_RANGE 10
bool kalman_heading = true;
float k_heading = 0;
bool heading_lock = false;
bool new_gps = false;

int8_t steer_bias = -5;

bool driving_straight = false;
//=============================================//
//  Defines used to control compile options
//=============================================//

// == Sim mode ==
//#define simMode true

//== encoder ==
#define useEncoder true

//== external logger ==
#define extLogger false

//==== debug vars ====/
bool extLogger_gps = false;
bool extLogger_gen = true;

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

int8_t turnAroundDir=0;

//== hardware related ==

HardwareSerial *commSerial	= &Serial;
Storage<float> *storage	= eeStorage::getInstance();
#ifdef simMode
LEA6H_sim		gps;
#else
LEA6H			gps;
#endif

//compass
//HMC5883L hmc;

//MPU6000			mpu;
MPU6000_DMP		mpudmp;


bumper bumperSensor;

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

uint32_t uTime = 0, nTime = 0, sTime = 0, pTime;
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

//== mpu6000 ==//
float euler_z_offset=0;
float last_euler_z=0;
float cur_euler_z=0;

//== gyro ==

double   gyroHalf; //Store Gyro Heading halfway between gps points
double   distance;
boolean  stop = true;
int8_t  backDir;

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
void readAccelerometer();
void reportLocation();
void reportState();
void extrapPosition();
void navigate();
void compass_sync();
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
	{readAccelerometer,	110	,22		,1},
	{reportLocation,	110	,44		,1},
	{reportState,		110	,110	,1},
	{checkPing,			22	,88		,1},
	{checkBumperSensor, 80	,55		,1},
	//{navigate,			0	,66		,1},
	{compass_sync,		500	,1000	,0},
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
	SCHD_FUNC_BUMPSENSOR,
	//SCHD_FUNC_NAVIGATE,
	SCHD_FUNC_COMPASS_SYNC,
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
int     cautionTime=1000;
float	tireDiameter;
int		steerCenter;
//in miles, margin for error in rover location
float PointRadius  = .0015; 
float approachRadius = .0038;


float compassOffset = M_PI;


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
//testing
void triggerGyroSyncNorth(float in){
	euler_z_offset = mpudmp.getEulerZ();
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

#define LOG_TYPE_ERROR 		0x01
#define LOG_TYPE_INFO 		0x02
#define LOG_TYPE_VERBOSE	0x04
#define LOG_TYPE_NONE		0x00
#define LOG_TYPE_ALL		( LOG_TYPE_ERROR & LOG_TYPE_INFO & LOG_TYPE_VERBOSE )


#define ENTITY_TYPE_GENERAL 	0x01
#define ENTITY_TYPE_GPS 		0x02
#define ENTITY_TYPE_COMPASS		0x04
#define ENTITY_TYPE_NAV			0x08
#define ENTITY_TYPE_STATE		0x10
#define ENTITY_TYPE_MISC		0x11
#define ENTITY_TYPE_NONE		0x00
#define ENTITY_TYPE_ALL		(ENTITY_TYPE_GENERAL & ENTITY_TYPE_GPS & ENTITY_TYPE_COMPASS & ENTITY_TYPE_MISCENTITY_TYPE_NAV & ENTITY_TYPE_MISC ) 


void extLog(const char type[], float val, int format=6)
{
	if (!extLogger_gen)
		return;
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
	if (!extLogger_gen)
	return;

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
	if (!extLogger_gen)
		return;	
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


 //    // Load accelerometer/magnetometer parameters from EEPROM
	// if(!settings.foundIMUTune())
	// {
	//     /*#IMULOAD Couldn't load a valid Accelerometer and
	//      * Magnetometer tune from EEPROM
	//     **/
	//     //errorsDetected = true;
	//     //comms.sendString("IMULOAD Failed");
	//     manager.sendString("IMULOAD Failed");

	// }
	// else 
	// {
	//     //mpu.tuneAccl(settings.getAccelTune());
	//     Serial2.println("Reading mag tune");
	//     hmc.tune(settings.getMagTune());
	//     Serial2.println("Done Reading mag tune");
	// }

	//start gps
	//todo check if ok?
	//set mag dec from settings?
 	//hmc.begin();


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
	manager.setVersionCallback(version);

	// //===   ba testing ===//

    // pinMode(A7, OUTPUT);    
	// pinMode(A8, OUTPUT);
	
	// pinMode(13, OUTPUT);
	
	// #ifdef extLogger
	// pinMode(45, OUTPUT);
	// #endif

	//=== end ba testing ===//


	

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

//this all needs to be cleaned up after it works (hopefully)
	for (uint8_t i=0;i<10;i++)
	{
		updateGyro();
		manager.update();
		delay(10);
	}

	// compass_sync();
	// compass_sync();
	// compass_sync();
	// compass_sync();	

	euler_z_offset = mpudmp.getEulerZ();


	changeAPMState(APM_STATE_SELF_TEST);

}



void loop()
{
	int i;


	



	//======================================================
	//Functions that don't directely affect the rover.
	//======================================================

	manager.update();

	updateGPS();
	updateGyro();
	bumperSensor.update();

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


	//======================================================
	//  Debug commands handling 
	//  (move to function eventually?)	
	//======================================================
	#ifdef extLogger
	if (Serial2.available())
	{

		Serial2.println();
		switch( Serial2.read() )
		{
			
			case 'n':
				euler_z_offset = mpudmp.getEulerZ();
				break;
			case 's':
			{
				Serial2.print(F("Current euler z: "));
				Serial2.println(cur_euler_z);
				Serial2.print(F("Last euler z: "));
				Serial2.println(last_euler_z);
				Serial2.print(F("Offset euler z: "));
				Serial2.println(euler_z_offset);

				Serial2.print(F("Gyro IRQ count: "));
				Serial2.println(mpudmp.irqCount());


				Serial2.print(F("True Heading: "));
				Serial2.println(trueHeading);
				Serial2.print(F("Path heading: "));
				Serial2.println(pathHeading);

				Serial2.print(F("Distance to waypoint: "));
				Serial2.println(distance);

				Serial2.print(F("last output angle: "));
				Serial2.println(lastOutputAngle);
				Serial2.print(F("last angular Error: "));
				Serial2.println(lastAngularError);

				Serial2.print(F("Target Waypoint: "));
				char str[32];
				GPS_COORD tempGPSCoord = manager.getTargetWaypoint().m_gpsCoord;
		        gps_coord_to_str(&tempGPSCoord,str,32,9,"DD");
		        Serial2.println(str);
				// Serial2.print(manager.getTargetWaypoint().degLatitude(),6);
				// Serial2.print(F(" "));
				// Serial2.println(manager.getTargetWaypoint().degLongitude(),6);

				break;
			}
			case 'm':
				Serial2.print(F("APM State:"));
				Serial2.println(apmState);
				Serial2.print(F("Drive State:"));
				Serial2.println(driveState);
				Serial2.print(F("Prev Drive State:"));
				Serial2.println(prevDriveState);
				Serial2.print(F("Auto State:"));
				Serial2.println(autoState);
				Serial2.print(F("Auto State flags:"));
				Serial2.println(autoStateFlags,HEX);
				break;
			case 'g':				
				extLogger_gps=!extLogger_gps;
				Serial2.print(F("gps msg set to "));
				Serial2.println(extLogger_gps ? "on" : "off");
				break;
			case 'v':
				extLogger_gen=!extLogger_gen;
				Serial2.print(F("Nav msg set to "));
				Serial2.println(extLogger_gen ? "on" : "off");				
				break;
			case 'c':
				compass_sync();
				break;
			case 'r':
			{			
				float   mph = ((APMRadio::get(RadioPin[1])-90) / 90.f)*maxFwd;
				uint8_t steer = APMRadio::get(RadioPin[2]);

				Serial2.print("Radio mph: ");
				Serial2.println(mph);
				Serial2.print("Radio steer: ");
				Serial2.println(steer);
				break;
			}
			case 'h':
				Serial2.println(F("n - Set current rover direction to North"));
				Serial2.println(F("m - State Info"));
				Serial2.println(F("s - Print current Nav vars"));
				Serial2.println(F("g - Toggle gps message"));
				Serial2.println(F("v - Toggle nav message"));
				Serial2.println(F("r - Radio info"));

				break;
			default:
				break;
		}

	}
	#endif





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
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
					
					//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;
					#ifdef simMode
					scheduler[SCHD_FUNC_UPDATESIM].enabled 	= true;
					#endif 
					
					driveState=newState;

					//still might be rolling so not best place?
					//compass_sync();

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

					backWaypoint = location;

					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= true;
					scheduler[SCHD_FUNC_RDACC].enabled 		= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
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

					extLog("Drive State","Radio");


					scheduler[SCHD_FUNC_EXTRPPOS].enabled 	= false;
					scheduler[SCHD_FUNC_RDACC].enabled 		= true;
					scheduler[SCHD_FUNC_RPRTLOC].enabled 	= true;
					scheduler[SCHD_FUNC_RPRTSTATE].enabled 	= true;
					scheduler[SCHD_FUNC_CHKPING].enabled 	= true;
					scheduler[SCHD_FUNC_BUMPSENSOR].enabled 	= true;
					//scheduler[SCHD_FUNC_NAVIGATE].enabled	= true;

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
					
					//maybe here?
					//compass_sync();


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

						//String msg("MPH " + String(mph));
						//manager.sendString(msg.c_str());
						//String msg1("RPM " + String(encoder::getRPM()));
						//manager.sendString(msg1.c_str());


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

			extLog("angularError",angularError,6);

			//angularCorrection = (lastAngularError - angularError) - lastOutputAngle - lastAngularCorrection;



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
				default:
					//simplest. Percentage of error multipled by 2x the steerthow
					// if angularError is 90 degrees off we max out the steering (left/right)
					// this is constrained below
					// at this point we could be 180 degress off and have output angle of 90
					outputAngle = (angularError/180.l)*(2.l*steerThrow);
					break;
			}


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

			//possible correction in steering (pulling left or right)
			//outputAngle += steerSkew;

			extLog("nav outputAngle post err correct",outputAngle);

			//only adjust steering using the ping sensors if CAUTION flag is active 
			if (isSetAutoStateFlag(AUTO_STATE_FLAG_CAUTION)) {
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
			}
			else {
				outputAngle += steerCenter;    
			}
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
	//ignore irrational values
	if(dT < 1000 && !gps.getWarning())
	{ 
		//3600000 = milliseconds per hour
		dTraveled = gps.getGroundSpeed()*dT/3600000.f;
		dTraveled *= (2.l/3.l);//purposly undershoot
		location = location.extrapolate(trueHeading, dTraveled);

		char str[32];
		GPS_COORD tempGPSCoord = location.m_gpsCoord;
		gps_coord_to_str(&tempGPSCoord,str,32,9,"DD");
		extLog("Extrap coord",str);
		//extLog("extrap lat",location.degLatitude(),6);
		//extLog("extrap long",location.degLongitude(),6);

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
	//digitalWrite(A7,LOW);

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

		#ifdef extLogger
		if (extLogger_gps)
		{
			char str[32];
			GPS_COORD tempGPSCoord = location.m_gpsCoord;
			gps_coord_to_str(&tempGPSCoord,str,32,9,"DD");
			extLog("GPS coord",str);
			// extLog("GPS lat",location.degLatitude(),6);
			// extLog("GPS long",location.degLongitude(),6);
		}
		#endif

    #ifdef M_DEBUG
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
	// if(!gps.getWarning() && gps.getCourse() != 0)
	// {

	// 	if (!isSetAutoStateFlag(AUTO_STATE_AVOID) && !isSetAutoStateFlag(AUTO_STATE_FLAG_APPROACH) && 
	// 							(fabs(lastOutputAngle)<20.0) )
	// 	{
	// 		trueHeading = gps.getCourse();
	// 		euler_z_offset = last_euler_z - trueHeading;


	// 		extLog("SH trueHeading", trueHeading,6);

	// 		/*
	// 		if(millis() - gpsTime < 1500) //dont use gyrohalf if it is too old
	// 			trueHeading = truncateDegree(gps.getCourse() + trueHeading - gyroHalf);
	// 		else
	// 			trueHeading = truncateDegree(gps.getCourse());
	// 		gpsHalfTime = millis()+(millis()-gpsTime)/2;*/
	// 	}
	// }
	
}




void positionChanged()
{
	nTime = millis();


	distance = manager.getTargetWaypoint().distanceTo(location);

	extLog("PC distance",distance,6);
	extLog("PC lat",location.degLatitude(),6);
	extLog("PC long",location.degLongitude(),6);

  #ifdef M_DEBUG
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
  
	if (kalman_heading && !heading_lock) {
		pathHeading = trueHeading; //drive straight
		return;
	}  

	if (backWaypoint.radLongitude() == 0 || distance*5280.l < 25)
	{
    #ifdef M_DEBUG
		msg.latIntermediate.minutes = 0;
		msg.latIntermediate.frac = 0;  
		msg.lonIntermediate.minutes = 0;
		msg.lonIntermediate.frac = 0;
    #endif
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
		Waypoint intermediate = backWaypoint.extrapolate(AB, D);
		//Find the heading to get there from current location
		pathHeading  = location.headingTo(intermediate);

    #ifdef M_DEBUG
		GPS_ANGLE intermediateWpLat = intermediate.angLatitude();
		GPS_ANGLE intermediateWpLon = intermediate.angLongitude();

		msg.latIntermediate.minutes = int16_t(intermediateWpLat.minutes);
		msg.latIntermediate.frac = debugger.frac_float_to_int32(intermediateWpLat.frac);  
		msg.lonIntermediate.minutes = int16_t(intermediateWpLon.minutes);
		msg.lonIntermediate.frac = debugger.frac_float_to_int32(intermediateWpLon.frac);
    #endif
	}

  #ifdef M_DEBUG
	msg.pathHeading = (int16_t)(truncateDegree(pathHeading)*100.0);
	debugger.send(msg);
  #endif
  
	extLog("PC PathHeading",pathHeading,6);

}

void checkBumperSensor()
{
  //If bumper is disabled, do nothing.
  if(!isBumperEnabled) return;
  
	uint8_t leftButtonState=bumperSensor.leftButtonState();
	uint8_t rightButtonState=bumperSensor.rightButtonState();

	//String msg("Left: " + String(leftButtonState) + " Right: " + String(rightButtonState));
	//	manager.sendString(msg.c_str());

	if ( driveState == DRIVE_STATE_AUTO && autoState == AUTO_STATE_FULL)
	{
		if (leftButtonState || rightButtonState)
		{		
	
			changeAutoState(AUTO_STATE_AVOID);	
		}
	}	
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
				//set current time
				pTime=millis();
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
					//extLog("Caution flag: ","0");
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

	//digitalWrite(A8,LOW);
}


void output(float mph, uint8_t steer)
{
	static uint8_t straight_ctr = 0;

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
	steer += steer_bias;
  
	#ifdef simMode
	//set speed to gps simulator (simulator only)
	gps.setGroundSpeed(mph);
	#endif


	extLog("mph",mph,6);
	extLog("steer",steer,6);

  #ifdef M_DEBUG
	// Logger Msg
	ControlMsg_t msg;
	msg.speed = (int16_t)(mph*100);
	msg.steering = steer;
	debugger.send(msg);
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
  float error_propagation_rate = 0.5729578; // rad/s in deg 
  float gps_heading_error = 57.2958; // 1 rad in deg
  float speed_thresh = 0.5592341; // 0.25 m/s in mph-- GPS heading will be bad at very low speeds
//  bool apply_heading_lock = true;
//  int heading_lock_init_time = 5000; // milliseconds

  //The main estimate (initialized to 0)
  static float cur_heading_est = 0.0;
  static float cur_heading_variance = 1000000;

  // To be populated with actual data from the system
//  float cur_gyro_meas =  -mpudmp.getGyroZ_raw(); // This should update at least once per estimation cycle (otherwise there's no reason do the estimation so frequently). Make sure that the direction is consistent with orientation.
  float cur_wheel_speed = RPMtoMPH(encoder::getRPM());

  static int last_time = mpudmp.lastUpdateTime();
//  static int stopped_time = 0; //in milliseconds 

  static int heading_good_counter = 0;

  static float offset = 0;
  int cur_time = mpudmp.lastUpdateTime();
  int dt = cur_time - last_time;
  last_time = cur_time;

/*  // Heading lock
  if (cur_wheel_speed == 0.0) {
      stopped_time += dt;
      if (apply_heading_lock and stopped_time > heading_lock_init_time) {
        return;
      }
  }
  else {
      // rover is moving, reset "stop_time"
      stopped_time = 0;
  }
*/

  // prediction:
  float euler_z = toDeg(mpudmp.getEulerZ());
  cur_heading_est = truncateDegree(euler_z - offset);
  float old_heading_est = truncateDegree(euler_z - offset);
  cur_heading_variance += pow((error_propagation_rate * dt/1000.0),2);

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
	//update gyro and accel here (mpu6000_dmp)
	mpudmp.update();

 	if (mpudmp.updateReady())
  	{
		updateHeading();
  		float euler_x, euler_y, euler_z;

	    //mpu.getQ(q_w,q_x,q_y,q_z);

	    mpudmp.getEuler(euler_x,euler_z,euler_z);
	    
 
	    
	    //Serial2.print("eulerz; "); 
	    //Serial2.println(euler_z); 
	    

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
	
	// float dt = lowFilter.millisSinceUpdate();
	// float Gz = -toDeg(mpu.gyroZ());
	// lowFilter.update(Gz);
	// highFilter.update(Gz-lowFilter.get());

	// //todo think about simulating the gyro?
	// #ifndef simMode	
	// trueHeading = truncateDegree(trueHeading + dt*(highFilter.get()));


	// extLog("UG trueHeading", trueHeading,6);


	// if(gpsHalfTime < millis() && gpsHalfTime!=0){
	// 	gyroHalf = trueHeading;
	// 	gpsHalfTime = 0;
	// }
	// #endif

  #ifdef M_DEBUG
	// Logger Msg
	OrientationMsg_t msg;
	msg.heading = (uint16_t)(trueHeading*100);
	msg.roll = (uint16_t)(toDeg(mpudmp.getEulerY())*100);
	msg.pitch = (uint16_t)(toDeg(mpudmp.getEulerX())*100);
	debugger.send(msg);
  #endif
}


void readAccelerometer()
{

	// digitalWrite(13,HIGH);
	// Ax = mpu.acclX();
	// Ay = mpu.acclY();
	// Az = mpu.acclZ();
	// //atan2 gets angle of x and y vectors
	// pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
	// //atan2 gets angle of y and z vectors
	// roll.update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
	// digitalWrite(13,LOW);
}

void compass_update()
{
	//cur_compass_heading = hmc.getAzimuth();
	//cur_compass_heading_avg;
}

void compass_sync()
{

	// if ( driveState == DRIVE_STATE_STOP)
	// {
	// 	//=== use gps to set offset of gyro ===//
	// 	//todo clean this up
	// 	float tmp = hmc.getAzimuth();
	// 	float gyro_tmp = mpudmp.getEulerZ();
	// 	float compassRaw = hmc.getRawAzimuth();

	// 	Serial2.print("compass raw: ");
	// 	Serial2.print(toDeg(compassRaw));
	// 	Serial2.print(" ");
	// 	Serial2.println(compassRaw);	    		


	// 	Serial2.print("compass: ");
	// 	Serial2.print(toDeg(tmp));
	// 	Serial2.print(" ");
	// 	Serial2.println(tmp);	    		

	// 	Serial2.print("Compass \"Calib\" offset: ");
	// 	Serial2.println(compassOffset);

	
	// 	//tmp += (M_PI+0.34);
	// 	tmp += (compassOffset+0.34);
	// 	if (tmp < -M_PI)
	// 	tmp = tmp + (2*M_PI);
	// 	if (tmp > M_PI)
	// 	tmp = tmp - (2*M_PI);

	    
	// 	Serial2.print("compass_corrected: ");
	// 	Serial2.print(toDeg(tmp));
	// 	Serial2.print(" ");
	// 	Serial2.println(tmp);


	  	
	//    	Serial2.print("gyro: ");
	// 	Serial2.print(toDeg(gyro_tmp));
	// 	Serial2.print(" ");
	// 	Serial2.println(gyro_tmp);


	// 	euler_z_offset=gyro_tmp-tmp;
	//     //correct for any wraps out of the -3.14 to 3.14
	// 	if (euler_z_offset < -PI)
	// 	euler_z_offset += 2*PI;

	// 	if (euler_z_offset > PI)
	// 	euler_z_offset -= 2*PI;
	  	

	//    	Serial2.print("offset: ");
	// 	Serial2.print(toDeg(euler_z_offset));
	// 	Serial2.print(" ");
	// 	Serial2.println(euler_z_offset);



	// 	Serial2.print("trueHeading: ");
	// 	Serial2.println(trueHeading);

	// }
}

void reportLocation()
{
	digitalWrite(45,HIGH);

	float voltage  = float(analogRead(67)/1024.l*5.l*10.1f);
	float amperage = float(analogRead(66)/1024.l*5.l*17.0f);
	manager.sendTelem(Protocol::telemetryType(LATITUDE),    location.degLatitude());
	manager.sendTelem(Protocol::telemetryType(LONGITUDE),   location.degLongitude());
	
	#ifdef simMode
	manager.sendTelem(Protocol::telemetryType(HEADING),     gps.getCourse());	
	#else	
	manager.sendTelem(Protocol::telemetryType(HEADING),     trueHeading);
	#endif

	//manager.sendTelem(Protocol::telemetryType(PITCH),       toDeg(pitch.get())-90);
	//manager.sendTelem(Protocol::telemetryType(ROLL),        toDeg(roll.get())-90);
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


// void calibrateGyro(){ //takes one second
// 	float tmp = 0;
// 	for(int i=0; i<100; i++)
// 	{
// 		float Gz = toDeg(mpu.gyroZ());
// 		tmp += Gz/100;
// 		delay(10);
// 	}
// 	lowFilter.set(tmp);
// }



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


void setupSettings()
{
	/*GROUNDSETTING index="0" name="Line Gravity" min="0" max="1" def="0.50"
	 *Defines how strongly the rover should attempt to return to the original
	 *course between waypoints, verses the direct path from its current location
	 * to the target<br>
	 */
	settings.attach(0, .50, callback<float, &lineGravity>);

	/*GROUNDSETTING index="1" name="Steer Throw" min="0" max="90" def="45"
	 *The number of degrees that rover will turn its wheels when it needs to
	 *to turn its most extreme amount
	 */
	settings.attach(1, 45, callback<int, &steerThrow>);

	/*GROUNDSETTING index="2" name="Steer Style" min="0" max="2" def="1"
	 *Switches between arctangent of error steering (0) <br>
	 *square of error steering (1) <br>
	 *and proportional to error steering (2)
	 */
	settings.attach(2, 1, callback<int, &steerStyle>);

	/*GROUNDSETTING index="3" name="Steer Scalar" min="0" max="8" def="1.5"
	 *Multiplier that determines how aggressively to steer
	 */
	settings.attach(3, 1.5, callback<float, &steerFactor>);


	/*GROUNDSETTING index="4" name="Min Fwd Speed" min="1" max="3" def="1.5"
	 *Minimum forward driving speed in MPH
	 */
	settings.attach(4, 1.5, callback<float, &minFwd>);

	/*GROUNDSETTING index="5" name="Max Fwd Speed" min="1.5" max="3" def="2.0"
	 *Maximum forward driving speed in MPH
	 */
	settings.attach(5, 2.0, callback<float, &maxFwd>);

	/*GROUNDSETTING index="6" name="Rev Str Throw" min="0" max="90" def="20"
	 *How far to turn the wheels when backing away from an obstacle
	 */
	settings.attach(6, 20, callback<int, &revThrow>);

	/*GROUNDSETTING index="7" name="Reverse Speed" min="-2" max="-1" def="-1.0"
	 *Speed in MPH to drive in reverse
	 */
	settings.attach(7, -1.0, callback<float, &revSpeed>);

	/*GROUNDSETTING index="8" name="Ping Factor" min="1" max="20000" def="1400"
	 *Factor to determine how strongly obstacles effect the rover's course <br>
	 *Larger numbers correspond to larger effects from obstacles
	 */
	settings.attach(8, 1400, callback<float, &pingWeight>);

	/*GROUNDSETTING index="9" name="Coast Time" min="2000" max="8000" def="2000"
	 *Time in milliseconds to coast before reversing when an obstacle is encountered
	 */
	settings.attach(9, 2000, callback<int, &avoidCoastTime>);

	/*GROUNDSETTING index="10" name="Min Rev Time" min="500" max="2000" def="1000"
	 *Minimum time in seconds to reverse away from an obstacle
	 */
	settings.attach(10, 1000, &dangerTimeCallback);

	/*GROUNDSETTING index="11" name="Cruise P" min="0" max="10" def="0.05"
	 *P term in cruise control PID loop
	 */
	settings.attach(11, 0.05, &newPIDparam);

	/*GROUNDSETTING index="12" name="Cruise I" min="0" max="10" def="0.1"
	 *I term in cruise control PID loop
	 */
	settings.attach(12, 0.1, &newPIDparam);

	/*GROUNDSETTING index="13" name="Cruise D" min="0" max="10" def="0.0"
	 *D term in cruise control PID loop
	 */
	settings.attach(13, 0.0, &newPIDparam);


	/*GROUNDSETTING index="14" name="Tire Diameter" min="0" max="12" def="5.85"
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

	//testing: gyro sync with north
	/*GROUNDSETTING index="18" name="GryoSync" min="0" max="1" def="0"
	 * Change to init gyro sync to north
	 */
	settings.attach(18, 0, &triggerGyroSyncNorth);	

//skew

	/*GROUNDSETTING index="57" name="Steer Skew" min="-45" max="45" def="0"
	 *
	 */
	settings.attach(20, 0, callback<float,&steerSkew>);

//ping 

	/*GROUNDSETTING index="58" name="Avoid Ping value Edges" min="500" max="10000" def="1000"
	 *
	 */
	settings.attach(21, 1000, &pingBlockLevelEdgesCallback);

	/*GROUNDSETTING index="59" name="Avoid Ping value Middles" min="500" max="10000" def="1600"
	 *
	 */
	settings.attach(22, 1600, &pingBlockLevelMiddlesCallback);

	/*GROUNDSETTING index="60" name="Avoid Ping value center" min="500" max="10000" def="3000"
	 *
	 */
	settings.attach(23, 3000, &pingBlockLevelCenterCallback);




	/*GROUNDSETTING index="61" name="Warn Ping value Edges" min="500" max="10000" def="2000"
	 *
	 */
	settings.attach(24, 2500, &pingWarnLevelEdgesCallback);

	/*GROUNDSETTING index="62" name="Warn Ping value Middles" min="500" max="10000" def="3200"
	 *
	 */
	settings.attach(25, 5000, &pingWarnLevelMiddlesCallback);

	/*GROUNDSETTING index="63" name="Warn Ping value center" min="500" max="10000" def="6000"
	 *
	 */
	settings.attach(26, 8000, &pingWarnLevelCenterCallback);

	


	settings.attach(27, compassOffset, callback<float,&compassOffset>);

}


 
