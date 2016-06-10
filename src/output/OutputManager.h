#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include "math/Quaternion.h"
#include "math/Vec3.h"
#include "output/QuadCrossOutput.h"
#include "output/FlightStrategy.h"
#include "util/PIDparameters.h"

/*
OutputManager -Manages four OutputDevices
			  -Runs the assigned FlightStrategy to get desired torques
			  -translates body torques to motor outputs
			  -disable sends the neutral signal to motors, stops outputs
			  -stop sends the stop signal to the output devices

Motors should be in cross configuration, counting clockwise from the front left

  ^	Forward ^
  ---     ---
 | 0 |   | 1 |
  ---\ ^ / ---
      XXX
      XXX
  ---/   \---
 | 3 |   | 2 |
  ---     ---
  0 - counter clockwise
  1 - clockwise
  2 - counter clockwise
  3 - clockwise

  enable / disable - software level stopping and starting of motors
  				   - can be called repeatedly or back and forth safely
  arm/calibrate/stop - should be called a single time per power on
  				     - stop is not intended to be undone
*/
class OutputManager{
private:
	volatile boolean enabled, armed;
	OutputDevice* 	 (&output)[4];
	FlightStrategy*  flightMode;
public:
	OutputManager(OutputDevice*   (&mots)[4], FlightStrategy* mode)
		: output(mots), flightMode(mode) {}
	OutputManager(OutputDevice* (&mots)[4])
		: output(mots) {}
	void setMode(FlightStrategy* mode){ flightMode = mode; }
	void enable(); //use with caution; arming takes time
	void disable();
	void stop();
	void calibrate();
	void arm();
	void update(OrientationEngine &orientation);
};
void OutputManager::enable(){
	if(enabled) return;
	if(!armed) arm();
	for(int i=0; i<4; i++) output[i]->set(0.0);
	if(flightMode != NULL) flightMode->reset();
	enabled = true;
}
void OutputManager::disable(){
	for(int i=0; i<4; i++) output[i]->set(-1.0);
	enabled = false;
}
void OutputManager::stop(){
	for(int i=0; i<4; i++) output[i]->stop();
	enabled = false;
}
void OutputManager::arm(){
	uint32_t startTime = millis();
	for(int i=0; i<4; i++){
		output[i]->startArming();
	}

	//pass control around until all the ESC's are done arming
	boolean finished;
	do {
		finished = true;
		for(int i=0; i<4; i++){
			finished &= output[i]->continueArming( millis()-startTime );
		}
	} while (!finished);

	armed = true;
}
void OutputManager::calibrate(){
	//calibration will fail if the motors are already armed
	if(armed) return;

	uint32_t startTime = millis();
	for(int i=0; i<4; i++){
		output[i]->startCalibrate();
	}

	//pass control around until all the ESC's are done arming
	boolean finished = false;
	do{
		finished = true;
		for(int i=0; i<4; i++){
			finished &= output[i]->continueCalibrate( millis()-startTime );
		}
	} while (!finished);

	for(int i=0; i<4; i++){
		output[i]->set(-1.0);
	}

	enabled = true;
	armed   = true;
}
void OutputManager::update(OrientationEngine &orientation){
	//stop here if the outputs should all be off
	if(!enabled) return;
	if(flightMode == NULL) return;

	float impulses[4];
	flightMode->update(orientation,impulses);

	float outThrottle[4];
	impulses[3] *= 4.0f; //throttle split 4 ways
	solveOutputs(impulses, outThrottle);

	//set motor outputs
	for(int i=0; i<4; i++){
		outThrottle[i] = constrain(outThrottle[i], 0.0f, 1.0f);
		output[i]->set(outThrottle[i]);
	}
}
#endif
