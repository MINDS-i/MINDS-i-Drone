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
*/
class OutputManager{
private:
	volatile boolean enabled, armed, standingby;
	OutputDevice* 	 (&output)[4];
	FlightStrategy*  flightMode;
public:
	OutputManager(OutputDevice*   (&mots)[4], FlightStrategy* mode)
		: output(mots), flightMode(mode) {}
	OutputManager(OutputDevice* (&mots)[4])
		: output(mots) {}
	/** Set the flightStrategy used to balance the aircraft */
	void setMode(FlightStrategy* mode){ flightMode = mode; }
	/**
	 * Calculate outputs based on enable-state and flightStrategy, then
	 * send them to the OutputDevices
	 */
	void update(OrientationEngine &orientation, float ms);
	/** Disable and spin down the motors, reset the flightStrategy */
	void disable();
	/** Keep the motors at idle, ready to fly but not applying torque */
	void standby();
	/** Enable flight, applying torques given by the flightStrategy */
	void enable();
	/** Have connected OutputDevices arm themselves; blocking */
	void arm();
	/** Have connected OutputDevices calibrate themselves; blocking */
	void calibrate();
};
void OutputManager::enable(){
	if(!armed) return;
	if(!enabled){
		flightMode->reset();
	}
	standingby = false;
	enabled = true;
}
void OutputManager::standby(){
	if(!enabled) enable();
	standingby = true;
}
void OutputManager::disable(){
	standingby = false;
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

	armed = true;
}
void OutputManager::update(OrientationEngine &orientation, float ms){
	if(!enabled || flightMode == NULL) {
		for(int i=0; i<4; i++){ output[i]->set(-1.0); }
		return;
	}
	if(standingby) {
		for(int i=0; i<4; i++){ output[i]->set(0.0); }
		return;
	}

	float impulses[4];
	flightMode->update(orientation,ms,impulses);
	impulses[3] *= 4.0f; // throttle split 4 ways in `solveOutputs`

	float outThrottle[4];
	solveOutputs(impulses, outThrottle);

	//set motor outputs
	for(int i=0; i<4; i++){
		outThrottle[i] = constrain(outThrottle[i], 0.0f, 1.0f);
		output[i]->set(outThrottle[i]);
	}
}
#endif
