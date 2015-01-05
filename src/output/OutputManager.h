#ifndef OUTPUT_MANAGER_H
#define OUTPUT_MANAGER_H

#include "output/OutputSolverCross.h"
#include "util/PIDparameters.h"

/*
OutputManager -Manages four OutputDevices (North, East, West, South)
			  -Calculates modified PI(D')
			  -Calculates and sets outputs using generated LU factorizations
			  -Feeds back information on the output state to OrientationEngine
*/
class OutputManager{
private:
	static const float LINEAR_SCALE_FACTOR = 4096; //180*32; LSF < 2^13

	volatile boolean enabled;
	volatile float 	 desiredState[4];
	OutputDevice** 	 output;
	PIDparameters 	 PID[2];
	float 			 RMS;
	boolean			 stopped;
public:
	OutputManager(OutputDevice*  NEWS[4], //North, East, West, South
				  PIDparameters  PitchPID,
				  PIDparameters  RollPID ,
				  float predictionRMS	 ): output(NEWS),
											PID{PitchPID, RollPID},
											RMS(predictionRMS)
											{};
	OutputManager(OutputDevice* NEWS[4]) : output(NEWS),
										   RMS(-1.0f)
										   {}
	void set(float pitch, float roll, float dYaw, float throttle);
	void enable(); //use with caution; arming takes time
	void disable();
	void stop();
	void start();
	void update(OrientationEngine &orientation);
	void setPitchPID(PIDparameters inputPID);
	void setRollPID (PIDparameters inputPID);
	void setFeedbackRMS(float input);
};
void OutputManager::set(float pitch, float roll, float dYaw, float throttle){
	desiredState[0] = pitch;
	desiredState[1] = roll;
	desiredState[2] = dYaw;
	desiredState[3] = throttle;
	stopped = false;
}
void OutputManager::enable(){ //use with caution; arming takes time
	uint32_t startTime = millis();
	for(int i=0; i<4; i++){
		output[i]->startArming();
	}

	//pass control around until all the ESC's are done arming
	boolean finishedArming = false;
	while( !finishedArming ){
		finishedArming = true;
		for(int i=0; i<4; i++){
			finishedArming &= output[i]->continueArming( millis()-startTime );
		}
	}

	for(int i=0; i<4; i++){
		output[i]->set(0.);
	}
	enabled = true;
	stopped = false;
}
void OutputManager::disable(){
	for(int i=0; i<4; i++){
		output[i]->set(0.);
		output[i]->stop();
	}
	enabled = false;
}
void OutputManager::stop(){
	stopped = true;
}
void OutputManager::start(){
	stopped = false;
}
void OutputManager::update(OrientationEngine &orientation){
	int impulses[4];
	int outThrottle[4];

	//stop here if the outputs should all be off
	if(stopped || (impulses[3]<.00001)){
		for(int i=0; i<4; i++){
			output[i]->set(0.0);
		}
		PID[0].acc = 0;
		PID[1].acc = 0;
		return;
	}

	//Set useful variables
	math::quaternion state = orientation.getLastAttitude();
	double q0 = (state.Scalar());
	double q1 = (state.Vector().x);
	double q2 = (state.Vector().y);
	double q3 = (state.Vector().z);
	float angles[2], angleRate[2];
	angles[0]    = orientation.getRoll ();
	angles[1]    = orientation.getPitch();
	angleRate[0] = orientation.getRate().y;
	angleRate[1] = orientation.getRate().x;

	//calculate PID based impulses
	float PIDout[2];
	for(int i=0; i<2; i++){
		float error   = (desiredState[i]-angles[i]);
		PID[i].acc += error;
		float val =   PID[i].P*error
					+ PID[i].I*PID[i].acc
					+ PID[i].D*angleRate[i];
		PIDout[i] = val*LINEAR_SCALE_FACTOR;
	}
	impulses[0] = PIDout[0]*LINEAR_SCALE_FACTOR;
	impulses[1] = PIDout[1]*LINEAR_SCALE_FACTOR;
	impulses[2] = desiredState[2]*LINEAR_SCALE_FACTOR;
	impulses[3] = desiredState[3]*LINEAR_SCALE_FACTOR*4.;

	//run generated LU output calculation code
	solveOutputs(impulses, outThrottle);

	//set motor outputs
	for(int i=0; i<4; i++){
		constrain(0, outThrottle[i], (int)LINEAR_SCALE_FACTOR);
		float out = ((float)outThrottle[i])/LINEAR_SCALE_FACTOR;
		output[i]->set(out);
	}

	//inform OrientationEnigne of output's predicted effects
	if(RMS > 0.) {
		math::vector3d prediction(PIDout[0], PIDout[1], desiredState[2]);
		orientation.updateRate(prediction, RMS);
	}
}
void OutputManager::setPitchPID(PIDparameters inputPID){
	PID[0] = inputPID;
}
void OutputManager::setRollPID(PIDparameters inputPID){
	PID[1] = inputPID;
}
void OutputManager::setFeedbackRMS(float input){
	RMS = input;
}
#endif
