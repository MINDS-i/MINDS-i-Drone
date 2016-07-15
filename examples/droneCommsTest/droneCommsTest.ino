#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

#include "platforms/Ardupilot.h"
using namespace Platform;

Waypoint loc;
uint32_t time;
uint32_t walkInterval;

void setupSettings(){
	settings.attach(0, 1000.f, callback<uint32_t, &walkInterval>);
}
void setup(){
	beginAPM();
	Serial.begin(9600);
	delay(500);
	setupSettings();
	comms.requestResync();
	time = millis();
}

void loop(){
	updateAPM();
	if(time <= millis()){
		time += walkInterval;

		loc = comms.getTargetWaypoint();
		comms.sendTelem(Protocol::telemetryType(LATITUDE) , loc.degLatitude());
		comms.sendTelem(Protocol::telemetryType(LONGITUDE), loc.degLongitude());
		comms.sendTelem(2, comms.numWaypoints());

		comms.sendString("Hello from the arduino!");

		//advance waypoint list
		if(comms.getTargetIndex() < comms.numWaypoints()-1){
			comms.advanceTargetIndex();
		}
		else if (comms.loopWaypoints()){
			comms.setTargetIndex(0);
		}
	}
}
