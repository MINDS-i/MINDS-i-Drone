#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

HardwareSerial *commSerial	= &Serial;
Storage<float> *storage	= eeStorage::getInstance();
CommManager		manager(commSerial, storage);
Settings		settings(storage);
Waypoint		loc;
uint32_t 		time;
uint32_t		walkInterval;

void setupSettings(){
	using namespace AirSettings;
	settings.attach(INT_PERIOD , 1000.f, callback<uint32_t, &walkInterval>);
}
void setup(){
	setupSettings();
	Serial.begin(9600);
	delay(500);
	manager.requestResync();
	time = millis();
}

void loop(){
	manager.update();
	if(time <= millis()){
		time += walkInterval;

		loc = manager.getTargetWaypoint();
		using namespace DroneProtocol;
		manager.sendTelem(telemetryType(LATITUDE) , loc.degLatitude());
		manager.sendTelem(telemetryType(LONGITUDE), loc.degLongitude());
		manager.sendTelem(2, manager.numWaypoints());

		manager.sendString("Hello from the arduino!");

		//advance waypoint list
		if(manager.getTargetIndex() < manager.numWaypoints()-1){
			manager.advanceTargetIndex();
		}
		else if (manager.loopWaypoints()){
			manager.setTargetIndex(0);
		}
	}
}
