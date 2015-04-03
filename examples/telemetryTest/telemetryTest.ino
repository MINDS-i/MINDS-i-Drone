#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const int UPDATE_INTERVAL = 100; //ms between transmits
HardwareSerial *commSerial  = &Serial;
Storage<float> *storage = eeStorage::getInstance();
CommManager     manager(commSerial, storage);
LEA6H           gps;
Waypoint        location(0,0);
HLA             pitch( 100, 0);
HLA             roll ( 100, 0);

void setup(){
    commSerial->begin(Protocol::BAUD_RATE);
    InitMPU();
    pinMode(40, OUTPUT); digitalWrite(40, HIGH); //SPI select pin

    gps.init();

    manager.requestResync();
}
void loop(){
	manager.update();
    updateGPS();
    readAccelerometer();
    static uint32_t outputTrack = millis();
    if(millis() > outputTrack){
        outputTrack += UPDATE_INTERVAL;
        reportTelemetry();
    }
}
void readAccelerometer(){
    long Ax = MPU_Ax();
    long Ay = MPU_Ay();
    long Az = MPU_Az();
    pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
    roll .update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
}
void updateGPS(){
    gps.update();
    if(gps.newData()){
        location = gps.getLocation();
    }
}
void reportTelemetry(){
    using namespace Protocol;
    float voltage = float(analogRead(67)/1024.l*5.l*10.1l);
    manager.sendTelem(telemetryType(LATITUDE),    location.degLatitude());
    manager.sendTelem(telemetryType(LONGITUDE),   location.degLongitude());
    manager.sendTelem(telemetryType(HEADING),     gps.getCourse());
    manager.sendTelem(telemetryType(PITCH),       pitch.get()*180/PI);
    manager.sendTelem(telemetryType(ROLL),        roll.get()*180/PI);
    manager.sendTelem(telemetryType(GROUNDSPEED), gps.getGroundSpeed());
    manager.sendTelem(telemetryType(VOLTAGE),     voltage);
}
