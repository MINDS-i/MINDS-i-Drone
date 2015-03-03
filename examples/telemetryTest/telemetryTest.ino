#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
/*
Example to just send telemetry to the dashboard
*/
const int UPDATE_INTERVAL = 1000; //ms between transmits
HardwareSerial *commSerial  = &Serial;
Storage<float> *storage = eeStorage::getInstance();
CommManager     manager(commSerial, storage);
NMEA            nmea(Serial1);
Waypoint        location(0,0);
HLA             pitch( 100, 0);
HLA             roll ( 100, 0);

void setup(){
	Serial1.begin(38400);
    commSerial->begin(Protocol::BAUD_RATE);
    InitMPU();
    pinMode(40, OUTPUT); digitalWrite(40, HIGH); //SPI select pin
    
    //configure GPS
    sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
    sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
    sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
    sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);
    
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
    nmea.update();
    if(nmea.newData()){
        location = nmea.getLocation();
    }
}
void reportTelemetry(){
    float voltage = float(analogRead(67)/1024.l*5.l*10.1l);
    manager.sendTelem(Protocol::telemetryType(LATITUDE),  location.degLatitude());
    manager.sendTelem(Protocol::telemetryType(LONGITUDE), location.degLongitude());
    manager.sendTelem(Protocol::telemetryType(HEADING),   nmea.getCourse());
    manager.sendTelem(Protocol::telemetryType(PITCH),     pitch.get()*180/PI);
    manager.sendTelem(Protocol::telemetryType(ROLL),      roll.get()*180/PI);
    manager.sendTelem(Protocol::telemetryType(SPEED),     nmea.getGroundSpeed());
    manager.sendTelem(Protocol::telemetryType(VOLTAGE),   voltage);
}
