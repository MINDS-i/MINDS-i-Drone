#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const int UPDATE_INTERVAL = 1000; //ms between transmits
HardwareSerial *commSerial  = &Serial;
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
}
void loop(){
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

    //header line
    commSerial->print("\n---- ");
    commSerial->print(nmea.getTimeOfFix());
    commSerial->print("\t--------------\n");

    //gps location lines
    commSerial->print(  "Lat:    ");
    commSerial->print(location.degLatitude(),DEC);
    commSerial->print("\nLon:    ");
    commSerial->print(location.degLongitude(),DEC);
    commSerial->print("\n");

    //course and speed line
    commSerial->print(  "Course: ");
    commSerial->print(nmea.getCourse());
    commSerial->print("\tSpeed:  ");
    commSerial->print(nmea.getGroundSpeed());
    commSerial->print("\n");

    //angles
    commSerial->print(  "Pitch:  ");
    commSerial->print(pitch.get()*180/PI);
    commSerial->print("\tRoll:   ");
    commSerial->print(roll.get()*180/PI );
    commSerial->print("\n");

    //voltage and runtime
    commSerial->print(  "Voltage:");
    commSerial->print(voltage);
    commSerial->print("\tupTime: ");
    commSerial->print(millis());
    commSerial->print("\n");
}
