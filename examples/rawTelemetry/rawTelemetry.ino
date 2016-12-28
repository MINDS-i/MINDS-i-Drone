#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

const int UPDATE_INTERVAL = 1000; //ms between transmits
HardwareSerial *commSerial  = &Serial;
Waypoint        location(0,0);
LEA6H           gps;
MPU6000         mpu;
HLA             pitch( 100, 0);
HLA             roll ( 100, 0);

void setup(){
    commSerial->begin(Protocol::BAUD_RATE);
    mpu.begin();
    gps.begin();
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
    long Ax = mpu.acclX();
    long Ay = mpu.acclY();
    long Az = mpu.acclZ();
    pitch.update( atan2(sqrt(Ax*Ax+Az*Az), Ay) );
    roll .update( atan2(sqrt(Ay*Ay+Az*Az),-Ax) );
}
void updateGPS(){
    gps.update();
    static size_t lastGpsDataIndex = gps.dataIndex();
    if(gps.dataIndex() > lastGpsDataIndex){
        lastGpsDataIndex = gps.dataIndex();
        location = gps.getLocation();
    }
}
void reportTelemetry(){
    float voltage = float(analogRead(67)/1024.l*5.l*10.1l);

    //header line
    commSerial->print("\n---- ");
    commSerial->print(gps.getTimeOfFix());
    commSerial->print("\t--------------\n");

    //gps location lines
    commSerial->print(  "Lat:    ");
    commSerial->print(location.degLatitude(),DEC);
    commSerial->print("\nLon:    ");
    commSerial->print(location.degLongitude(),DEC);
    commSerial->print("\n");

    //course and speed line
    commSerial->print(  "Course: ");
    commSerial->print(gps.getCourse());
    commSerial->print("\tSpeed:  ");
    commSerial->print(gps.getGroundSpeed());
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
