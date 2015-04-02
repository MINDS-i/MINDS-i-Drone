#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
/*
GPS, store position data every 2 seconds or so, if possible? The idea would be
to have them walk around and have it data log
as a standalone device no telemetry.
*/
List<Waypoint> *eeList;
NMEA            nmea(Serial1);
const uint32_t  UPDATE_INTERVAL = 50;

void setup(){
    eeList = EEPROMlist::getInstance();
    Serial.begin(9600);

    Serial1.begin(38400);
    sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
    sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
    sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
    sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);

    showList();
    Serial.println("Press C to clear");
}
void loop(){
    //Check if the list clear command is sent
    if(Serial.available()){
        char c = Serial.read();
        if(c=='C') eeList->clear();
        if(c=='S') showList();
    }

    //update the gps and store good readings periodically
    nmea.update();
    if(nmea.newData()){
        Waypoint newPoint = nmea.getLocation();
        static uint16_t count = 0;
        newPoint.setExtra(count++);//nmea.getWarning());

        static uint32_t intervalTimer;
        if(millis()>intervalTimer){
            intervalTimer += UPDATE_INTERVAL;
            addToList(newPoint);
        }
    }
}

void addToList(Waypoint point){
    if(eeList->size() == eeList->maxSize()) eeList->popTop();
    eeList->pushBottom(point);
}

void showList(){
    Serial.println("Printing all stored waypoints:");
    for(int i=0; i<eeList->size(); i++){
        Waypoint point = eeList->get(i);
        display(point);
    }
}

void display(Waypoint point){
    Serial.print(  "Lat:    ");
    Serial.print(point.degLatitude(),DEC);
    Serial.print("\tLon:    ");
    Serial.print(point.degLongitude(),DEC);
    Serial.print("\tExt:");
    Serial.print(point.getExtra(),DEC);
    Serial.print("\n");
}
