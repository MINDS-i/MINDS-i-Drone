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

void setup(){
    eeList = EEPROMlist::getInstance();
    Serial.begin(9600);

    Serial1.begin(38400);
    sendGPSMessage(0x06, 0x01, 0x0003, GPRMC_On);
    sendGPSMessage(0x06, 0x17, 0x0004, CFG_NMEA);
    sendGPSMessage(0x06, 0x00, 0x0014, CFG_PRT);
    sendGPSMessage(0x06, 0x24, 0x0024, Pedestrian_Mode);

    Serial.println("Printing stored waypoints:");
    for(int i=0; i<eeList->size(); i++){
        Waypoint point = eeList->get(i);
        display(point);
    }
    Serial.println("Press C to clear");
}
void loop(){
    /*if(Serial.available()){
        float lat    = Serial.parseFloat();
        float lng    = Serial.parseFloat();
        uint16_t ext = Serial.parseInt();
        Waypoint fresh(lat,lng,ext);
        eeList->add(fresh);
        display(fresh);
    }*/

    if(Serial.available()){
        char c = Serial.read();
        if(c=='C') eeList->clear();
    }

    nmea.update();
    if(nmea.newData()){
        eeList->add(nmea.getLocation());
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
