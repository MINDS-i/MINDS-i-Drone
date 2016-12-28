#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

List<Waypoint> *eeList;
const uint32_t  UPDATE_INTERVAL = 4000;
LEA6H           gps;

void setup(){
    eeList = EEPROMlist::getInstance();
    Serial.begin(9600);
    gps.begin();

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
    gps.update();
    static size_t lastGpsDataIndex = gps.dataIndex();
    if((gps.dataIndex() > lastGpsDataIndex) && gps.status().good()){
        lastGpsDataIndex = gps.dataIndex();
        Waypoint newPoint = gps.getLocation();
        static uint32_t intervalTimer;
        if(millis()>intervalTimer){
            intervalTimer = millis()+UPDATE_INTERVAL;
            static uint16_t count = 0;
            newPoint.setExtra(count++);
            addToList(newPoint);
        }
    }
}

void addToList(Waypoint point){
    if(eeList->size() == eeList->maxSize()) eeList->popTop();
    eeList->pushBottom(point);
    Serial.println("Adding point");
}

void showList(){
    Serial.print("Printing all ");
    Serial.print(eeList->size());
    Serial.print(" stored waypoints:");
    Serial.println();
    for(size_t i=0; i<eeList->size(); i++){
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
