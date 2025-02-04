#include "MINDS-i-Drone.h"

LEA6H gps;

void setup() {
    Serial.begin(9600);
    gps.begin();
}

void loop() {
    gps.update();
    static size_t lastGpsDataIndex = gps.dataIndex();
    if (gps.dataIndex() > lastGpsDataIndex) {
        lastGpsDataIndex = gps.dataIndex();
        Serial.print("gps.getWarning()     ");
        Serial.println(gps.getWarning());
        Serial.print("gps.getCourse()      ");
        Serial.println(gps.getCourse());
        Serial.print("gps.getDateOfFix()   ");
        Serial.println(gps.getDateOfFix());
        Serial.print("gps.getGroundSpeed() ");
        Serial.println(gps.getGroundSpeed());
        Serial.print("gps.getLatitude()    ");
        Serial.println(gps.getLatitude());
        Serial.print("gps.getLongitude()   ");
        Serial.println(gps.getLongitude());
        Serial.print("gps.getMagVar()      ");
        Serial.println(gps.getMagVar());
        Serial.print("gps.getTimeOfFix()   ");
        Serial.println(gps.getTimeOfFix());
        Serial.println("-----");
        delay(500);
    }
}
