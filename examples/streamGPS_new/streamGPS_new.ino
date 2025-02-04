#include "MINDS-i-Drone.h"

LEA6H gps;

void setup() {
    Serial.begin(115200);
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
        Serial.println(gps.getLatitude(), 9);
        Serial.print("gps.getLongitude()   ");
        Serial.println(gps.getLongitude(), 9);
        Serial.print("gps.getMagVar()      ");
        Serial.println(gps.getMagVar());
        Serial.print("gps.getTimeOfFix()   ");
        Serial.println(gps.getTimeOfFix());
        GPS_COORD tempGPSCoord = gps.getGPS_COORD();
        char str[32];
        print_gps(&Serial, "GPS_COORD: ", &tempGPSCoord);
        gps_coord_to_str(&tempGPSCoord, str, 32, 9, "DD");
        Serial.print("GPS: ");
        Serial.println(str);

        float_to_gps_angle(gps.getLatitude(), &tempGPSCoord.latitude);
        float_to_gps_angle(gps.getLongitude(), &tempGPSCoord.longitude);
        gps_coord_to_str(&tempGPSCoord, str, 32, 9, "DD");
        Serial.print("GPS (from float): ");
        Serial.println(str);

        Serial.println("-----");
        delay(500);
    }
}
