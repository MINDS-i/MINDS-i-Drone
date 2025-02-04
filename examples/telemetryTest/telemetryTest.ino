#include "MINDS-i-Drone.h"
#include "SPI.h"
#include "Wire.h"

#include "platforms/Ardupilot.h"
using namespace Platform;

const int UPDATE_INTERVAL = 100; // ms between telemetry transmits
const int MSG_INTERVAL = 30000;  // ms between message transmits
Waypoint location(0, 0);
HLA pitch(100, 0);
HLA roll(100, 0);

void setup() {
    beginAPM();
    comms.requestResync();
}

void loop() {
    updateAPM();
    updateGPS();
    readAccelerometer();
    static uint32_t outputTrack = millis();
    if (millis() > outputTrack) {
        outputTrack += UPDATE_INTERVAL;
        reportTelemetry();
    }

    /*#MSG_TEST This is a test of a robot transmitted string */
    static uint32_t msgTrack = millis();
    if (millis() > msgTrack) {
        msgTrack += MSG_INTERVAL;
        comms.sendString("MSG_TEST");
    }
}
void readAccelerometer() {
    long Ax = mpu.acclX();
    long Ay = mpu.acclY();
    long Az = mpu.acclZ();
    pitch.update(atan2(sqrt(Ax * Ax + Az * Az), Ay));
    roll.update(atan2(sqrt(Ay * Ay + Az * Az), -Ax));
}
void updateGPS() {
    gps.update();
    static size_t lastGpsDataIndex = gps.dataIndex();
    if (gps.dataIndex() > lastGpsDataIndex) {
        lastGpsDataIndex = gps.dataIndex();
        location = gps.getLocation();
    }
}
void reportTelemetry() {
    using namespace Protocol;
    float voltage = float(analogRead(67) / 1024.l * 5.l * 10.1l);
    comms.sendTelem(telemetryType(LATITUDE), location.degLatitude());
    comms.sendTelem(telemetryType(LONGITUDE), location.degLongitude());
    comms.sendTelem(telemetryType(HEADING), gps.getCourse());
    comms.sendTelem(telemetryType(PITCH), pitch.get() * 180 / PI);
    comms.sendTelem(telemetryType(ROLL), roll.get() * 180 / PI);
    comms.sendTelem(telemetryType(GROUNDSPEED), gps.getGroundSpeed());
    comms.sendTelem(telemetryType(VOLTAGE), voltage);
}
