#include "MINDS-i-Drone.h"
#include "SPI.h"
#include "Wire.h"

#include "platforms/Quadcopter.h"
using namespace Platform;

void setup() { beginMultirotor(); }

void loop() {
    // always running updates
    updateMultirotor();
    sendTelemetry();
}

void sendTelemetry() {
    static auto timer = Interval::every(250);
    if (timer()) {
        using namespace Protocol;
        comms.sendTelem(HEADING, toDeg(orientation.getYaw()));
        comms.sendTelem(PITCH, toDeg(orientation.getPitch()));
        comms.sendTelem(ROLL, toDeg(orientation.getRoll()));
        Serial.println();
        Serial.flush();
    }
}
