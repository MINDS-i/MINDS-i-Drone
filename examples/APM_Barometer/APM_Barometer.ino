#include "MINDS-i-Drone.h"
#include "SPI.h"
#include "Wire.h"

MS5611 baro;
Altitude altitude;
SRAMstorage<float, 2> storage;
Settings settings(&storage);
CommManager comms(&Serial, &storage);

void setup() {
    Serial.begin(9600);

    baro.begin();
    delay(100);
    baro.calibrate();

    settings.attach(0, 0.046f, [](float g) { altitude.setBarometerGain(g); });
    settings.attach(1, 0.020f, [](float g) { altitude.setVelocityGain(g); });
}

void loop() {
    comms.update();
    baro.update();
    altitude.update(baro.getAltitude());

    static auto timer = Interval::every(250);
    if (timer()) {
        float temp = baro.getCelsius();
        float bar = baro.getMilliBar();
        float alt = baro.getAltitude();

        comms.sendTelem(GROUNDSPEED, altitude.getVelocity());
        comms.sendTelem(VOLTAGE, temp);
        comms.sendTelem(AMPERAGE, bar);
        comms.sendTelem(ALTITUDE, altitude.getAltitude());
        comms.sendTelem(ALTITUDE + 1, alt);
    }
}
