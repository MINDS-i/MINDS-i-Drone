#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
/*
Altimeter, get a raw value and maybe a converted value to feet?
(through the Arduino Serial Terminal)
*/

MS5611 baro;
SRAMstorage<float,2> storage;
Settings settings(&storage);
CommManager comms(&Serial, &storage);
Altitude altitude;

void setup(){
    Serial.begin(9600);

    baro.begin();
    delay(100);
    baro.calibrate();

    altitude.setup(0.0);
    settings.attach(0, 0.05, [](float g){ altitude.setBarometerGain(g); });
    settings.attach(1, 0.02, [](float g){ altitude.setVelocityGain(g); });
}
void loop(){
    comms.update();
    baro.update();
    altitude.update(baro.getAltitude());

    static auto timer = Interval::every(10);
    if(timer()){
        float temp = baro.getCelsius();
        float bar  = baro.getMilliBar();
        float alt  = baro.getAltitude();

        comms.sendTelem(GROUNDSPEED, altitude.getVelocity());
        comms.sendTelem(VOLTAGE, temp);
        comms.sendTelem(AMPERAGE, bar);
        comms.sendTelem(ALTITUDE, alt);
        comms.sendTelem(ALTITUDE+1, altitude.getAltitude());
    }
}
