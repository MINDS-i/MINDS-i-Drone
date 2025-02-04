#include "MINDS-i-Drone.h"
#include "SPI.h"
#include "Wire.h"

#include "platforms/Ardupilot.h"
using namespace Platform;

const uint32_t UPDATE_INTERVAL = 5;
InertialVec* sens[2] = {&mpu, &hmc};
Translator conv[2] = {Translators::identity, Translators::identity};
InertialManager sensors(sens, conv, 2);

float accl[3];
float gyro[3];
float magn[3];

void setup() {
    beginAPM();
    Serial.begin(115200);
}

void display(float input) {
    Serial.print(input);
    Serial.print("\t");
}

void loop() {
    updateAPM();
    static auto timer = Interval::every(UPDATE_INTERVAL);
    if (timer()) {
        sensors.update();
        sensors.getLinAccel(accl[0], accl[1], accl[2]);
        sensors.getRotRates(gyro[0], gyro[1], gyro[2]);
        sensors.getMagField(magn[0], magn[1], magn[2]);

        display(millis());
        for (int i = 0; i < 3; i++)
            display(accl[i]);
        for (int i = 0; i < 3; i++)
            display(gyro[i] * 1000);
        for (int i = 0; i < 3; i++)
            display(magn[i]);

        Serial.print("\n");
    }
}
