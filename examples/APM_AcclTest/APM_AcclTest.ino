#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

const float INT_PERIOD = 5000;

Settings        settings(eeStorage::getInstance());
MPU6000         mpu;
InertialVec*    sens[1] = {&mpu};
Translator      conv[1] = {Translators::APM};
InertialManager sensors(sens, conv, 1);
AcclOnly        orientation;

const float I = 4.0f;
float integral;
ServoGenerator::Servo output;

void isrCallback(uint16_t microseconds){
    float ms = ((float)microseconds)/1000.0;
    sensors.update();
    orientation.update(sensors, ms);
}

void setup() {
    Serial.begin(9600);
    mpu.tuneAccl(settings.getAccelTune());
    sensors.start();
    sensors.calibrate();
    output.attach(A0);
    ServoGenerator::setUpdateCallback(isrCallback);
    ServoGenerator::begin(INT_PERIOD);
}

void loop(){
    Quaternion attitude = orientation.getAttitude();
    float pitch = attitude.getPitch();
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(integral);
    Serial.print("\n");

    integral += I*pitch;
    integral = constrain(integral, -90, 90);
    output.write(integral+90);
}
