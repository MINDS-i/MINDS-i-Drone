#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

#include "platforms/Ardupilot.h"
using namespace Platform;

const float INT_PERIOD = 5000;

InertialVec*    sens[1] = {&mpu};
Translator      conv[1] = {Translators::APM};
InertialManager sensors(sens, conv, 1);

DualErrorFilter orientation(1.0f, 1000.0f, 1000000.0f);

PIDparameters tune(30.0f,400.0f,0.0f);
PIDcontroller pid(&tune);
ServoGenerator::Servo output;

void isrCallback(uint16_t microseconds){
    float ms = ((float)microseconds)/1000.0;
    sensors.update();
    orientation.update(sensors, ms);
}

void setup() {
    beginAPM();
    Serial.begin(9600);
    output.attach(A0);
    pid.set(0.0f);
    ServoGenerator::setUpdateCallback(isrCallback);
    ServoGenerator::begin(INT_PERIOD);
}

void loop(){
    updateAPM();
    Quaternion attitude = orientation.getAttitude();
    float pitch = attitude.getPitch();
    Serial.print(pitch);
    Serial.print("\t");

    float angle = pid.update(-pitch);
    angle = constrain(angle, -90, 90);
    output.write(angle+90);
}
