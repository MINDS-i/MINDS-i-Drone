#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
const float INT_PERIOD = 5000;

Settings        settings(eeStorage::getInstance());
MpuSensor       mpu;
Sensor* sens[1] = {&mpu};
InertialManager sensors(sens, 1);
DualErrorParams params(1.0f, 1000.0f, 1000000.0f);
DualErrorFilter orientation(params);

PIDparameters tune(30.0f,400.0f,0.0f);
PIDcontroller pid(tune);
Servo output;

void isrCallback(){
    sensors.update(orientation);
}

void setup() {
    Serial.begin(9600);
    mpu.tuneAccl(settings.getAccelTune());
    sensors.start();
    sensors.calibrate();
    output.attach(A0);
    startInterrupt(isrCallback, INT_PERIOD);
    pid.set(0.0f);
}

void loop(){
    Quaternion attitude = orientation.getAttitude();
    float pitch = attitude.getPitch();
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(orientation.getAcclGain());
    Serial.print("\n");


    float angle = pid.calc(-pitch);
    angle = constrain(angle, -90, 90);
    output.write(angle+90);
}
