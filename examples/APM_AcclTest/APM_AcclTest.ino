#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

const float INT_PERIOD = 5000;

Settings        settings(eeStorage::getInstance());
MPU6000         mpu;
Sensor* sens[1] = {&mpu};
InertialManager sensors(sens, 1);
AcclOnly        orientation;

const float I = 4.0f;
float integral;
Servo output;

void isrCallback(){
    sensors.update(orientation);
}

void setup() {
    Serial.begin (9600);
    mpu.tuneAccl(settings.getAccelTune());
    sensors.start();
    sensors.calibrate();
    output.attach(A0);
    startInterrupt(isrCallback, INT_PERIOD);
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
