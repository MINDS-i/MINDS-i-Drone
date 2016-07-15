#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

#include "platforms/Ardupilot.h"
using namespace Platform;


InertialVec*    sens[1] = {&mpu};
Translator      conv[1] = {Translators::APM};
InertialManager sensors(sens, conv, 1);
AcclOnly        orientation;

ServoGenerator::Servo output;

const float I = 4.0f;
float integral;

void isrCallback(uint16_t microseconds){
    float ms = ((float)microseconds)/1000.0;
    sensors.update();
    orientation.update(sensors, ms);
}

void setup() {
    beginAPM();
    Serial.begin(9600);
    output.attach(A0);
    ServoGenerator::setUpdateCallback(isrCallback);
}

void loop(){
    updateAPM();
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
