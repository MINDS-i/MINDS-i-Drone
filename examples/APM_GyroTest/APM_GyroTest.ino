#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
const float INT_PERIOD = 5000;

Settings        settings(eeStorage::getInstance());
MpuSensor       mpu;
InertialSensor* sens[1] = {&mpu};
InertialManager sensors(sens, 1);
GyroOnly        orientation;

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

    uint8_t oldSREG = SREG;
    cli();
    uint32_t p = -micros();
    float pitch = attitude.getPitch();
    float roll  = attitude.getRoll();
    p += micros();

    uint32_t q = -micros();
    Vec3 ref(0,0,-1);
    ref.rotateBy(attitude);
    float vPitch = asin(ref[0]);
    float vRoll  = asin(-ref[1]);
    q += micros();
    SREG = oldSREG;

    Serial.print(p);
    Serial.print("\t");
    Serial.print(q);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print(roll);
    Serial.print(vPitch);
    Serial.print("\n");


    integral += I*pitch;
    integral = constrain(integral, -90, 90);
    output.write(integral+90);
}
