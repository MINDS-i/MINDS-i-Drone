#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
/*
Compass, make the compass point at a specific heading.
*/
MpuSensor  mpu;
APMCompass cmp;
InertialSensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
Settings        set(eeStorage::getInstance());
HLA             angle( 200, 0); //half-life average; halflife = 200ms, init 0
Servo table;
const float I_VAL = .04;
float accumulator;

void setup(){
    Serial.begin(57600);
    mpu.tuneAccl(set.getAccelTune());
    cmp.tune(set.getMagTune());
    sensors.start();
    delay(1000);
    sensors.calibrate();
    
    table.attach(A0);
}
void loop(){
	sensors.update();
    float val[3];
    sensors.getMagField(val[0], val[1], val[2]);
    for(int i=0; i<3; i++){
        Serial.print(val[i]);
        Serial.print('\t');
    }
    float tmp = atan2(val[0], val[1]);
    angle.update(tmp);
    Serial.println(angle.get(),DEC);
    
    accumulator += I_VAL*toDeg(angle.get());
    accumulator = trunkAngle(accumulator);
    table.write(toDeg(angle.get()));
    
    delay(20);
}
