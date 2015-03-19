#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"

APMCompass cmp;
Settings        set(eeStorage::getInstance());
HLA             angle( 75, 0); //half-life average; halflife = 75ms, init 0

Servo table;

void setup(){
    Serial.begin(57600);
    cmp.tune(set.getMagTune());
    table.attach(A0);
}
void loop(){
    float tmp = cmp.getAzimuth();

    angle.update(tmp);
    Serial.println(angle.get(),DEC);
    table.write(toDeg(angle.get()));
}
