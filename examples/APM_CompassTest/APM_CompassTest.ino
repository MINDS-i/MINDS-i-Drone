#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

#include "platforms/Ardupilot.h"
using namespace Platform;

HLA      angle( 75, 0); //half-life average; halflife = 75ms, init 0

ServoGenerator::Servo table;

void setup(){
    beginAPM();
    Serial.begin(9600);
    table.attach(A0);
}
void loop(){
    updateAPM();
    float tmp = hmc.getAzimuth();

    angle.update(tmp);
    Serial.println(angle.get(),DEC);
    table.write(toDeg(angle.get()));
}
