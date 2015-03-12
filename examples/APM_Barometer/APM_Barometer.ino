#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
/*
Altimeter, get a raw value and maybe a converted value to feet?
(through the Arduino Serial Terminal)
*/

MS5611 baro;

void setup(){
    Serial.begin(9600);
    
    baro.init();
    delay(100);
    baro.calibrate();
}
void loop(){
    Serial.print(baro.getTemp()); Serial.print("\t");
    Serial.print(baro.getMilliBar()); Serial.print("\t");
    Serial.print("\n");
}
