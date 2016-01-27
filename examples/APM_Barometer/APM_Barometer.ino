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

    baro.begin();
    delay(100);
    baro.calibrate();
}
void loop(){
    uint32_t time = -micros();
    baro.update();
    time += micros();

    float temp = baro.getCelsius();
    float bar  = baro.getMilliBar();
    float alt  = baro.getAltitude();

    Serial.print("temp: ");
    Serial.print(temp);
    Serial.print("\tmbar: ");
    Serial.print(bar);
    Serial.print("\talt: ");
    Serial.print(alt);
    Serial.print("\tin: ");
    Serial.print(time);
    Serial.println();
}
