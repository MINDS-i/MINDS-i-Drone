#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
/*
GPS, store position data every 2 seconds or so, if possible? The idea would be 
to have them walk around and have it data log 
as a standalone device no telemetry.
*/
void setup(){
    Serial.begin(9600);
    Serial.println("Ready!");
}
void loop(){
    while(!Serial.available());
    while(Serial.available()) { Serial.read(); delay(50); }
    runEEListTest();
    Serial.print("\nDone\n");
}
