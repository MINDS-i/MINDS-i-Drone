#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"

void setup(){
    APMRadio::setup();
    Serial.begin(9600);
}

void loop(){
    static auto timer = Interval::every(100);
    if(timer()){
        for(int i=0; i<8; i++){
            Serial.print(APMRadio::get(i));
            Serial.print(" ");
        }
        Serial.println();
    }
}
