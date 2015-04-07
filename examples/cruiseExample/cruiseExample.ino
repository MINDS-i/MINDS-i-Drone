#include "Servo.h"
#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "DroneLibs.h"

const float maxMPH        = 6.0;
const float tireDiameter  = 5.85;
                            //tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
                            //hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }

PIDparameters param(0.05 ,0.1,0.0);
PIDcontroller cruise(param);
Servo drive, steer, backsteer;

void setup(){
    Serial.begin(9600);

    drive.attach(4);
    steer.attach(5);
    backsteer.attach(6);
    drive.write(90);
    steer.write(90);
    backsteer.write(90);
    delay(2000);

    encoder::begin(0, 1);
}

void loop(){
    float   mph = ((getRadio(2)-90) / 90.f)*maxMPH;
    uint8_t str = getRadio(3);

    if(abs(mph)<0.5f){
        cruise.stop();
    } else {
        cruise.set(MPHtoRPM(mph));
    }

    float output = cruise.calc(encoder::getRPM());
    drive.write(90+output);
    steer.write(str);
    backsteer.write(180-str);

    Serial.print(90+output);
    Serial.print("\t");
    Serial.print(str);
    Serial.print("\n");

    delay(2);
}
