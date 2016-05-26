#include "SPI.h"
#include "Wire.h"
#include "MINDSi.h"
#include "Encoder.h"
#include "DroneLibs.h"

const uint8_t EncoderPin[]= { 2,  3 }; //apm 6,7
const uint8_t ServoPin[]  = {12, 11,  8};//drive, steer, backS; APM 1,2,3 resp.
const uint8_t RadioPin[]  = { 0,  1 }; //drive, steer
const float maxMPH        = 6.0;
const float tireDiameter  = 5.85;
                            //tire circ in miles per inch diameter * diff ratio
const float MilesPerRev   = (((PI)/12.f)/5280.f) * (13.f/37.f);
                            //hours per min      rev per mile
const float MPHvRPM       = (1.f/60.f)        * (1.f/MilesPerRev);
inline float MPHtoRPM(float mph){ return (mph*MPHvRPM)/tireDiameter; }
inline float RPMtoMPH(float rpm){ return (rpm*tireDiameter)/MPHvRPM; }

PIDparameters param(0.05 ,0.1,0.0);
PIDcontroller cruise(&param);
ServoGenerator::Servo drive, steer, backsteer;

void setup(){
    Serial.begin(9600);
    ServoGenerator::setup(20000);

    drive.attach(ServoPin[0]);
    steer.attach(ServoPin[1]);
    backsteer.attach(ServoPin[2]);
    drive.write(90);
    steer.write(90);
    backsteer.write(90);
    delay(2000);

    setupAPM2radio();
    encoder::begin(EncoderPin[0], EncoderPin[1]);
}

void loop(){
    /* for a leonardo w/ shield, use getRadio, and plug the radio into 0 and 1
    float   mph = ((getRadio(RadioPin[1])-90) / 90.f)*maxMPH;
    uint8_t str =   getRadio(RadioPin[2]);
    */
    float   mph = ((getAPM2Radio(RadioPin[1])-90) / 90.f)*maxMPH;
    uint8_t str =   getAPM2Radio(RadioPin[2]);

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
