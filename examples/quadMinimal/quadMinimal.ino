#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
#include "platforms/Quadcopter.h"

void setup() {
    setupQuad();
    output.setMode(&horizon);
}

void loop() {
    //always running updates
    loopQuad();
    sendTelemetry();
}

void sendTelemetry(){
    static uint32_t sendTime = millis();
    if(sendTime < millis()){
        sendTime += 50;

        Quaternion attitude = orientation.getAttitude();
        using namespace Protocol;
        //comms.sendTelem(LATITUDE   , horizon.testPoint*100.0);
        //comms.sendTelem(LONGITUDE  , ((float)getAPM2Radio(RADIO_YAW)-90)/-90.0);
        comms.sendTelem(HEADING    , toDeg(orientation.getYaw()));
        comms.sendTelem(PITCH      , toDeg(orientation.getPitch()));
        comms.sendTelem(ROLL       , toDeg(orientation.getRoll()));
        comms.sendTelem(GROUNDSPEED, profile[0]);
        comms.sendTelem(VOLTAGE    , esc[0].get());

        Serial.println();
        Serial.flush();
    }
}
