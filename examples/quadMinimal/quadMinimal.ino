#include "Wire.h"
#include "SPI.h"
#include "Servo.h"
#include "DroneLibs.h"
#include "platforms/Quadcopter.h"

#include "MINDSi.h"

void setup() {
    setupQuad();
}

void loop() {
    //always running updates
    loopQuad();
    sendTelemetry();
}

void sendTelemetry(){
/*    static uint32_t sendTime = millis();
    if(sendTime < millis()){
*/
        using namespace Protocol;

        sensors.update();
        orientation.update(sensors);

        Vec3 accl = sensors.getAccl();
        accl.rotateBy(~orientation.getAttitude());

        comms.sendTelem(0, millis());
        comms.sendTelem(1, baro.getAltitude());
        comms.sendTelem(2, accl[2]);
        comms.sendTelem(3, getPing(A0));

/*        sendTime += 20; //100Hz
    }*/
}
