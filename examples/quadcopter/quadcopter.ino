#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
#include "platforms/Quadcopter.h"

const float GsToFeet = -32.17;
boolean calibrated = false;
boolean altHold    = false;
float altitudeSetpoint = 0;
PIDcontroller altPID(&altHoldParams);
uint32_t calStartTime;
const uint8_t CHANNEL_MIN = 32;
const uint8_t CHANNEL_MAX = 148;
float yawTarget = 0.0;
enum State {
    DISARMED,
    CALIBRATE,
    FLYING
};
const char* stateString[] = {
    [DISARMED] = "DISARMED", [CALIBRATE] = "CALIBRATE", [FLYING] = "FLYING" };
State state;

enum RadioChannel{
    RADIO_PITCH    = 0,
    RADIO_ROLL     = 1,
    RADIO_YAW      = 3,
    RADIO_THROTTLE = 2
};
typedef bool (*boolfunc)();
template<boolfunc Func> uint32_t timeState();

void setup() {
    setupQuad();
    setState(DISARMED);
}

void setState(State s){
    state = s;
    comms.sendString(stateString[s]);
}

void loop() {
    //always running updates
    loopQuad();
    sendTelemetry();

    //flight mode state machine
    switch(state){
        /*#DISARMED Hold Down and to the right on
         * the Throttle stick for two seconds to begin the arming process
        **/
        case DISARMED:
            if(timeState<&radio_downRight>() > 2000) {
                if(safe()){
                    calStartTime = millis();
                    setState(CALIBRATE);
                } else {
                    /*#NOFLY Flight aborted due to error status */
                    comms.sendString("NOFLY");
                }
            }
            break;
        /*#CALIBRATE Calibrating the gyroscope.
         * This lasts for two seconds. Keep the quadcopter as still as possible.
        **/
        case CALIBRATE:
            orientation.calibrate(true);
            if(calStartTime + 2000 < millis()) {
                orientation.calibrate(false);
                yawTarget = orientation.getYaw();
                output.standby();
                setState(FLYING);
            }
            break;
        /*#FLYING
         * Hold down and to the left on the throttle stick to disarm
        **/
        case FLYING:
            fly();
            if(timeState<&radio_downLeft>() > 750) {
                output.disable();
                setState(DISARMED);
            }
            break;
    }

}

void fly(){
    const float pitchCmd = ((float)getAPM2Radio(RADIO_PITCH)-90) /-70.0;
    const float rollCmd  = ((float)getAPM2Radio(RADIO_ROLL)-90)  /-70.0;
    const float yawCmd   = ((float)getAPM2Radio(RADIO_YAW)-90)   / 90.0;
    const float throttle = ((float)getAPM2Radio(RADIO_THROTTLE)-25)/130.0;
    const bool  gearCmd  = (getAPM2Radio(4) > 90);

    if(getAPM2Radio(RADIO_THROTTLE) <= CHANNEL_MIN){
        output.standby();
        return;
    } else {
        output.enable();
    }

    if(gearCmd == false){
        //switch to manual mode
        altHold = false;
    } else if (altHold == false && gearCmd == true){
        altitudeSetpoint = altitude.get();
        altPID.train(throttleCurve.get(throttle));
        velocity.set(0);
        altPID.set(0);
        altHold = true;
    }

    if(fabs(yawCmd) > 0.1){
        yawTarget += yawCmd/8;
        yawTarget = truncateRadian(yawTarget);
    }

    float throttleOut;
    if(!altHold){ //manual throttle
        throttleOut = throttleCurve.get(throttle);
    } else { //autonomous throttle





        //adjust setpoint
        float th = (throttle-0.5);
        if(fabs(th) > 0.1) altitudeSetpoint += th/4.0;
        //calculate delta time
        float dt = float(velocity.microsSinceUpdate())/1E6f;
        //calculate vertical acceleration
        Vec3 accl = sensors.getAccl();
        accl.rotateBy(~orientation.getAttitude());
        float va = (accl[2] + 1.0)*GsToFeet;
        //calculate barometer altitude changes
        static float lastAltitude = baro.getAltitude();
        float thisAltitude = baro.getAltitude();
        float deltaAltitude = thisAltitude - lastAltitude;
        lastAltitude = thisAltitude;
        //calculate new velocity
        velocity.set(velocity.get() + va*dt);
        velocity.update(deltaAltitude / dt);
        //calculate output
        float altitudeError  = altitudeSetpoint - altitude.get();
        float targetVelocity = altitude_hold_V * altitudeError;
        throttleOut = altPID.update(velocity.get() - targetVelocity);
        //send debug info
        comms.sendTelem(VOLTAGE+3, altitude.get());
        comms.sendTelem(VOLTAGE+4, velocity.get());
        comms.sendTelem(VOLTAGE+5, altitudeError);
        comms.sendTelem(VOLTAGE+6, targetVelocity);





    }

    comms.sendTelem(VOLTAGE+2  , throttleOut);
    horizon.set(pitchCmd, rollCmd, yawTarget, throttleOut);
}

void sendTelemetry(){
    static uint32_t sendTime = millis();
    if(sendTime < millis()){
        sendTime += 50;

        float voltage  = float(analogRead(67)/1024.l*5.l*10.1f);

        using namespace Protocol;
        comms.sendTelem(LATITUDE   , state);
        comms.sendTelem(LONGITUDE  , ((float)getAPM2Radio(RADIO_THROTTLE)-25)/130.0 );
        comms.sendTelem(HEADING    , toDeg(orientation.getYaw()));
        comms.sendTelem(PITCH      , toDeg(orientation.getPitch()));
        comms.sendTelem(ROLL       , toDeg(orientation.getRoll()));
        comms.sendTelem(GROUNDSPEED, horizon.testPoint[0]);//profile[0]);
        comms.sendTelem(VOLTAGE    , voltage);
        comms.sendTelem(VOLTAGE+1  , altitude.get());

        Serial.println();
        Serial.flush();
    }
}

template<boolfunc Func>
uint32_t timeState() throw() {
    static uint32_t stateEnterTime = 0;
    if(!Func()) {
        //condition fails
        stateEnterTime = 0;
    } else if (stateEnterTime == 0) {
        //time not set
        stateEnterTime = millis();
    } else {
        //time is set and condition still true
        return (millis() - stateEnterTime);
    }
    return 0;
}

bool radio_downRight(){
    bool down  = getAPM2Radio(RADIO_THROTTLE) <= CHANNEL_MIN;
    bool right = getAPM2Radio(RADIO_YAW)      <= CHANNEL_MIN;
    return down && right;
}
bool radio_downLeft(){
    bool down  = getAPM2Radio(RADIO_THROTTLE) <= CHANNEL_MIN;
    bool left  = getAPM2Radio(RADIO_YAW)      >= CHANNEL_MAX;
    return down && left;
}
