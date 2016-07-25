#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
#include "platforms/Quadcopter.h"
using namespace Platform;

// Radio Channel Mapping
enum RadioChannel{ RADIO_PITCH = 0, RADIO_ROLL = 1, RADIO_THROTTLE = 2,
                   RADIO_YAW   = 3, RADIO_GEAR = 4, RADIO_AUX      = 5 };
const uint8_t CHANNEL_TRIGGER_MIN = 32;
const uint8_t CHANNEL_TRIGGER_MAX = 148;

// Time constants in milliseconds
const uint32_t ARMING_TIME = 2000;
const uint32_t CALIBRATING_TIME = 2000;
const uint32_t DISARMING_TIME = 750;

// State timer used to detect ARMING
StateTimer radioDownRight([](){
    bool down  = APMRadio::get(RADIO_THROTTLE) <= CHANNEL_TRIGGER_MIN;
    bool right = APMRadio::get(RADIO_YAW)      <= CHANNEL_TRIGGER_MIN;
    return down && right;
});

// State timer used to detect DISARMING
StateTimer radioDownLeft([](){
    bool down  = APMRadio::get(RADIO_THROTTLE) <= CHANNEL_TRIGGER_MIN;
    bool left  = APMRadio::get(RADIO_YAW)      >= CHANNEL_TRIGGER_MAX;
    return down && left;
});

// State machine Variables
enum State { DISARMED, CALIBRATE, FLYING } state;
const char* stateString[] = {
    [DISARMED] = "DISARMED", [CALIBRATE] = "CALIBRATE", [FLYING] = "FLYING" };

// Flight State variables
boolean altHoldEnabled = false;
float yawTarget = 0.0;
float altitudeSetpoint;
auto calibrateEndTimer = Interval::elapsed(0);

// Temporary test variables
float outputThrottle;

void setState(State s){
    state = s;
    comms.sendString(stateString[s]);
}

void setup() {
    beginMultirotor();
    setState(DISARMED);
}

void loop() {
    //always running updates
    updateMultirotor();
    sendTelemetry();
    //flight mode state machine
    switch(state){
        /*#DISARMED Hold Down and to the right on
         * the Throttle stick for two seconds to begin the arming process
        **/
        case DISARMED:
            if(radioDownRight.trueFor(ARMING_TIME)) {
                if(safe()){
                    calibrateEndTimer = Interval::elapsed(CALIBRATING_TIME);
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
            if(calibrateEndTimer()) {
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
            if(radioDownLeft.trueFor(DISARMING_TIME)) {
                output.disable();
                setState(DISARMED);
            }
            break;
    }
}

void fly(){
    static auto timer = Interval::every(10);
    if(timer()){
        float pitchCmd = ((float)APMRadio::get(RADIO_PITCH)-90) /-70.0;
        float rollCmd  = ((float)APMRadio::get(RADIO_ROLL)-90)  /-70.0;
        float yawCmd   = ((float)APMRadio::get(RADIO_YAW)-90)   / 90.0;
        float throttle = ((float)APMRadio::get(RADIO_THROTTLE)-25)/130.0;
        bool altSwitch = (APMRadio::get(RADIO_GEAR) > 90);

        // check for low throttle standby mode
        if(APMRadio::get(RADIO_THROTTLE) <= CHANNEL_TRIGGER_MIN){
            output.standby();
            return;
        } else {
            output.enable();
        }

        // switch into and out of altitude hold mode
        if(altSwitch == false){
            //switch to manual mode
            altHoldEnabled = false;
        } else if (altHoldEnabled == false /* implied altSwitch == true*/){
            comms.sendString("Alt Hold");
            altitudeHold.setup(outputThrottle);
            altitudeSetpoint = altitude.getAltitude();
            altHoldEnabled = true;
        }

        // Calculate time delta
        static uint32_t lastRunTime = micros();
        uint32_t now = micros();
        float dt = (now - lastRunTime)/1e6; // in seconds
        lastRunTime = now;

        // update yaw target
        if(fabs(yawCmd) > 0.1){
            yawTarget += yawCmd*dt*M_PI*YawTargetSlewRate;
            yawTarget = truncateRadian(yawTarget);
        }

        // update altitude target
        if(altHoldEnabled){
            float setpointCommand = throttle-0.5;
            if(fabs(setpointCommand) > 0.1){
                altitudeSetpoint += setpointCommand*dt*AltitudeTargetSlewRate;
            }
        }

        // calculate throttle
        float throttleOut = (altHoldEnabled) ?
            altitudeHold.update(altitudeSetpoint, altitude) :
            throttleCurve.get(throttle);

        // set outputs
        outputThrottle = throttleOut;
        horizon.set(pitchCmd, rollCmd, yawTarget, throttleOut);
    }
}

void sendTelemetry(){
    static auto timer = Interval::every(250);
    if(timer()){
        float voltage  = float((analogRead(67)/1024.l)*5.l*10.1f);
        float amperage = float((analogRead(66)/1024.l)*5.l*17.0f);

        using namespace Protocol;
        comms.sendTelem(LATITUDE   , gps.getLatitude());
        comms.sendTelem(LONGITUDE  , gps.getLongitude());
        comms.sendTelem(HEADING    , toDeg(orientation.getYaw()));
        comms.sendTelem(PITCH      , toDeg(orientation.getPitch()));
        comms.sendTelem(ROLL       , toDeg(orientation.getRoll()));
        comms.sendTelem(GROUNDSPEED, outputThrottle);
        comms.sendTelem(VOLTAGE    , voltage);
        comms.sendTelem(AMPERAGE   , amperage);
        comms.sendTelem(ALTITUDE   , altitude.getAltitude());
        comms.sendTelem(ALTITUDE+1 , altitude.getVelocity());
        comms.sendTelem(ALTITUDE+2 , altitudeSetpoint);
        comms.sendTelem(ALTITUDE+3 , baro.getAltitude());
        comms.sendTelem(ALTITUDE+4 , profileTime(0));

        Serial.println();
        Serial.flush();
    }
}
