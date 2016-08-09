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

float averageInterval = 0.0;
void loop() {
    toc(1);
    uint32_t loopUpdateTime = profileTime(1);
    averageInterval = 0.01*loopUpdateTime + 0.99*averageInterval;
    tic(1);
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
                positionHold.setTarget(gps.getLocation());
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

PositionHold::Result proutput;
float radioPitch;
float radioRoll;

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


        radioPitch = pitchCmd;
        radioRoll = rollCmd;
        outputThrottle = throttleOut;

        proutput = positionHold.update(gps, orientation.getYaw());

        if(altHoldEnabled){
            horizon.set(proutput.pitch, proutput.roll, yawTarget, throttleOut);
        } else {
            horizon.set(pitchCmd, rollCmd, yawTarget, throttleOut);
        }
        // set outputs

    }
}

//
//make this a class?
float voltage = 0.0;
float amperage = 0.0;
float currentAmperage(){
    return amperage;
}
float currentVoltage(){
    amperage = float((analogRead(66)/1024.l)*5.l*17.0f);
    float rawVolt = float((analogRead(67)/1024.l)*5.l*9.7f);
    float adjVolt = rawVolt + 0.035*amperage;
    voltage = adjVolt * 0.05 + voltage * 0.95;
    return voltage;
}
//
//

typedef float (*telemLine)(void);
const telemLine telemetryTable[] = {
    [](){ return gps.getLatitude(); },             //LATITUDE
    [](){ return gps.getLongitude(); },            //LONGITUDE
    [](){ return toDeg(orientation.getYaw()); },   //HEADING
    [](){ return 1E6f/averageInterval; },
    [](){ return (float)profileTime(2); },
/*
    [](){ return toDeg(orientation.getPitch()); }, //PITCH
    [](){ return toDeg(orientation.getRoll()); },  //ROLL
    [](){ return gps.getGroundSpeed(); },          //GROUNDSPEED
    [](){ return currentVoltage(); },              //VOLTAGE
    [](){ return currentAmperage(); },             //AMPERAGE
    [](){ return altitude.getAltitude(); },        //ALTITUDE

    [](){ return 1E6f/averageInterval; },
    [](){ return gps.getMagVar(); },
    [](){ return gps.getCourse(); },
*/
    [](){ return positionHold.targetSpeed; },
    [](){ return positionHold.distance; },
    [](){ return gps.getCourse(); },
    [](){ return positionHold.speed; },
    [](){ return positionHold.targetNS; },
    [](){ return positionHold.targetEW; },
    [](){ return positionHold.speedNS; },
    [](){ return positionHold.speedEW; },
    [](){ return positionHold.NSoutput; },
    [](){ return positionHold.EWoutput; },

    [](){ return proutput.pitch; },
    [](){ return proutput.roll; },

    [](){ return radioPitch; },
    [](){ return radioRoll; },
};

const uint8_t telemetryTotal =
    sizeof(telemetryTable)/sizeof(telemetryTable[0]);
const uint16_t refreshInterval = 240;
const uint16_t transmitInterval = refreshInterval / telemetryTotal;
void sendTelemetry(){
    static auto timer = Interval::every(transmitInterval);
    static int nextTelemIndex = 0;
    if(timer()){
        comms.sendTelem(nextTelemIndex, telemetryTable[nextTelemIndex]());
        nextTelemIndex = (nextTelemIndex+1) % telemetryTotal;
    }
}
