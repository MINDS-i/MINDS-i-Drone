#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
#include "platforms/Quadcopter.h"
using namespace Platform;
const uint8_t MAX_ALLOWED_THROTTLE = 75;

// Radio Channel Mapping
enum RadioChannel{ RADIO_PITCH = 1, RADIO_ROLL = 0, RADIO_THROTTLE = 2,
                   RADIO_YAW   = 3, RADIO_GEAR = 4, RADIO_AUX      = 5 };
const uint8_t CHANNEL_TRIGGER_MIN = 32;
const uint8_t CHANNEL_TRIGGER_MAX = 148;
const float CENTER_RATIO = 0.1f; // 1/2 percentage of stick considered centered

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
enum State { DISARMED, CALIBRATE, FLYING, FAILSAFE } state;
const char* stateString[] = {
    [DISARMED] = "DISARMED", [CALIBRATE] = "CALIBRATE",
    [FLYING] = "FLYING", [FAILSAFE] = "FAILSAFE" };

auto calibrateEndTimer = Interval::elapsed(0);
float yawTarget = 0.0;

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
                if(safe() && !power.isBatteryLow()){
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
                sendHomeLocation();
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
        /*#FAILSAFE
         * Auto-landing; Radio signal loss detected
        **/
        case FAILSAFE:
            land();
            break;
    }
}

boolean assistedMode = false;
boolean gpsStabilization = false;
float altitudeSetpoint;
float outputPitch, outputRoll, outputThrottle;

void fly(){
    static auto timer = Interval::every(10);
    if(!timer()) return;

    uint8_t radioBaseThrottle = APMRadio::get(RADIO_THROTTLE);
    if(radioBaseThrottle>MAX_ALLOWED_THROTTLE) radioBaseThrottle=MAX_ALLOWED_THROTTLE; //limit maximum throttle for drone on gimbal

    if(radioBaseThrottle == 0) {
        // radio disconnect failsafe
        altitudeSetpoint = altitude.getAltitude();
        setState(FAILSAFE);
    } else if(radioBaseThrottle <= CHANNEL_TRIGGER_MIN && !assistedMode){
        // low throttle standby mode
        output.standby();
        return;
    }

    // normal flight mode
    output.enable();

    // read radio controls
    float pitchCmd = ((float)APMRadio::get(RADIO_PITCH)-90) /-70.0;
    float rollCmd  = ((float)APMRadio::get(RADIO_ROLL)-90)  /-70.0;
    float yawCmd   = ((float)APMRadio::get(RADIO_YAW)-90)   /-90.0;
    float throttle = ((float)radioBaseThrottle-25)/130.0;
    bool altSwitch = (APMRadio::get(RADIO_GEAR) > 90);

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

    // switch into and out of altitude hold mode
    if(altSwitch == false){
        assistedMode = false;
        gpsStabilization = false;
    } else if (assistedMode == false /* implied altSwitch == true*/){
        comms.sendString("Alt Hold");
        altitudeHold.setup(outputThrottle);
        altitudeSetpoint = altitude.getAltitude();
        positionHold.setTarget(gps.getLocation());
        assistedMode = true;
    }

    // calculate desired outputs
    if(true){//!assistedMode){  lock quadcopter on gimbal in standard mode never go into assisted(altitude hold)
        outputPitch = pitchCmd;
        outputRoll = rollCmd;
        outputThrottle = throttleCurve.get(throttle);
    } else {
        // update altitude setpoint
        float setpointCommand = throttle-0.5;
        if(fabs(setpointCommand) > 0.1){
            altitudeSetpoint += setpointCommand*dt*AltitudeTargetSlewRate;
        }

        // switch in and out of loiter gps stabilization mode
        bool prStickCentered =
            (pitchCmd <= CENTER_RATIO && pitchCmd >= -CENTER_RATIO) &&
            (rollCmd  <= CENTER_RATIO && rollCmd  >= -CENTER_RATIO);
        if(!gpsAssist || !prStickCentered){
            gpsStabilization = false;
        } else if (gpsStabilization == false ){
            /* implied prStickCentered == true, gpsAssist == true */
            gpsStabilization = true;
            positionHold.setTarget(gps.getLocation());
        }

        // calculate gps stabilization corrections
        auto proutput = positionHold.update(gps, orientation.getYaw()+magneticDeclination);

        // set outputs
        if(gpsStabilization){
            outputPitch = proutput.pitch;
            outputRoll = proutput.roll;
        } else {
            outputPitch = pitchCmd;
            outputRoll = rollCmd;
        }
        outputThrottle = altitudeHold.update(altitudeSetpoint, altitude);
    }

    // limit throttle power when the battery is low
    if(power.isBatteryLow()){
        outputThrottle *= power.suggestedPowerCap();
    }

    horizon.set(outputPitch, outputRoll, yawTarget, outputThrottle);
}

void land(){
    static auto timer = Interval::every(10);
    if(!timer()) return;

    static bool landed = false;

    if(landed){
        output.disable();
    } else {
        altitudeSetpoint -= autolandDescentRate/(1e2/*interval in seconds*/);
        outputThrottle = altitudeHold.update(altitudeSetpoint, altitude);
        horizon.set(0, 0, yawTarget, outputThrottle);
        if(altitudeHold.landingDetected()) landed = true;
    }
}

void sendHomeLocation(){
    comms.sendTelem(Protocol::HOMELATITUDE, gps.getLatitude());
    comms.sendTelem(Protocol::HOMELONGITUDE, gps.getLongitude());
    comms.sendTelem(Protocol::HOMEALTITUDE, altitude.getAltitude());
}

typedef float (*telemLine)(void);
const telemLine telemetryTable[] = {
    [](){ return gps.getLatitude(); },             //LATITUDE
    [](){ return gps.getLongitude(); },            //LONGITUDE
    [](){ return toDeg(orientation.getYaw()); },   //HEADING
    [](){ return toDeg(orientation.getPitch()); }, //PITCH
    [](){ return toDeg(orientation.getRoll()); },  //ROLL
    [](){ return gps.getGroundSpeed(); },          //GROUNDSPEED
    [](){ return power.getVoltage(); },            //VOLTAGE
    [](){ return power.getAmperage(); },           //AMPERAGE
    [](){ return altitude.getAltitude(); },        //ALTITUDE
    [](){ return (float)APMRadio::get(RADIO_THROTTLE); },
    [](){ return (float)APMRadio::get(RADIO_PITCH); },
    [](){ return (float)APMRadio::get(RADIO_ROLL); },
    [](){ return (float)APMRadio::get(RADIO_YAW); },
    [](){ return (float)APMRadio::get(RADIO_GEAR); },
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
