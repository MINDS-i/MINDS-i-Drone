#include "Wire.h"
#include "SPI.h"
#include "MINDS-i-Drone.h"
#include "platforms/Quadcopter.h"

boolean calibrated = false;
boolean altHold = false;
float outputThrottle;
uint32_t calStartTime;
const uint8_t CHANNEL_MIN = 32;
const uint8_t CHANNEL_MAX = 148;
float yawTarget = 0.0;

enum State { DISARMED, CALIBRATE, FLYING } state;
const char* stateString[] = {
    [DISARMED] = "DISARMED", [CALIBRATE] = "CALIBRATE", [FLYING] = "FLYING" };
enum RadioChannel{ RADIO_PITCH = 0, RADIO_ROLL = 1, RADIO_THROTTLE = 2,
                   RADIO_YAW   = 3, RADIO_GEAR = 4, RADIO_AUX      = 5 };

StateTimer radioDownRight([](){
    bool down  = APMRadio::get(RADIO_THROTTLE) <= CHANNEL_MIN;
    bool right = APMRadio::get(RADIO_YAW)      <= CHANNEL_MIN;
    return down && right;
});

StateTimer radioDownLeft([](){
    bool down  = APMRadio::get(RADIO_THROTTLE) <= CHANNEL_MIN;
    bool left  = APMRadio::get(RADIO_YAW)      >= CHANNEL_MAX;
    return down && left;
});

//
uint32_t loopstart;
uint32_t loopcount;
//

void setup() {
    setupQuad();
    setState(DISARMED);
}

void setState(State s){
    state = s;
    comms.sendString(stateString[s]);
    loopstart = micros();
    loopcount = 0;
}

void loop() {
    loopcount++;
    //always running updates
    loopQuad();
    sendTelemetry();

    //flight mode state machine
    switch(state){
        /*#DISARMED Hold Down and to the right on
         * the Throttle stick for two seconds to begin the arming process
        **/
        case DISARMED:
            if(radioDownRight.trueFor(2000)) {
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
                altitudeCalcInit();
                output.standby();
                setState(FLYING);
            }
            break;
        /*#FLYING
         * Hold down and to the left on the throttle stick to disarm
        **/
        case FLYING:
            fly();
            if(radioDownLeft.trueFor(750)) {
                output.disable();
                setState(DISARMED);
            }
            break;
    }
}

void fly(){
    const float pitchCmd = ((float)APMRadio::get(RADIO_PITCH)-90) /-70.0;
    const float rollCmd  = ((float)APMRadio::get(RADIO_ROLL)-90)  /-70.0;
    const float yawCmd   = ((float)APMRadio::get(RADIO_YAW)-90)   / 90.0;
    const float throttle = ((float)APMRadio::get(RADIO_THROTTLE)-25)/130.0;
    const bool  gearCmd  = (APMRadio::get(RADIO_GEAR) > 90);
    altitudeCalculation();

    // check for low throttle standby mode
    if(APMRadio::get(RADIO_THROTTLE) <= CHANNEL_MIN){
        output.standby();
        return;
    } else {
        output.enable();
    }

    // switch into and out of altitude hold mode
    if(gearCmd == false){
        //switch to manual mode
        altHold = false;
    } else if (altHold == false && gearCmd == true){
        comms.sendString("Alt Hold");
        altHoldInit(throttleCurve.get(throttle));
        altHold = true;
    }

    // update yaw target
    if(fabs(yawCmd) > 0.1){
        static uint32_t integrationTimer = micros();
        float dt = (micros() - integrationTimer)/1e6;
        yawTarget += yawCmd*dt;
        yawTarget = truncateRadian(yawTarget);
    }

    // calculate target throttle
    float throttleOut = (altHold)? altHoldUpdate(throttle) :
                                   throttleCurve.get(throttle);

    // set output targets
    outputThrottle = throttleOut;
    horizon.set(pitchCmd, rollCmd, yawTarget, throttleOut);
}

// Variables used in altitude hold
float altitudeEst;
float velocityEst;
float oldIntegral;
float hover;
float throttleOutput;
float altitudeSetpoint;
//

void altHoldInit(float curThrottle){
    altitudeSetpoint = altitudeEst;
    hover = curThrottle;
    oldIntegral = 0.0;
    velocityEst = 0.0;
    throttleOutput = 0.0;
}

void altitudeCalcInit(){
    altitudeEst = baro.getAltitude();
}

void altitudeCalculation(){
    static auto timer = Interval::every(10);
    if(timer()){
        //float dt = (micros() - nextUpdate + UPDATE_MICROS) / 1e6;
        const float dt = 0.01;

        float C0 = AHP.C0;
        float C1 = AHP.C1;

        float barometer = baro.getAltitude();
        float altitude = barometer*C0 + altitudeEst*(1.0-C0);
        float newvelocity = (altitude-altitudeEst) / dt;
        float velocity = newvelocity*C1 + velocityEst*(1.0-C1);

        altitudeEst = altitude;
        velocityEst = velocity;
    }
}

float altHoldUpdate(float throttleCMD){
    static auto timer = Interval::every(10);
    if(timer()){
        //float dt = (micros() - nextUpdate + UPDATE_MICROS) / 1e6;
        const float dt = 0.01;

        float K0 = AHP.K0;
        float K1 = AHP.K1;
        float K2 = AHP.K2;

        //adjust setpoint
        float th = (throttleCMD-0.5);
        if(fabs(th) > 0.1) altitudeSetpoint += th/4.0;

        float error = altitudeSetpoint-altitudeEst;
        float newIntegral = oldIntegral + error * dt;
        throttleOutput = (K0/10.0)*(error + K1*velocityEst + K2*newIntegral) + hover;

        oldIntegral = newIntegral;
    }

    return throttleOutput;
}

void sendTelemetry(){
    static auto timer = Interval::every(250);
    if(timer()){
        float voltage  = float((analogRead(67)/1024.l)*5.l*10.1f);
        float amperage = float((analogRead(66)/1024.l)*5.l*17.0f);

        using namespace Protocol;
        comms.sendTelem(LATITUDE   , (micros()-loopstart)/loopcount);
        comms.sendTelem(LONGITUDE  , altHold );
        comms.sendTelem(HEADING    , toDeg(orientation.getYaw()));
        comms.sendTelem(PITCH      , toDeg(orientation.getPitch()));
        comms.sendTelem(ROLL       , toDeg(orientation.getRoll()));
        comms.sendTelem(GROUNDSPEED, outputThrottle);
        comms.sendTelem(VOLTAGE    , voltage);
        comms.sendTelem(AMPERAGE   , amperage);
        comms.sendTelem(ALTITUDE, altitudeEst);
        comms.sendTelem(ALTITUDE+1, velocityEst);
        comms.sendTelem(ALTITUDE+2, oldIntegral);
        comms.sendTelem(ALTITUDE+3, altitudeSetpoint);

        Serial.println();
        Serial.flush();
    }
}
