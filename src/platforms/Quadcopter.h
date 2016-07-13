#include "MINDS-i-Drone.h"

const float MINIMUM_INT_PERIOD = 5000;
Settings        settings(eeStorage::getInstance());
HardwareSerial *commSerial  = &Serial;
CommManager     comms(commSerial, eeStorage::getInstance());
RCFilter        orientation(0.0, 0.0);

MPU6000   mpu;
HMC5883L  cmp;
LEA6H     gps;
MS5611    baro;
InertialVec* sens[2] = {&cmp, &mpu};
Translator   conv[2] = {Translators::APM, Translators::APM};
InertialManager sensors(sens, conv, 2);
#define Output_t AfroESC
Output_t esc[4] = {Output_t(12/*CCW TR APM 1*/), Output_t(11/*CCW BL APM 2*/),
                   Output_t( 8/*CW  TL APM 3*/), Output_t( 7/*CW  BR APM 4*/) };
OutputDevice* outDev[4] = {&esc[0], &esc[1], &esc[2], &esc[3]};
OutputManager output(outDev);

PIDparameters attPID(  -1,  1), yawPID(  -1,  1);
PIDparameters attVel(-100,100), yawVel(-100,100);
PIDparameters altHoldParams(0.0, 1.0);
ThrottleCurve throttleCurve(0.37, 0.4);
Horizon       horizon(&attPID, &attVel,
                      &attPID, &attVel,
                      &yawPID, &yawVel );
bool errorsDetected = false;
void setupSettings();

struct AltitudeHoldParameters{ float C0, C1, K0, K1, K2; } AHP;
float YawTargetSlewRate = 1.0f;

///////////
bool safe();
void arm();
void calibrateESCs();
void setupQuad();
void loopQuad();
///////////

void isrCallback(uint16_t microseconds) {
    float ms = ((float)microseconds)/1000.0;
    tic(0);
    sensors.update();
    orientation.update(sensors, ms);
    output.update(orientation, ms);
    toc(0);
}
void changeInterruptPeriod(float newPeriod){
    if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
    ServoGenerator::setUpdateCallback(isrCallback);
    ServoGenerator::begin(newPeriod);
}
bool safe(){
    return !errorsDetected;
}
void arm(){
    delay(500);
    output.arm();
}
void calibrateESCs(){
    delay(500);
    output.calibrate();
}
void setupQuad() {
    Serial.begin(Protocol::BAUD_RATE);
    comms.requestResync();

    arm();

    if(!settings.foundIMUTune()){
        /*#IMULOAD Couldn't load a valid Accelerometer and
         * Magnetometer tune from EEPROM
        **/
        errorsDetected = true;
        comms.sendString("IMULOAD");
    }
    mpu.tuneAccl(settings.getAccelTune());
    cmp.tune(settings.getMagTune());
    setupSettings();
    APMRadio::setup();

    sensors.start();
    baro.begin();
    gps.begin();
    sensors.calibrate();
    baro.calibrate();
    baro.setTempDutyCycle(2);
    boolean sensorErrors = sensors.errorMessages([](const char * errmsg){
            comms.sendString(errmsg);}
        );
    errorsDetected |= sensorErrors;

    output.setMode(&horizon);
}
void loopQuad() {
    comms.update();
    gps.update();
    baro.update();
}
void setupSettings(){
    if(!settings.foundSettings()){
        /*#DEFAULTS No previously configured flight parameters detected,
         * resetting to defaults
        **/
        comms.sendString("DEFAULTS");
    }

    // these setting messages are specially formatted so a tool in the dashboard
    // can parse them and display the full text. For now, indexes must be unique
    // and both indexes and def (default) values will need to be present in
    // the code and the comment

    /*AIRSETTING index="0" name="Output Period" min="5000" max="10000" def="6666"
     *Period in milliseconds between reading the sensors, calculating orientation,
     *and sending a signal to the ESC's<br>
     *This value should be between 5000 (200Hz) and 10000(100Hz) <br>
     *Higher speeds will decrease the processing time left for other tasks,
     *but could lead to a more stable flight
     */
    settings.attach(0, 6666, &changeInterruptPeriod);

    /*AIRSETTING index="1" name="Accel Gain" min="0.0" max="1.0" def="0.003"
     *Factor used during sensor update step. Should be between 0.0 and 1.0
     *A value of 0.0 flies completely based on the best estimate and gyroscope
     *As the value approaches 1.0, the quad increasingly uses the accel measurement
     *to inform pitch/roll.
     */
    settings.attach(1, 0.003f, [](float g){ orientation.setAccelGain(g); });

    /*AIRSETTING index="2" name="Mag Gain" min="0.0" max="1.0" def="0.00015"
     *Factor used during sensor update step. Should be between 0.0 and 1.0
     *The closer to 1 it is, the larger impact the magnetometer has on the aircraft's
     *yaw estimate
     */
    settings.attach(2, 0.0015f, [](float g){ orientation.setMagGain(g); });

    /*AIRSETTING index="3" name="Tilt Compensation" min="0.0" max="0.0" def="0.0"
     */
    settings.attach(3, 0.00f , [](float g){ horizon.setTiltCompLimit(g); });

    /*AIRSETTING index="4" name="Att P Term" min="0" max="+inf" def="0.375"
     *Attitude Stabilization P term<br>
     *Control proportional to current error.<br>
     *Generally the main driver of PID control.<br>
     *Higher P makes reaction quicker, but increases overshoot and degrades stability.
     */
    settings.attach(4, 0.375f, [](float g){ attPID.setIdealP(g); });

    /*AIRSETTING index="5" name="Att I Term" min="0" max="+inf" def="0.000"
     *Attitude Stabilization I term<br>
     *Used to eliminate steady state error, too much I can increase overshoot and degrade stability.
     */
    settings.attach(5, 0.000f, [](float g){ attPID.setIdealI(g); });

    /*AIRSETTING index="6" name="Att D Term" min="0" max="+inf" def="0.006"
     *Attitude Stabilization D term<br>
     *D Will dampen the output by predicting the future quadcopter position with linear extrapolation.<br>
     *It will decrease overshoot and decrease settling time, but can cause new oscillations if set too high.
     */
    settings.attach(6, 0.006f, [](float g){ attPID.setIdealD(g); });

    /*AIRSETTING index="7" name="Att VP Term" min="0" max="+inf" def="4.50"
     *P term on attitude Velocity control loop <br>
     *Higher values will make stabilization more aggressive.
     */
    settings.attach(7, 4.50f, [](float g){ attVel.setIdealP(g); });

    /*AIRSETTING index="8" name="Att VI Term" min="0" max="+inf" def="2.0"
     *I term on attitude Velocity control loop <br>
     *Higher values increase response to drifting and unevent weight <br>\
     *Too high can cause instability and oscillations
     */
    settings.attach(8, 2.00f, [](float g){ attVel.setIdealI(g); });

    /*AIRSETTING index="9" name="Att VD Term" min="0" max="+inf" def="0.05"
     *D term on attitude Velocity control loop <br>
     *Can be used to dampen oscillations and increase P's ceiling
     */
    settings.attach(9, 0.05f, [](float g){ attVel.setIdealD(g); });

    /*AIRSETTING index="10" name="Yaw P Term" min="-inf" max="+inf" def="1.0"
     *Yaw Stabilization P term<br>
     */
    settings.attach(10, 1.0f, [](float g){ yawPID.setIdealP(g); });

    /*AIRSETTING index="11" name="Yaw I Term" min="0" max="+inf" def="0.0"
     *Yaw Stabilization I term<br>
     */
    settings.attach(11, 0.00f, [](float g){ yawPID.setIdealI(g); });

    /*AIRSETTING index="12" name="Yaw D Term" min="0" max="+inf" def="0.0"
     *Yaw Stabilization D term<br>
     */
    settings.attach(12, 0.00f, [](float g){ yawPID.setIdealD(g); });

    /*AIRSETTING index="13" name="Yaw V Term" min="0" max="+inf" def="8.00"
     *Yaw stabilization V term<br>
     */
    settings.attach(13, 8.00f, [](float g){ yawVel.setIdealP(g); });

    /*AIRSETTING index="14" name="Hover Throttle" min="0" max="1.0" def="0.40"
     *Raw output throttle necessary to hover (used for throttle stick centering)<br>
     *Used in the RC radio throttle stick curve equation. When the throttle stick in at 50%,
     *This is what the quad's final output throttle will be at.
     */
    settings.attach(14, 0.40f, [](float g){ throttleCurve.setHoverPoint(g); });

    /*AIRSETTING index="15" name="Throttle Linearity" min="0" max="1.0" def="0.40"
     *Affects radio throttle curve linearity<br>
     *A Value of 0.5 is as linear as possible around the hover throttle<br>
     *A Value of 0.0 is heavily curved for fine control around the hover point<br>
     *A value of 1.0 is heavily curved for more sensitivity<br>
     *A value slightly under 0.5 tends to work best
     */
    settings.attach(15, 0.40f, [](float g){ throttleCurve.setLinearity(g); });

    /*AIRSETTING index="16" name="C0" min="0.0" max="1.0" def="0.046"
    */
    settings.attach(16, 0.046f, [](float g){ AHP.C0 = g; });
    /*AIRSETTING index="17" name="C1" min="0.0" max="1.0" def="0.020"
    */
    settings.attach(17, 0.020f, [](float g){ AHP.C1 = g; });
    /*AIRSETTING index="18" name="K0" min="0.0" max="1.0" def="0.090"
    */
    settings.attach(18, 0.090f, [](float g){ AHP.K0 = g; });
    /*AIRSETTING index="19" name="K1" min="0.0" max="1.0" def="-2.00"
    */
    settings.attach(19,-2.000f, [](float g){ AHP.K1 = g; });
    /*AIRSETTING index="20" name="K2" min="0.0" max="1.0" def="0.004"
    */
    settings.attach(20, 0.004f, [](float g){ AHP.K2 = g; });

    settings.attach(21, 1.0f, [](float g){ YawTargetSlewRate = g; });
}
