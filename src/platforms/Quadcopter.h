#include "DroneLibs.h"

const float MINIMUM_INT_PERIOD = 5000;
Settings        settings(eeStorage::getInstance());
HardwareSerial *commSerial  = &Serial;
CommManager     comms(commSerial, eeStorage::getInstance());
RCFilter        orientation(0.05, 0.0);

MPU6000   mpu;
HMC5883L  cmp;
LEA6H     gps;
MS5611    baro;
InertialVec* sens[2] = {&cmp, &mpu};
Translator   conv[2] = {Translators::APM, Translators::APM};
InertialManager sensors(sens, conv, 2);

#define Output_t HK_ESCOutputDevice
Output_t esc[4] =
    { Output_t(12), Output_t(11)
     ,Output_t( 8), Output_t( 7) };
OutputDevice* outDev[4] = {&esc[0], &esc[1], &esc[2], &esc[3]};
OutputManager output(outDev);

PIDparameters attPID(0,0,0,-100,100), yawPID(0,0,0,-1,1);
PIDparameters attVel(0,0,0,-100,100), yawVel(0,0,0,-1,1);
PIDparameters altHoldParams(0,0,0, 0.0, 1.0);
ThrottleCurve throttleCurve(0.33, 0.4);
Horizon       horizon(&attPID, &attVel,
                      &attPID, &attVel,
                      &yawPID, &yawVel );
HLA           altitude(1, 0);

///////////
void arm();
void calibrateESCs();
void setupQuad();
void loopQuad();
///////////

void isrCallback() {
    tic(0);
    sensors.update();
    orientation.update(sensors);
    output.update(orientation);
    toc(0);
}
void changeInterruptPeriod(float newPeriod){
    if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
    startInterrupt(isrCallback, newPeriod);
}
void setupSettings(){
     /*
     TTC 0.85
     PID: 0.6, 0.06 0.0225; 6.5 0.0 0.0
          0.8, 4.00 0.023 ; 6.5 0.0 0.0
          0.2  0.0  0.005;  8.0 0.5 0.5
     */
    using namespace AirSettings;
    settings.attach(INT_PERIOD, 6500  , &changeInterruptPeriod );
    settings.attach(INRT_U_FAC, 0.0038f, callback<RCFilter, &orientation, &RCFilter::setwGain>);
    settings.attach(GYRO_CMP_F, 0.99999f, callback<RCFilter, &orientation, &RCFilter::setRateGain>);
    settings.attach(TILT_CMP_L, 1.00f , callback<Horizon, &horizon, &Horizon::setTiltCompLimit>);
    settings.attach(ATT_P_TERM, 0.250f, callback<PIDparameters, &attPID, &PIDparameters::setIdealP>);
    settings.attach(ATT_I_TERM, 0.000f, callback<PIDparameters, &attPID, &PIDparameters::setIdealI>);
    settings.attach(ATT_D_TERM, 0.003f, callback<PIDparameters, &attPID, &PIDparameters::setIdealD>);
    settings.attach(ATT_VP_TERM,5.00f , callback<PIDparameters, &attVel, &PIDparameters::setIdealP>);
    settings.attach(ATT_VI_TERM,0.80f , callback<PIDparameters, &attVel, &PIDparameters::setIdealI>);
    settings.attach(ATT_VD_TERM,0.17f , callback<PIDparameters, &attVel, &PIDparameters::setIdealD>);
    settings.attach(YAW_P_TERM, -1.0f , callback<PIDparameters, &yawPID, &PIDparameters::setIdealP>);
    settings.attach(YAW_I_TERM, 0.00f , callback<PIDparameters, &yawPID, &PIDparameters::setIdealI>);
    settings.attach(YAW_D_TERM, 0.00f , callback<PIDparameters, &yawPID, &PIDparameters::setIdealD>);
    settings.attach(YAW_V_TERM, 8.00f , callback<PIDparameters, &yawVel, &PIDparameters::setIdealP>);
    settings.attach(HOVER_THL , 0.40f , callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setHoverPoint>);
    settings.attach(THL_LINITY, 0.37f , callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setLinearity>);
    settings.attach(BARO_HL   , 500.f , callback<HLA, &altitude, &HLA::setHalfLife>);
    // alt hold pid
    settings.attach(BARO_P, 0.f , callback<PIDparameters, &altHoldParams, &PIDparameters::setIdealP>);
    settings.attach(BARO_I, 0.f , callback<PIDparameters, &altHoldParams, &PIDparameters::setIdealI>);
    settings.attach(BARO_D, 0.f , callback<PIDparameters, &altHoldParams, &PIDparameters::setIdealD>);

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

    mpu.tuneAccl(settings.getAccelTune());
    cmp.tune(settings.getMagTune());
    setupSettings();

    sensors.start();
    baro.begin();
    gps.begin();
    sensors.calibrate();
    baro.calibrate();

    arm();
    output.setMode(&horizon);

    setupAPM2radio();
    comms.requestResync();
}
void loopQuad() {
    comms.update();
    gps.update();
    baro.update();
    altitude.update(baro.getAltitude());
}
