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
InertialVec* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
#define Output_t HK_ESCOutputDevice
Output_t esc[4] =
    { Output_t(12), Output_t(11)
     ,Output_t( 8), Output_t( 7) };
OutputDevice* outDev[4] = {&esc[0], &esc[1], &esc[2], &esc[3]};
OutputManager output(outDev);

PIDparameters pitchPID(0,0,0,-1,1), rollPID(0,0,0,-1,1);
PIDparameters yawPID(0,0,0,-1,1);
ThrottleCurve throttleCurve(0.33, 0.4);
Horizon       horizon(&pitchPID, &rollPID, &yawPID, &throttleCurve);
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
void updatePID(float d){
    using namespace AirSettings;
    PIDparameters newPID = PIDparameters( settings.get(ATT_P_TERM),
                                          settings.get(ATT_I_TERM),
                                          settings.get(ATT_D_TERM),
                                          -1, 1 );
    pitchPID = newPID;
    rollPID  = newPID;
}
void updateYawPID(float d){
    using namespace AirSettings;
    yawPID = PIDparameters( settings.get(YAW_P_TERM),
                            settings.get(YAW_I_TERM),
                            settings.get(YAW_D_TERM),
                            -1, 1 );
}
void changeInterruptPeriod(float newPeriod){
    if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
    startInterrupt(isrCallback, newPeriod);
}
void setupSettings(){
    /*
     1E8 1E1 1E3
     374 0.001425 2,214
     334633 1.0 0.0212
     */
     /*
     TTC 0.85
     PID: 0.6, 0.06, 0.0225, 6.5
          0.8, 4.00, 0.023 , 6.5
            yaw: -1.0, -0.4, 0.0, 8.0
     */
    using namespace AirSettings;
    settings.attach(INT_PERIOD, 6500  , &changeInterruptPeriod );
    settings.attach(INRT_U_FAC, 0.05f , callback<RCFilter, &orientation, &RCFilter::setwGain>);
    settings.attach(GYRO_CMP_F, 0.99999f, callback<RCFilter, &orientation, &RCFilter::setRateGain>);
    settings.attach(TILT_CMP_L, 1.00f , callback<Horizon, &horizon, &Horizon::setTiltCompLimit>);
    settings.attach(ATT_P_TERM, 0.550f, &updatePID);
    settings.attach(ATT_I_TERM, 0.060f, &updatePID);
    settings.attach(ATT_D_TERM, 0.023f, &updatePID);
    settings.attach(ATT_V_TERM, 6.50f , callback<Horizon, &horizon, &Horizon::setVelFac>);
    settings.attach(YAW_P_TERM, 0.00f , &updateYawPID);
    settings.attach(YAW_I_TERM, 0.00f , &updateYawPID);
    settings.attach(YAW_D_TERM, 0.00f , &updateYawPID);
    settings.attach(YAW_V_TERM, 1.00f , callback<Horizon, &horizon, &Horizon::setYawFac>);
    settings.attach(HOVER_THL , 0.40f , callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setHoverPoint>);
    settings.attach(THL_LINITY, 0.30f , callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setLinearity>);
    settings.attach(BARO_HL   , 1.00f , callback<HLA, &altitude, &HLA::setHalfLife>);
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
