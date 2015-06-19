#include "DroneLibs.h"

const float MINIMUM_INT_PERIOD = 5000;
Settings        settings(eeStorage::getInstance());
HardwareSerial *commSerial  = &Serial;
CommManager     comms(commSerial, eeStorage::getInstance());
DualErrorFilter orientation(1,1,1);

LEA6H     gps;
MPU6000   mpu;
HMC5883L  cmp;
Sensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
ServoOutput motors[2] = { ServoOutput(A0), ServoOutput(A1) };

PIDparameters rollPID(0,0,0,-1,1);
PIDcontroller PID(&rollPID);

float velFac, rollCommand, out;
float lMotCenter, rMotCenter;//can't use constexpr on arduino
float throttleThrowDiv = 45.0f;
float balanceCenter = 0.0f;

void stabalize(){
    float error = velFac * (rollCommand - orientation.getRoll());
    PID.set(error);
    out = PID.update(orientation.getRollRate()*1024.f) / 2.0f;
    motors[0].set( out+lMotCenter);
    motors[1].set(-out+rMotCenter);
}

void isrCallback(){
    tic(0);
    sensors.update();
    orientation.update(sensors);
    stabalize();
    toc(0);
}
void updatePID(float d){
    using namespace AirSettings;
    PIDparameters newPID = PIDparameters( settings.get(ATT_P_TERM),
                                          settings.get(ATT_I_TERM),
                                          settings.get(ATT_D_TERM),
                                          -1, 1 );
    rollPID  = newPID;
}
void changeInterruptPeriod(float newPeriod){
    if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
    startInterrupt(isrCallback, newPeriod);
}
void setupSettings(){
    using namespace AirSettings;
    settings.attach(INT_PERIOD,  6000, &changeInterruptPeriod );
    settings.attach(ACCL_MSE  , 1E2f , callback<DualErrorFilter, &orientation, &DualErrorFilter::setAcclMSE>);
    settings.attach(ATT_SYSMSE, 1E1f , callback<DualErrorFilter, &orientation, &DualErrorFilter::setSysMSE> );
    settings.attach(ATT_ERRFAC, 1E10f, callback<DualErrorFilter, &orientation, &DualErrorFilter::setAcclEF> );
    settings.attach(ATT_P_TERM,  0.1f, &updatePID );
    settings.attach(ATT_I_TERM,  0.0f, &updatePID );
    settings.attach(ATT_D_TERM,  0.0f, &updatePID );
    settings.attach(VEL_P_TERM, 4.50f, callback<float, &velFac>);

    settings.attach(UNUSED_D  ,  45.0f, callback<float, &throttleThrowDiv>);
    settings.attach(UNUSED_C  ,   0.0f, callback<float, &balanceCenter>);
    settings.attach(UNUSED_B  ,  0.5300f, callback<float, &lMotCenter >);
    settings.attach(UNUSED_A  ,  0.5050f, callback<float, &rMotCenter >);
}
void setupPlatform() {
    Serial.begin(Protocol::BAUD_RATE);

    mpu.tuneAccl(settings.getAccelTune());
    cmp.tune(settings.getMagTune());
    setupSettings();

    sensors.start();
    gps.init();
    sensors.calibrate();

    motors[0].startArming();
    motors[1].startArming();

    orientation.calibrate(true);
    delay(2000);
    orientation.calibrate(false);

    setupAPM2radio();
    comms.requestResync();
}
void loopPlatform() {
    comms.update();
    gps.update();
}
