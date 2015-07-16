#include "DroneLibs.h"

typedef SQEFilter Filter_t;
const float MINIMUM_INT_PERIOD = 5000;
Settings        settings(eeStorage::getInstance());
HardwareSerial *commSerial  = &Serial;
CommManager     comms(commSerial, eeStorage::getInstance());
Filter_t        orientation(1,1,1);

LEA6H     gps;
MPU6000   mpu;
HMC5883L  cmp;
Sensor* sens[2] = {&mpu, &cmp};
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
Horizon horizon(&pitchPID, &rollPID, &yawPID, &throttleCurve);

///////////
void arm();
void calibrateESCs();
void setupQuad();
void loopQuad();
///////////

float pidScore = 100000000;
float rollCommand, pitchCommand;
void measurePID(){
    static float lastError[2];
    lastError[1] = lastError[0];
    lastError[0] = (rollCommand - orientation.getRoll())+
                   (pitchCommand- orientation.getPitch());

    const float e1 = lastError[0];
    const float e2 = lastError[1];
    const float de = e2 - e1;

    pidScore  = pidScore * 0.99;
    pidScore += e1*e1 + e1*de + de*de/3.0f;
    if(isnan(pidScore)) pidScore = 100000000;
}

void isrCallback(){
    tic(0);
    sensors.update();
    orientation.update(sensors);
    output.update(orientation);
    toc(0);
    measurePID();
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
    using namespace AirSettings;
    settings.attach(INT_PERIOD, 6500   , &changeInterruptPeriod );
    settings.attach(ACCL_MSE  , 500.0f , callback<Filter_t, &orientation, &Filter_t::setAcclMSE>);
    settings.attach(ATT_SYSMSE, 0.0015f, callback<Filter_t, &orientation, &Filter_t::setSysMSE> );
    settings.attach(ATT_ERRFAC, 10.0f  , callback<Filter_t, &orientation, &Filter_t::setAcclEF> );
    settings.attach(ATT_P_TERM, 0.300f , &updatePID );
    settings.attach(ATT_I_TERM, 0.050f , &updatePID );
    settings.attach(ATT_D_TERM, 0.020f , &updatePID );
    settings.attach(VEL_P_TERM, 4.50f, callback<Horizon, &horizon, &Horizon::setVelFac>);
    settings.attach(YAW_P_TERM, 3.00f, &updateYawPID);
    settings.attach(YAW_I_TERM, 1.00f, &updateYawPID);
    settings.attach(YAW_D_TERM, 0.00f, &updateYawPID);
    settings.attach(HOVER_THL , 0.40f, callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setHoverPoint>);
    settings.attach(THL_LINITY, 0.30f, callback<ThrottleCurve, &throttleCurve, &ThrottleCurve::setLinearity>);
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
    gps.init();
    sensors.calibrate();

    arm();
    output.setMode(&horizon);

    setupAPM2radio();
    comms.requestResync();
}
void loopQuad() {
    comms.update();
    gps.update();
}
