#include "DroneLibs.h"

const float MINIMUM_INT_PERIOD = 5000;
Settings        settings(eeStorage::getInstance());
HardwareSerial *commSerial  = &Serial;
CommManager     comms(commSerial, eeStorage::getInstance());
DualErrorParams parameters(1,1,1);//defaults stored in settings
DualErrorFilter orientation(parameters);

LEA6H     gps;
MPU6000   mpu;
HMC5883L  cmp;
Sensor* sens[2] = {&mpu, &cmp};
InertialManager sensors(sens, 2);
#define Output_t HK_ESCOutputDevice
//12->7 digital pins = APM outputs 1->4
/*Output_t esc[4] =
    { Output_t(12), Output_t(11)    //North, South
     ,Output_t( 8), Output_t( 7) }; // East,  West*/
Output_t esc[4] =
    { Output_t( 7), Output_t( 8)    //North, South
     ,Output_t(12), Output_t(11) }; // East,  West
OutputDevice* outDev[4] = {&esc[0], &esc[1], &esc[2], &esc[3]};
OutputManager output(outDev);

PIDparameters pitchPID, rollPID;

void isrCallback(){
    sensors.update();
    orientation.update(sensors);
    output.update(orientation);
}
void updatePID(float d){
    using namespace AirSettings;
    PIDparameters newPID = PIDparameters( settings.get(ATT_P_TERM),
                                          settings.get(ATT_I_TERM),
                                          settings.get(ATT_D_TERM) );
    pitchPID = newPID;
    rollPID  = newPID;
}
void changeInterruptPeriod(float newPeriod){
    if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
    startInterrupt(isrCallback, newPeriod);
}
void setupSettings(){
    using namespace AirSettings;
    settings.attach(INT_PERIOD,  6000, &changeInterruptPeriod );
    settings.attach(ACCL_MSE  , 1E2f , callback<DualErrorParams, &parameters, &DualErrorParams::setAcclMSE>);
    settings.attach(ATT_SYSMSE, 1E1f , callback<DualErrorParams, &parameters, &DualErrorParams::setSysMSE> );
    settings.attach(ATT_ERRFAC, 1E10f, callback<DualErrorParams, &parameters, &DualErrorParams::setAcclEF> );
    settings.attach(ATT_P_TERM,  0.0f, &updatePID );
    settings.attach(ATT_I_TERM,  0.0f, &updatePID );
    settings.attach(ATT_D_TERM,  0.0f, &updatePID );
}
void arm(){
    orientation.calibrate(true);
    delay(500);
    output.enable();
    orientation.calibrate(false);
}
void calibrate(){
    orientation.calibrate(true);
    delay(500);
    output.calibrate();
    orientation.calibrate(false);
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

    setupAPM2radio();
    comms.requestResync();
}
void loopQuad() {
    comms.update();
    gps.update();
}
