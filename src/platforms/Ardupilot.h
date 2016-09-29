#ifndef ARDUPILOTPLATFORM_H
#define ARDUPILOTPLATFORM_H

#include "MINDS-i-Drone.h"

#if not defined(__AVR_ATmega2560__)
    #error "Quadcopter platform only supported on arduino Mega"
#endif

/**
 * Platform code for the sensors and hardware capabilites of the
 * Ardupilot mega 2.*
 */
namespace Platform{
    // EEPROM
    Settings settings(eeStorage::getInstance());

    // Comms
    HardwareSerial *commSerial = &Serial;
    CommManager comms(commSerial, eeStorage::getInstance());

    // Sensors
    MPU6000 mpu;
    HMC5883L hmc;
    LEA6H gps;
    MS5611 baro;
    Power power;

    // State variables
    bool errorsDetected = false;

    /**
     * Returns true if none of the hardware has reported an error
     */
    bool safe(){
        return !errorsDetected;
    }

    /**
     * Initialize the Ardupilot hardware
     * Should only be called once
     */
    void beginAPM(){
        // Enable IO
        commSerial->begin(Protocol::BAUD_RATE);
        APMRadio::setup();
        ServoGenerator::begin();

        // Load accelerometer/magnetometer parameters from EEPROM
        if(!settings.foundIMUTune()){
            /*#IMULOAD Couldn't load a valid Accelerometer and
             * Magnetometer tune from EEPROM
            **/
            errorsDetected = true;
            comms.sendString("IMULOAD");
        } else {
            mpu.tuneAccl(settings.getAccelTune());
            hmc.tune(settings.getMagTune());
        }

        // Startup the onboard sensors
        Sensor* sensors[] = {&mpu, &hmc, &baro, &gps};
        for(int i=0; i<4; i++) sensors[i]->begin();
        delay(50);
        for(int i=0; i<4; i++) sensors[i]->calibrate();
        delay(50);
        for(int i=0; i<4; i++) {
            auto status = sensors[i]->status();
            if(!status.good()){
                errorsDetected = true;
                comms.sendString(status.message);
            }
        }
    }

    /**
     * Update the ardupilot's sensors
     * This should be called frequently while the drone is running
     * Note that this does not update the accelerometer, gyroscope, or
     *     magnetometer values; they must be stored in a InertialManager
     *     object
     */
    void updateAPM() {
        comms.update();
        gps.update();
    }
}
#endif
