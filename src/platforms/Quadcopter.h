#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "MINDS-i-Drone.h"
#include "platforms/Ardupilot.h"

namespace Platform {
    // Output devices
    #ifndef Output_t
    #define Output_t AfroESC
    #endif
    Output_t esc[4] =
        { Output_t(12/*CCW TR APM 1*/), Output_t(11/*CCW BL APM 2*/),
          Output_t( 8/*CW  TL APM 3*/), Output_t( 7/*CW  BR APM 4*/) };
    OutputDevice* outDev[4] = {&esc[0], &esc[1], &esc[2], &esc[3]};
    OutputManager output(outDev);

    // Inertial sensors and their frame translators
    InertialVec* sens[2] = {&hmc, &mpu};
    Translator   conv[2] = {Translators::APM, Translators::APM};
    InertialManager imu(sens, conv, 2);

    // Horizon flight controller and attitude PID parameters
    PIDparameters attPID(  -1,  1), yawPID(  -1,  1);
    PIDparameters attVel(-100,100), yawVel(-100,100);
    Horizon horizon(&attPID, &attVel,
                    &attPID, &attVel,
                    &yawPID, &yawVel );

    // Altitude hold controller; paramaters configured by settings
    AltitudeHold altitudeHold;

    PIDparameters position(-0.25, 0.25);
    PositionHold positionHold(&position);

    // Input parameter variables; default values rewritten by settings
    ThrottleCurve throttleCurve(0.0, 0.0);
    float YawTargetSlewRate;
    float AltitudeTargetSlewRate;
    float magneticDeclination;
    float autolandDescentRate;
    bool gpsAssist;

    // Minimum time between orientation and output updates in milliseconds
    const float MINIMUM_INT_PERIOD = 5000;

    // Quadcopter state trackers; defaults rewritten by settings
    Altitude altitude;
    RCFilter orientation(0.0,0.0);

    /**
     * Arm all the motors.
     * Safe to call a second time.
     * Blocking call.
     */
    void arm(){
        delay(500);
        output.arm();
    }

    /**
     * Send the calibration sequence to the motors.
     * Usually, this leaves them armed, and running when armed will cause
     * dangerous behavior.
     * Blocking call.
     */
    void calibrateESCs(){
        delay(500);
        output.calibrate();
    }

    /**
     * Update function called in interrupts to read essential sensors,
     * calculate correction torques, and apply new motor outputs
     *
     * It also includes the barometer update, because, while not flight critical
     *   it is best to keep all SPI bus reads in the same "thread" and
     *   consistently timed. Reading the barometer takes ~.1 ms
     */
    void isrCallback(uint16_t microseconds) {
        tic(0);
        float ms = ((float)microseconds)/1000.0;
        imu.update();
        orientation.update(imu, ms);
        output.update(orientation, ms);
        baro.update();
        toc(0);
    }

    /**
     * Update the frequency that the servo signal generator refreshes and that
     * `isrCallback` is run.
     */
    void changeInterruptPeriod(float newPeriod){
        if(newPeriod < MINIMUM_INT_PERIOD) newPeriod = MINIMUM_INT_PERIOD;
        ServoGenerator::setUpdateCallback(isrCallback);
        ServoGenerator::begin(newPeriod);
    }

    void setupSettings();

    Storage<float> *storage	= eeStorage::getInstance();

    const float settingsData[][3] PROGMEM = { 
        {5000,10000,6666},            //Output Period
        {0,1,0.003},                  //Accel Gain
        {0,1,0.0015},                 //Mag Gain
        {0,INFINITY,0.2},             //Attitude P    
        {0,INFINITY,0.05},            //Attitude I
        {0,INFINITY,0.0},             //Attitude D
        {0,INFINITY,3.0},             //Attitude VP
        {0,INFINITY,0.05},            //Attitude VI
        {0,INFINITY,0.0},             //Attitude VD
        {-INFINITY,INFINITY,1.0},     //Yaw P
        {0,INFINITY,0.0},             //Yaw I
        {0,INFINITY,0.003},           //Yaw D
        {0,INFINITY,2.0},             //Yaw VP
        {0,INFINITY,8.0},             //Yaw VI
        {0,INFINITY,8.0},             //Yaw VD
        {0,1.0,0.4},                  //Hover Throttle
        {0,1.0,0.4},                  //Throttle Linearity
        {0,10.0,1.0},                 //Yaw Slew Rate
        {0,10.0,1.0},                 //Altitude Hold Slew Rate
        {0,1.0,0.3},                  //Barometer Gain
        {0,1.0,0.3},                  //Velocity Gain
        {0,1.0,0.01},                 //Altitude Response
        {0,1.0,0.06},                 //Altitude Velocity Factor
        {0,1.0,0.004},                //Altitude Integral Factor
        {-INFINITY,INFINITY,0.0375},  //Position P
        {-INFINITY,INFINITY,0.005},   //Position I
        {-INFINITY,INFINITY,0.01},    //Position D
        {-INFINITY,INFINITY,4.0},     //Maximum Velocity
        {-INFINITY,INFINITY,1056.0},  //MaxV Distance
        {0,INFINITY,10.5},            //Low Battery Warning
        {-180,180,15.0},              //Magnetic Declination
        {0,1,0},                      //GPS assist
        {0,INFINITY,1.0},             //Auto Descent Rate
    };

    void setDefaultSettings()
    {
        for (byte i=0;i<sizeof(settingsData)/sizeof(settingsData[0]);i++)
        {
            // Set the default value for the setting (default value is the third column)
            storage->updateRecord(i,pgm_read_float_near(&settingsData[i][2]));
        }
    }

    /**
     * Startup code for initializing the Multirotor
     * This should be called once
     * Arms the drone
     */
    void beginMultirotor() {
        beginAPM();
        setupSettings();
        comms.setSettingsResetCallback(setDefaultSettings);

        // Settings should be set before `arm`, because serial signal fresh
        // period is a setting
        arm();

        // Estimate of startup altitude
        // The barometer readings aren't available yet
        // The altitude filter will rapidly converge on the correct altitude
        altitude.setup(1000.0);

        comms.requestResync();
        output.setMode(&horizon);
    }

    /**
     * Update function for the Multirotor
     * This should be called frequently while the drone is running
     * It does not contain flight critical updates
     */
    void updateMultirotor() {
        updateAPM();
        altitude.update(baro.getAltitude());
        power.checkCapacity(comms);
    }

    /**
     * Initialize the various air settings, either loading their stored values,
     * or loading the defaults if this is the first run since the last updload
     * It will write to every controlled variable in either case.
     */
    void setupSettings(){
        if(!settings.foundSettings()){
            /*#DEFAULTS No previously configured flight parameters detected,
             * resetting to defaults
            **/
            comms.sendString("DEFAULTS");
        }

        uint8_t index;

        // these setting messages are specially formatted so a tool in the dashboard
        // can parse them and display the full text. For now, indexes must be unique
        // and both indexes and def (default) values will need to be present in
        // the code and the comment

        /*AIRSETTING index="0" name="Output Period" min="5000" max="10000" def="6666"
         *Period in milliseconds between reading the imu, calculating orientation,
         *and sending a signal to the ESC's<br>
         *This value should be between 5000 (200Hz) and 10000(100Hz) <br>
         *Higher speeds will decrease the processing time left for other tasks,
         *but could lead to a more stable flight
         */
        index = 0;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            &changeInterruptPeriod
        );

        /*AIRSETTING index="1" name="Accel Gain" min="0.0" max="1.0" def="0.003"
         *Factor used during sensor update step. Should be between 0.0 and 1.0
         *A value of 0.0 flies completely based on the best estimate and gyroscope
         *As the value approaches 1.0, the quad increasingly uses the accel measurement
         *to inform pitch/roll.
         */
        index = 1;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ orientation.setAccelGain(g); }
        );

        /*AIRSETTING index="2" name="Mag Gain" min="0.0" max="1.0" def="0.0015"
         *Factor used during sensor update step. Should be between 0.0 and 1.0
         *The closer to 1 it is, the larger impact the magnetometer has on the aircraft's
         *yaw estimate
         */
        index = 2;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ orientation.setMagGain(g); }
        );

        /*AIRSETTING index="3" name="Att P Term" min="0" max="+inf" def="0.2"
         *Attitude Stabilization P term<br>
         *Control proportional to current error.<br>
         *Generally the main driver of PID control.<br>
         *Higher P makes reaction quicker, but increases overshoot and degrades stability.
         */
        index = 3;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attPID.setIdealP(g); }
        );

        /*AIRSETTING index="4" name="Att I Term" min="0" max="+inf" def="0.050"
         *Attitude Stabilization I term<br>
         *Used to eliminate steady state error, too much I can increase overshoot and degrade stability.
         */
        index = 4;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attPID.setIdealI(g); }
        );

        /*AIRSETTING index="5" name="Att D Term" min="0" max="+inf" def="0.000"
         *Attitude Stabilization D term<br>
         *D Will dampen the output by predicting the future quadcopter position with linear extrapolation.<br>
         *It will decrease overshoot and decrease settling time, but can cause new oscillations if set too high.
         */
        index = 5;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attPID.setIdealD(g); }
        );

        /*AIRSETTING index="6" name="Att VP Term" min="0" max="+inf" def="3.00"
         *P term on attitude Velocity control loop <br>
         *Higher values will make stabilization more aggressive.
         */
        index = 6;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attVel.setIdealP(g); }
        );

        /*AIRSETTING index="7" name="Att VI Term" min="0" max="+inf" def="0.05"
         *I term on attitude Velocity control loop <br>
         *Higher values increase response to drifting and unevent weight <br>\
         *Too high can cause instability and oscillations
         */
        index = 7;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attVel.setIdealI(g); }
        );

        /*AIRSETTING index="8" name="Att VD Term" min="0" max="+inf" def="0.00"
         *D term on attitude Velocity control loop <br>
         *Can be used to dampen oscillations and increase P's ceiling
         */
        index = 8;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ attVel.setIdealD(g); }
        );

        /*AIRSETTING index="9" name="Yaw P Term" min="-inf" max="+inf" def="1.0"
         *Yaw Stabilization P term<br>
         */
        index = 9;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawPID.setIdealP(g); }
        );

        /*AIRSETTING index="10" name="Yaw I Term" min="0" max="+inf" def="0.0"
         *Yaw Stabilization I term<br>
         */
        index = 10;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawPID.setIdealI(g); }
        );

        /*AIRSETTING index="11" name="Yaw D Term" min="0" max="+inf" def="0.003"
         *Yaw Stabilization D term<br>
         */
        index = 11;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawPID.setIdealD(g); }
        );

        /*AIRSETTING index="12" name="Yaw VP Term" min="0" max="+inf" def="2.00"
         *Yaw stabilization VP term<br>
         */
        index = 12;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawVel.setIdealP(g); }
        );

        /*AIRSETTING index="13" name="Yaw VI Term" min="0" max="+inf" def="8.00"
         *Yaw stabilization VI term<br>
         */
        index = 13;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawVel.setIdealI(g); }
        );

        /*AIRSETTING index="14" name="Yaw VD Term" min="0" max="+inf" def="8.00"
         *Yaw stabilization VD term<br>
         */
        index = 14;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ yawVel.setIdealD(g); }
        );

        /*AIRSETTING index="15" name="Hover Throttle" min="0" max="1.0" def="0.40"
         *Raw output throttle necessary to hover (used for throttle stick centering)<br>
         *Used in the RC radio throttle stick curve equation. When the throttle stick in at 50%,
         *This is what the quad's final output throttle will be at.
         */
        index = 15;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ throttleCurve.setHoverPoint(g); }
        );

        /*AIRSETTING index="16" name="Throttle Linearity" min="0" max="1.0" def="0.40"
         *Affects radio throttle curve linearity<br>
         *A Value of 0.5 is as linear as possible around the hover throttle<br>
         *A Value of 0.0 is heavily curved for fine control around the hover point<br>
         *A value of 1.0 is heavily curved for more sensitivity<br>
         *A value slightly under 0.5 tends to work best
         */
        index = 16;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ throttleCurve.setLinearity(g); }
        );

        /*AIRSETTING index="17" name="Yaw Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the yaw stick can adjust the yaw setpoint
         *in half turns per second
         */
        index = 17;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ YawTargetSlewRate = g; }
        );

        /*AIRSETTING index="18" name="Altitude Hold Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the throttle stick can adjust the altitude
         *hold setpoint in feet per second
         */
        index = 18;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ AltitudeTargetSlewRate = g; }
        );

        /*AIRSETTING index="19" name="Barometer Gain" min="0.0" max="1.0" def="0.3"
         * The altitude estimate's sensitivity to changes in barometer readings
         * 1.0 implies the altitude estimate is the exact barometer value
         * 0.0 implies the altitude estimate is not effected by the barometer at all
         */
        index = 19;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ altitude.setBarometerGain(g); }
        );

        /*AIRSETTING index="20" name="Velocity Gain" min="0.0" max="1.0" def="0.30"
         * The vertical velocity estimate's sensitivity
         * 1.0 implies the vertical velocity estimate updates rapidly
         * 0.0 implies the vertical velocity estimate never changes
         */
        index = 20;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ altitude.setVelocityGain(g); }
        );

        /*AIRSETTING index="21" name="Altitude Response" min="0.0" max="1.0" def="0.010"
         * How powerful the quadcopter's responses to unwanted changes in altitude are
         */
        index = 21;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ altitudeHold.setResponseFactor(g); }
        );

        /*AIRSETTING index="22" name="Altitude Velocity Factor" min="0.0" max="1.0" def="0.06"
         * How much the quadcopter's vertical velocity impacts its altitude hold control
         */
        index = 22;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ altitudeHold.setVelocityFactor(g); }
        );

        /*AIRSETTING index="23" name="Altitude Integral Factor" min="0.0" max="1.0" def="0.004"
         * How much the quadcopter's integrated altitude error contributes to its
         * overall altitude corrections
         */
        index = 23;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ altitudeHold.setIntegralFactor(g); }
        );

        /*AIRSETTING index="24" name="Position P Term" min="-inf" max="+inf" def="0.0375"
         *Position hold P term<br>
         */
        index = 24;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ position.setIdealP(g); }
        );

        /*AIRSETTING index="25" name="Position I Term" min="-inf" max="+inf" def="0.005"
         *Position hold I Term<br>
         */
        index = 25;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ position.setIdealI(g); }
        );

        /*AIRSETTING index="26" name="Position D Term" min="-inf" max="+inf" def="0.01"
         *Position hold D term
         */
        index = 26;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ position.setIdealD(g); }
        );

        /*AIRSETTING index="27" name="Maximum Velocity" min="-inf" max="+inf" def="4.0"
         *Maximum over-ground travel velocity<br>
         */
        index = 27;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ positionHold.setMaximumVelocityTarget(g); }
        );

        /*AIRSETTING index="28" name="MaxV Distance" min="-inf" max="+inf" def="1056"
         *Minimum away from desired position that results in flying towards the
         *target at the maximum allowed velocity
         */
        index = 28;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ positionHold.setVelocityScale(g); }
        );

        /*AIRSETTING index="29" name="Low Battery Warning" min="0.0" max="+inf" def="10.5"
         * At what voltage to consider the quadcopter low on battery
         */
        index = 29;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ power.setLowVolt(g); }
        );

        /*AIRSETTING index="30" name="Magnetic Declination" min="-180" max="180" def="15.0f"
         * The magnetic declination in degrees of the area the quad will be
         *   flying in
         */
        index = 30;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ magneticDeclination = toRad(g); }
        );

        /*AIRSETTING index="31" name="GPS assist" min="0" max="1" def="0"
         * Set to 1 to enable gps loitering when flying in assisted mode with
         *   the pitch/roll commands centered.
         * Set to 0 to disable gps loitering; the pilot retains complete
         *   control of pitch and roll when in assisted mode, with the processor
         *   only stabilizing the altitude autonomously
         */
        index = 31;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ gpsAssist = (g != 0.0f); }
        );

        /*AIRSETTING index="32" name="Auto Descent Rate" min="0" max="+inf" def="1"
         * The desired descent rate in feet per second for the quadcopter to
         *   fall at when auto landing because of a radio signall loss
         */
        index = 32;
        settings.attach(
            index,
            pgm_read_float_near(&settingsData[index][0]),
            pgm_read_float_near(&settingsData[index][1]),
            pgm_read_float_near(&settingsData[index][2]),
            [](float g){ autolandDescentRate = g; }
        );
    }
}
#endif