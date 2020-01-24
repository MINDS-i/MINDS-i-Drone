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

    /**
     * Startup code for initializing the Multirotor
     * This should be called once
     * Arms the drone
     */
    void beginMultirotor() {
        beginAPM();
        setupSettings();

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
        settings.attach(0, 6666, &changeInterruptPeriod);

        /*AIRSETTING index="1" name="Accel Gain" min="0.0" max="1.0" def="0.003"
         *Factor used during sensor update step. Should be between 0.0 and 1.0
         *A value of 0.0 flies completely based on the best estimate and gyroscope
         *As the value approaches 1.0, the quad increasingly uses the accel measurement
         *to inform pitch/roll.
         */
        settings.attach(1, 0.003f, [](float g){ orientation.setAccelGain(g); });

        /*AIRSETTING index="2" name="Mag Gain" min="0.0" max="1.0" def="0.0015"
         *Factor used during sensor update step. Should be between 0.0 and 1.0
         *The closer to 1 it is, the larger impact the magnetometer has on the aircraft's
         *yaw estimate
         */
        settings.attach(2, 0.0015f, [](float g){ orientation.setMagGain(g); });

        /*AIRSETTING index="3" name="Att P Term" min="0" max="+inf" def="0.2"
         *Attitude Stabilization P term<br>
         *Control proportional to current error.<br>
         *Generally the main driver of PID control.<br>
         *Higher P makes reaction quicker, but increases overshoot and degrades stability.
         */
        settings.attach(3, 0.2f, [](float g){ attPID.setIdealP(g); });

        /*AIRSETTING index="4" name="Att I Term" min="0" max="+inf" def="0.050"
         *Attitude Stabilization I term<br>
         *Used to eliminate steady state error, too much I can increase overshoot and degrade stability.
         */
        settings.attach(4, 0.050f, [](float g){ attPID.setIdealI(g); });

        /*AIRSETTING index="5" name="Att D Term" min="0" max="+inf" def="0.000"
         *Attitude Stabilization D term<br>
         *D Will dampen the output by predicting the future quadcopter position with linear extrapolation.<br>
         *It will decrease overshoot and decrease settling time, but can cause new oscillations if set too high.
         */
        settings.attach(5, 0.000f, [](float g){ attPID.setIdealD(g); });

        /*AIRSETTING index="6" name="Att VP Term" min="0" max="+inf" def="3.00"
         *P term on attitude Velocity control loop <br>
         *Higher values will make stabilization more aggressive.
         */
        settings.attach(6, 3.00f, [](float g){ attVel.setIdealP(g); });

        /*AIRSETTING index="7" name="Att VI Term" min="0" max="+inf" def="0.05"
         *I term on attitude Velocity control loop <br>
         *Higher values increase response to drifting and unevent weight <br>\
         *Too high can cause instability and oscillations
         */
        settings.attach(7, 0.05f, [](float g){ attVel.setIdealI(g); });

        /*AIRSETTING index="8" name="Att VD Term" min="0" max="+inf" def="0.00"
         *D term on attitude Velocity control loop <br>
         *Can be used to dampen oscillations and increase P's ceiling
         */
        settings.attach(8, 0.00f, [](float g){ attVel.setIdealD(g); });

        /*AIRSETTING index="9" name="Yaw P Term" min="-inf" max="+inf" def="1.0"
         *Yaw Stabilization P term<br>
         */
        settings.attach(9, 1.0f, [](float g){ yawPID.setIdealP(g); });

        /*AIRSETTING index="10" name="Yaw I Term" min="0" max="+inf" def="0.0"
         *Yaw Stabilization I term<br>
         */
        settings.attach(10, 0.0f, [](float g){ yawPID.setIdealI(g); });

        /*AIRSETTING index="11" name="Yaw D Term" min="0" max="+inf" def="0.003"
         *Yaw Stabilization D term<br>
         */
        settings.attach(11, 0.003f, [](float g){ yawPID.setIdealD(g); });

        /*AIRSETTING index="12" name="Yaw VP Term" min="0" max="+inf" def="2.00"
         *Yaw stabilization VP term<br>
         */
        settings.attach(12, 2.00f, [](float g){ yawVel.setIdealP(g); });

        /*AIRSETTING index="13" name="Yaw VI Term" min="0" max="+inf" def="8.00"
         *Yaw stabilization VI term<br>
         */
        settings.attach(13, 0.00f, [](float g){ yawVel.setIdealI(g); });

        /*AIRSETTING index="14" name="Yaw VD Term" min="0" max="+inf" def="8.00"
         *Yaw stabilization VD term<br>
         */
        settings.attach(14, 0.00f, [](float g){ yawVel.setIdealD(g); });

        /*AIRSETTING index="15" name="Hover Throttle" min="0" max="1.0" def="0.40"
         *Raw output throttle necessary to hover (used for throttle stick centering)<br>
         *Used in the RC radio throttle stick curve equation. When the throttle stick in at 50%,
         *This is what the quad's final output throttle will be at.
         */
        settings.attach(15, 0.40f, [](float g){ throttleCurve.setHoverPoint(g); });

        /*AIRSETTING index="16" name="Throttle Linearity" min="0" max="1.0" def="0.40"
         *Affects radio throttle curve linearity<br>
         *A Value of 0.5 is as linear as possible around the hover throttle<br>
         *A Value of 0.0 is heavily curved for fine control around the hover point<br>
         *A value of 1.0 is heavily curved for more sensitivity<br>
         *A value slightly under 0.5 tends to work best
         */
        settings.attach(16, 0.40f, [](float g){ throttleCurve.setLinearity(g); });

        /*AIRSETTING index="17" name="Yaw Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the yaw stick can adjust the yaw setpoint
         *in half turns per second
         */
        settings.attach(17, 1.0f, [](float g){ YawTargetSlewRate = g; });

        /*AIRSETTING index="18" name="Altitude Hold Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the throttle stick can adjust the altitude
         *hold setpoint in feet per second
         */
        settings.attach(18, 1.0f, [](float g){ AltitudeTargetSlewRate = g; });

        /*AIRSETTING index="19" name="Barometer Gain" min="0.0" max="1.0" def="0.3"
         * The altitude estimate's sensitivity to changes in barometer readings
         * 1.0 implies the altitude estimate is the exact barometer value
         * 0.0 implies the altitude estimate is not effected by the barometer at all
         */
        settings.attach(19, 0.3f, [](float g){ altitude.setBarometerGain(g); });

        /*AIRSETTING index="20" name="Velocity Gain" min="0.0" max="1.0" def="0.30"
         * The vertical velocity estimate's sensitivity
         * 1.0 implies the vertical velocity estimate updates rapidly
         * 0.0 implies the vertical velocity estimate never changes
         */
        settings.attach(20, 0.3f, [](float g){ altitude.setVelocityGain(g); });

        /*AIRSETTING index="21" name="Altitude Response" min="0.0" max="1.0" def="0.010"
         * How powerful the quadcopter's responses to unwanted changes in altitude are
         */
        settings.attach(21, 0.010f, [](float g){ altitudeHold.setResponseFactor(g); });

        /*AIRSETTING index="22" name="Altitude Velocity Factor" min="0.0" max="1.0" def="0.06"
         * How much the quadcopter's vertical velocity impacts its altitude hold control
         */
        settings.attach(22, 0.060f, [](float g){ altitudeHold.setVelocityFactor(g); });

        /*AIRSETTING index="23" name="Altitude Integral Factor" min="0.0" max="1.0" def="0.004"
         * How much the quadcopter's integrated altitude error contributes to its
         * overall altitude corrections
         */
        settings.attach(23, 0.000f, [](float g){ altitudeHold.setIntegralFactor(g); });

        /*AIRSETTING index="24" name="Position P Term" min="-inf" max="+inf" def="0.0375"
         *Position hold P term<br>
         */
        settings.attach(24, 0.0375f, [](float g){ position.setIdealP(g); });

        /*AIRSETTING index="25" name="Position I Term" min="-inf" max="+inf" def="0.005"
         *Position hold I Term<br>
         */
        settings.attach(25, 0.005f, [](float g){ position.setIdealI(g); });

        /*AIRSETTING index="26" name="Position D Term" min="-inf" max="+inf" def="0.01"
         *Position hold D term
         */
        settings.attach(26, 0.010f, [](float g){ position.setIdealD(g); });

        /*AIRSETTING index="27" name="Maximum Velocity" min="-inf" max="+inf" def="4.0"
         *Maximum over-ground travel velocity<br>
         */
        settings.attach(27, 4.000f, [](float g){ positionHold.setMaximumVelocityTarget(g); });

        /*AIRSETTING index="28" name="MaxV Distance" min="-inf" max="+inf" def="1056"
         *Minimum away from desired position that results in flying towards the
         *target at the maximum allowed velocity
         */
        settings.attach(28, Units::FEET_PER_MILE / 5.0f, [](float g){ positionHold.setVelocityScale(g); });

        /*AIRSETTING index="29" name="Low Battery Warning" min="0.0" max="+inf" def="14.0"
         * At what voltage to consider the quadcopter low on battery
         */
        settings.attach(29, 10.5f, [](float g){ power.setLowVolt(g); });

        /*AIRSETTING index="30" name="Magnetic Declination" min="-180" max="180" def="15.0f"
         * The magnetic declination in degrees of the area the quad will be
         *   flying in
         */
        settings.attach(30, 15.0f, [](float g){ magneticDeclination = toRad(g); });

        /*AIRSETTING index="31" name="GPS assist" min="0" max="1" def="0"
         * Set to 1 to enable gps loitering when flying in assisted mode with
         *   the pitch/roll commands centered.
         * Set to 0 to disable gps loitering; the pilot retains complete
         *   control of pitch and roll when in assisted mode, with the processor
         *   only stabilizing the altitude autonomously
         */
        settings.attach(31, 0.0f, [](float g){ gpsAssist = (g != 0.0f); });

        /*AIRSETTING index="32" name="Auto Descent Rate" min="0" max="+inf" def="1"
         * The desired descent rate in feet per second for the quadcopter to
         *   fall at when auto landing because of a radio signall loss
         */
        settings.attach(32, 1.0f, [](float g){ autolandDescentRate = g; });
    }
}
#endif
