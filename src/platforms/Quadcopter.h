#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "MINDS-i-Drone.h"
#include "platforms/Ardupilot.h"

namespace Platform {
    // Output devices
    #define Output_t AfroESC
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

    // Input parameter variables; default values rewritten by settings
    ThrottleCurve throttleCurve(0.0, 0.0);
    float YawTargetSlewRate;
    float AltitudeTargetSlewRate;

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
     */
    void isrCallback(uint16_t microseconds) {
        float ms = ((float)microseconds)/1000.0;
        imu.update();
        orientation.update(imu, ms);
        output.update(orientation, ms);
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

        /*AIRSETTING index="16" name="Yaw Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the yaw stick can adjust the yaw setpoint
         *in half turns per second
         */
        settings.attach(16, 1.0f, [](float g){ YawTargetSlewRate = g; });

        /*AIRSETTING index="17" name="Altitude Hold Slew Rate" min="0" max="10.0" def="1.0"
         *The maximum rate at which the throttle stick can adjust the altitude
         *hold setpoint in feet per second
         */
        settings.attach(17, 1.0f, [](float g){ AltitudeTargetSlewRate = g; });

        /*AIRSETTING index="18" name="Barometer Gain" min="0.0" max="1.0" def="0.046"
         * The altitude estimate's sensitivity to changes in barometer readings
         * 1.0 implies the altitude estimate is the exact barometer value
         * 0.0 implies the altitude estimate is not effected by the barometer at all
         */
        settings.attach(18, 0.046f, [](float g){ altitude.setBarometerGain(g); });

        /*AIRSETTING index="19" name="Velocity Gain" min="0.0" max="1.0" def="0.020"
         * The vertical velocity estimate's sensitivity
         * 1.0 implies the vertical velocity estimate updates rapidly
         * 0.0 implies the vertical velocity estimate never changes
         */
        settings.attach(19, 0.020f, [](float g){ altitude.setVelocityGain(g); });

        /*AIRSETTING index="20" name="Altitude Response" min="0.0" max="1.0" def="0.090"
         * How powerful the quadcopter's responses to unwanted changes in altitude are
         */
        settings.attach(20, 0.050f, [](float g){ altitudeHold.setResponseFactor(g); });

        /*AIRSETTING index="21" name="Altitude Velocity Factor" min="0.0" max="1.0" def="-2.00"
         * How much the quadcopter's vertical velocity impacts its altitude hold control
         */
        settings.attach(21, 0.000f, [](float g){ altitudeHold.setVelocityFactor(g); });

        /*AIRSETTING index="22" name="Altitude Integral Factor" min="0.0" max="1.0" def="0.004"
         * How much the quadcopter's integrated altitude error contributes to its
         * overall altitude corrections
         */
        settings.attach(22, 0.004f, [](float g){ altitudeHold.setIntegralFactor(g); });
    }
}
#endif
