#ifndef OUTPUT_DEVICE_H
#define OUTPUT_DEVICE_H
class OutputDevice {
  public:
    /** Begin the arming process */
    virtual void startArming() = 0;
    /**
     * Continue the arming process while `dt` milliseconds have passed
     * Returns true when arming is completed
     */
    virtual boolean continueArming(uint32_t dt) = 0;
    /** Begin the calibration process */
    virtual void startCalibrate() = 0;
    /**
     * Continue the calibrating process while `dt` milliseconds have passed
     * Returns true when arming is completed
     */
    virtual boolean continueCalibrate(uint32_t dt) = 0;
    /**
     * Set the output throttle
     * `in` < 0.0 => stopped
     * `in` == 0.0 => idle
     * `in` == 1.0 => full power
     */
    virtual void set(float in) = 0;
    /**
     * Stop the output device
     * It does not need to function again until the arming process has been
     * completed again
     */
    virtual void stop() = 0;
};
#endif
