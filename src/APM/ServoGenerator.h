#pragma once

#include "Arduino.h"
#include <util/atomic.h>


namespace ServoGenerator{
    /**
     * Timer index that the ServoGenerator will run on
     * This must be a 16-bit timer
     *     1 on Uno, Leonardo
     *     1,3,4,5 on Mega
     */
    #define TIMER_NUM 1
    /**
     * The maximum number of servo output channels that can be produces
     * Each additional channel takes up more memory and each used channel
     * costs some processing time each frame.
     */
    constexpr uint8_t MAX_OUTPUTS = 8;
    /**
     * Default refresh interval in microseconds that will be used until begin
     * is explicitly called with a different interval
     */
    const uint16_t DEFAULT_REFRESH_INTERVAL = 20000; // 20ms => 50Hz
    /**
     * Function signature that will be called one each frame
     * @param  microseconds Current update interval in microseconds
     */
    typedef void (*UpdateFunc)(uint16_t microseconds);
    /**
     * Set a previously enabled channel to a particular signal width
     * @param channel Channel index [0,MAX_OUTPUTS)
     * @param us      signal high time in microseconds
     */
    void set(uint8_t channel, uint16_t us);
    /**
     * Turn off a previously enabled channel
     * @param channel Channel index [0,MAX_OUTPUTS)
     */
    void disable(uint8_t channel);
    /**
     * Enable an unused channel and attach it to an arduino pin
     * If the servo generator has not previously been started with `begin` and
     *     The channel attachment is successful, `begin` will be called with
     *     DEFAULT_REFRESH_INTERVAL as the frame interval
     * Sets `pin` to output and updates the channel to write to `pin` weather
     *     The channel was previously in use or not
     * @param  channel Channel index [0,MAX_OUTPUTS)
     * @param  pin     Arduino digital or analog pin
     * @return         true if channel was newly attached
     */
    bool enable(uint8_t channel, int pin);
    /**
     * Begin generated servo signals on all enabled channels
     * Calling after servo signals have been started will update the refresh
     *     interval but will skip a frame of signal outputs
     * @param refreshIntervalMicroseconds Microseconds between servo frames
     */
    void begin(uint16_t refreshIntervalMicroseconds = DEFAULT_REFRESH_INTERVAL);
    /**
     * Attach a function can be installed to update the servo setpoints
     * It will be called at the start of each frame after the output channels
     *     are written high, unless a previous call has not yet completed
     * Interrupts will be enabled when the callback starts
     * @param callback The function to be called
     */
    void setUpdateCallback(UpdateFunc callback);

    class Servo{
        int8_t channel;
    public:
        Servo();
        /**
         * Start generating servo signals on a given pin
         * Fails if the servo is already attached or there are no available
         *     channels
         * The pin will be set to OUTPUT mode if successful
         * @param  arduinopin The arduino pin to attach to
         * @return            If the servo was successfully attached
         */
        bool attach(uint8_t arduinopin);
        /**
         * disables servo signal generation on the pin
         */
        void detach();
        /**
         * queries this servos status
         * @return True if the servo is attached and generating a signal
         */
        bool attached();
        /**
         * Writes a servo angle value to the attached pin
         * @param sig A standard [0,180] servo signal
         */
        void write(uint8_t sig){
            if(channel != -1)
                // convert [0,180] to [600,2400]
                set(channel, ((uint16_t)sig)*10 + 600);
        }
        /**
         * Writes a specific microsecond value to the attached pin
         * @param us servo signal high time
         */
        void writeMicroseconds(uint16_t us){
            if(channel != -1)
                set(channel, us);
        }
    };
}
