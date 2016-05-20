#pragma once

#include "Arduino.h"
#include <util/atomic.h>

namespace ServoGenerator{
    void set(int channel, uint16_t us);
    void disable(int channel);
    bool enable(int channel, int pin);
    void setup(uint16_t refreshIntervalMicroseconds);

    class Servo{
        uint8_t channel;
    public:
        Servo();
        Servo(uint16_t frameUs);
        bool attach(uint8_t arduinopin);
        void detach();
        void write(uint8_t sig);
        void writeMicroseconds(uint16_t us);
        bool attached();
    };
}
