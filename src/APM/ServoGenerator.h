#pragma once

#include "Arduino.h"
#include <util/atomic.h>

namespace ServoGenerator{
    void set(uint8_t channel, uint16_t us);
    void disable(uint8_t channel);
    bool enable(uint8_t channel, int pin);
    void setup(uint16_t refreshIntervalMicroseconds);

    class Servo{
        int8_t channel;
    public:
        Servo();
        bool attach(uint8_t arduinopin);
        void detach();
        void write(uint8_t sig);
        void writeMicroseconds(uint16_t us);
        bool attached();
    };
}
