#ifndef AFROESC_OUTPUT_DEV_H
#define AFROESC_OUTPUT_DEV_H

#include "APM/ServoGenerator.h"
#include "math/Algebra.h"
#include "output/OutputDevice.h"
// wrapper for arduino Servo Library to OutputDevice type
class AfroESC : public OutputDevice {
  private:
    constexpr static uint16_t STOP = 1060;
    constexpr static uint16_t IDLE = 1135;
    constexpr static uint16_t FULL = 1860;
    constexpr static uint16_t RANGE = FULL - STOP;
    ServoGenerator::Servo servo;
    uint8_t pin;

  public:
    AfroESC(uint8_t in) : pin(in) {}
    ~AfroESC() { stop(); }
    void startArming() { servo.attach(pin); }
    boolean continueArming(uint32_t dt) {
        if (dt < 3500) {
            servo.writeMicroseconds(STOP);
            return false;
        }
        return true;
    }
    void startCalibrate() { servo.attach(pin); }
    boolean continueCalibrate(uint32_t dt) {
        if (dt < 5000) {
            servo.writeMicroseconds(FULL);
            return false;
        }
        if (dt < 10000) {
            servo.writeMicroseconds(STOP);
            return false;
        }
        return true;
    }
    void set(float in) {
        if (in >= 0.0f) {
            servo.writeMicroseconds(max(in * RANGE + STOP, IDLE));
        } else {
            servo.writeMicroseconds(STOP);
        }
    }
    void stop() { servo.detach(); }
    float get() { return 0; }       //((float)servo.readMicroseconds()-MIN)/RANGE; }
    uint16_t getRaw() { return 0; } // servo.readMicroseconds(); }
};

#endif
