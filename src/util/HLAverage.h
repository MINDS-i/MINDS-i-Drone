#ifndef HLA_H
#define HLA_H
#include "Arduino.h"
#include "math.h"

// Time controlled exponential running average
// set the half-life in milliseconds
class HLA {
  private:
    float halfLife;
    float value;
    uint32_t time;

  public:
    /**
     * Construct a HalfLifeAverage setting half life to `half` milliseconds
     */
    HLA(float half, float init) : halfLife(half * 1000.l), value(init) { time = micros(); }
    float update(float newValue) {
        if (isnan(newValue))
            return value;
        uint32_t cTime = micros();
        float dt = float(cTime - time);
        float factor = pow(2.l, -dt / halfLife);
        value = (1.l - factor) * newValue + factor * value;
        time = cTime;
        return value;
    }
    float get() { return value; }
    float millisSinceUpdate() { return (micros() - time) / 1000.l; }
    uint32_t microsSinceUpdate() { return micros() - time; }
    void set(float newValue) {
        if (isnan(newValue))
            return;
        value = newValue;
    }
    void resetTime() { time = micros(); }
    void setHalfLife(float half) { halfLife = half * 1000.0f; }
};
#endif
