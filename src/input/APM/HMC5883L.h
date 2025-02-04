#ifndef HMC5883L_H
#define HMC5883L_H

#include "input/InertialManager.h"
#include "util/LTATune.h"
constexpr auto HMC5883L_MAX_EXPECTED_VALUE_MAG = 9999;
constexpr auto HMC5883L_MIN_EXPECTED_VALUE_MAG = 50;

// HMC5883L Compass
class HMC5883L : public InertialVec {
  protected:
    static const uint8_t HMC_I2C_ADDR = 0x1E;
    static const uint8_t HMC_STATUS_REG = 0x09;
    LTATune LTA;
    uint8_t address;

  public:
    bool isTrueHMC5883L;
    HMC5883L() : address(HMC_I2C_ADDR) {}
    HMC5883L(uint8_t addr) : address(addr) {}
    void begin();
    void end();
    bool checkGoodValues();
    Sensor::Status status();
    void calibrate();
    void update(InertialManager& man, Translator axis);
    void tune(LTATune t);
    void rawValues(int& x, int& y, int& z);
    float getAzimuth();
};
#endif
