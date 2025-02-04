#ifndef STMtwi_H
#define STMtwi_H

#include "Wire.h"
#include "input/Sensor.h"

class STMtwiDev : public Sensor {
  protected:
    uint8_t ADDRESS;
    bool READ_LINE_HOLD;

  public:
    STMtwiDev(uint8_t addr, bool line) : ADDRESS(addr), READ_LINE_HOLD(line) {}
    virtual ~STMtwiDev() {}
    void write(uint8_t reg, uint8_t data);
    void batchRead(uint8_t reg, uint8_t num, uint8_t* buf);
    uint8_t read(uint8_t reg);
};
void STMtwiDev::write(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}
void STMtwiDev::batchRead(uint8_t reg, uint8_t num, uint8_t* buf) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg | 0x80);
    Wire.endTransmission(READ_LINE_HOLD);
    Wire.requestFrom(ADDRESS, (uint8_t)num);
    for (uint8_t i = 0; i < num; i++) {
        buf[i] = Wire.read();
    }
    Wire.endTransmission();
}
uint8_t STMtwiDev::read(uint8_t reg) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(READ_LINE_HOLD);
    Wire.requestFrom(ADDRESS, (uint8_t)1);
    uint8_t value = Wire.read();
    Wire.endTransmission();
    return value;
}

#endif
