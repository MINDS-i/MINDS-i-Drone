#pragma once

#include "MPU6000_DMP.h"
#include <SPI.h>
#include <math.h>

#include "input/Sensor.h"

// This arduino library is based on the following
// https://storage.ning.com/topology/rest/1.0/file/get/3691047852?profile=original

// Which according to the source is based on:

// This sketch is largely based on the excellent (but complicated) sketch from Jeff Rowberg, version
//  6/21/2012, Copyright (c) 2012 J. Rowberg
//  https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

// The following is the orignal copyright notice from above link->code (github::jrowberg)

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// On the ArduIMU+ V3, output pin D4 on the ATmega328P connects to
// input pin /CS (/Chip Select) of the MPU-6000.
// const int ChipSelPin1 = 53;

#define MPU6000_DMP_DEFAULT_CS 53

// class MPU6000_DMP : public InertialVec
class MPU6000_DMP {

  protected:
    // MPU control & status variables
    boolean m_dmpReady = false;
    unsigned int m_packetSize = 42;
    unsigned int m_fifoCount;
    byte m_fifoBuffer[64];
    uint8_t m_chipSelect = MPU6000_DMP_DEFAULT_CS;
    byte m_devStatus = 0;

    boolean m_updateReady = false;

    int m_raw_q_w = 0;
    int m_raw_q_x = 0;
    int m_raw_q_y = 0;
    int m_raw_q_z = 0;
    float m_q_w = 0;
    float m_q_x = 0;
    float m_q_y = 0;
    float m_q_z = 0;
    int m_AcceX = 0;
    int m_AcceY = 0;
    int m_AcceZ = 0;
    float m_Ax = 0;
    float m_Ay = 0;
    float m_Az = 0;
    int m_GyroX = 0;
    int m_GyroY = 0;
    int m_GyroZ = 0;

    float m_euler_x = 0;
    float m_euler_y = 0;
    float m_euler_z = 0;

    uint32_t m_lastUpdateTime = 0;

  public:
    MPU6000_DMP(){};
    MPU6000_DMP(uint8_t chipSelect) { m_chipSelect = chipSelect; }

    void begin();
    void end();
    Sensor::Status status();
    void calibrate();

    boolean dmpReady() { return m_dmpReady; }

    boolean updateReady() { return m_updateReady; }
    void clearUpdateReady() { m_updateReady = false; }

    uint16_t irqCount();
    uint16_t irqCountClear();

    uint32_t lastUpdateTime() { return m_lastUpdateTime; }
    void setLastUpdateTime(uint32_t time) { m_lastUpdateTime = time; }

    void update();

    void getQ(float& q_w, float& q_x, float& q_y, float& q_z) {
        q_w = m_q_w;
        q_x = m_q_x;
        q_y = m_q_y;
        q_z = m_q_z;
    }
    void getEuler(float& euler_x, float& euler_y, float& euler_z) {
        euler_x = m_euler_x;
        euler_y = m_euler_y;
        euler_z = m_euler_z;
    }
    float getEulerX() { return m_euler_x; }
    float getEulerY() { return m_euler_y; }
    float getEulerZ() { return m_euler_z; }
    float get_ax() { return m_Ax; }
    float get_ay() { return m_Ay; }
    float get_az() { return m_Az; }
    float get_wx() { return m_GyroX; }
    float get_wy() { return m_GyroY; }
    float get_wz() { return m_GyroZ; }

  private:
    byte SPIread(byte reg, int ChipSelPin);
    void SPIwrite(byte reg, byte data, int ChipSelPin);
    byte SPIreadBit(byte reg, byte bitNum, int ChipSelPin);
    void SPIwriteBit(byte reg, byte bitNum, byte databit, int ChipSelPin);
    byte SPIreadBits(byte reg, byte bitStart, byte length, int ChipSelPin);
    void SPIwriteBits(byte reg, byte bitStart, byte length, byte data, int ChipSelPin);
    void SPIreadBytes(byte reg, unsigned int length, byte* data, int ChipSelPin);

    void setMemoryBank(byte bank, boolean prefetchEnabled, boolean userBank, int ChipSelPin);
    void setMemoryStartAddress(byte startaddress, int ChipSelPin);
    boolean writeDMPMemory();
    boolean verifyDMPMemory();
    boolean writeDMPConfig();
    boolean verifyDMPConfig();
    unsigned int writeDMPUpdates(unsigned int pos, byte update_number);
    unsigned int verifyDMPUpdates(unsigned int pos_verify, byte update_number);
    unsigned int getFIFOCount(int ChipSelPin);
    byte dmpInitialize();
};
