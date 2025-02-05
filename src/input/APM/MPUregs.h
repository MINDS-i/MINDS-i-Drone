#ifndef MPUregs_H
#define MPUregs_H

const uint8_t REG_WHOAMI = 0x75;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_INT_PIN_CFG = 0x37;
const uint8_t REG_INT_ENABLE = 0x38;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_ACCEL_XOUT_L = 0x3C;
const uint8_t REG_ACCEL_YOUT_H = 0x3D;
const uint8_t REG_ACCEL_YOUT_L = 0x3E;
const uint8_t REG_ACCEL_ZOUT_H = 0x3F;
const uint8_t REG_ACCEL_ZOUT_L = 0x40;
const uint8_t REG_TEMP_OUT_H = 0x41;
const uint8_t REG_TEMP_OUT_L = 0x42;
const uint8_t REG_GYRO_XOUT_H = 0x43;
const uint8_t REG_GYRO_XOUT_L = 0x44;
const uint8_t REG_GYRO_YOUT_H = 0x45;
const uint8_t REG_GYRO_YOUT_L = 0x46;
const uint8_t REG_GYRO_ZOUT_H = 0x47;
const uint8_t REG_GYRO_ZOUT_L = 0x48;
const uint8_t REG_USER_CTRL = 0x6A;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_PWR_MGMT_2 = 0x6C;

const uint8_t BIT_SLEEP = 0x40;
const uint8_t BIT_H_RESET = 0x80;
const uint8_t BITS_CLKSEL = 0x07;
const uint8_t MPU_CLK_SEL_PLLGYROX = 0x01;
const uint8_t MPU_CLK_SEL_PLLGYROZ = 0x03;
const uint8_t MPU_EXT_SYNC_GYROX = 0x02;
const uint8_t BITS_FS_250DPS = 0x00;
const uint8_t BITS_FS_500DPS = 0x08;
const uint8_t BITS_FS_1000DPS = 0x10;
const uint8_t BITS_FS_2000DPS = 0x18;
const uint8_t BITS_FS_MASK = 0x18;
const uint8_t BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00;
const uint8_t BITS_DLPF_CFG_188HZ = 0x01;
const uint8_t BITS_DLPF_CFG_98HZ = 0x02;
const uint8_t BITS_DLPF_CFG_42HZ = 0x03;
const uint8_t BITS_DLPF_CFG_20HZ = 0x04;
const uint8_t BITS_DLPF_CFG_10HZ = 0x05;
const uint8_t BITS_DLPF_CFG_5HZ = 0x06;
const uint8_t BITS_DLPF_CFG_2100HZ_NOLPF = 0x07;
const uint8_t BITS_DLPF_CFG_MASK = 0x07;
const uint8_t BIT_INT_ANYRD_2CLEAR = 0x10;
const uint8_t BIT_RAW_RDY_EN = 0x01;
const uint8_t BIT_I2C_DIS = 0x10;
const uint8_t WHOIIS = 0b01101000;

const uint8_t REG_DATA_START = REG_ACCEL_XOUT_H;

#endif
