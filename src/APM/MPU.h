#include <SPI.h>

#ifndef MPULIB
#define MPULIB

#define MPU6000_CHIP_SELECT_PIN 53 //53 pin on megas, would be 4 on an uno
#define SAMPLE_RATE 50

#define REG_WHOAMI 			0x75
#define REG_SMPLRT_DIV 		0x19
#define REG_CONFIG 			0x1A
#define REG_GYRO_CONFIG 	0x1B
#define REG_ACCEL_CONFIG 	0x1C
#define REG_INT_PIN_CFG 	0x37
#define REG_INT_ENABLE 		0x38
#define REG_ACCEL_XOUT_H 	0x3B
#define REG_ACCEL_XOUT_L 	0x3C
#define REG_ACCEL_YOUT_H 	0x3D
#define REG_ACCEL_YOUT_L 	0x3E
#define REG_ACCEL_ZOUT_H 	0x3F
#define REG_ACCEL_ZOUT_L 	0x40
#define REG_TEMP_OUT_H 		0x41
#define REG_TEMP_OUT_L 		0x42
#define REG_GYRO_XOUT_H 	0x43
#define REG_GYRO_XOUT_L 	0x44
#define REG_GYRO_YOUT_H 	0x45
#define REG_GYRO_YOUT_L 	0x46
#define REG_GYRO_ZOUT_H 	0x47
#define REG_GYRO_ZOUT_L 	0x48
#define REG_USER_CTRL 		0x6A
#define REG_PWR_MGMT_1 		0x6B
#define REG_PWR_MGMT_2 		0x6C

#define BIT_SLEEP 					0x40
#define BIT_H_RESET 				0x80
#define BITS_CLKSEL 				0x07
#define MPU_CLK_SEL_PLLGYROX 		0x01
#define MPU_CLK_SEL_PLLGYROZ 		0x03
#define MPU_EXT_SYNC_GYROX 			0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_RAW_RDY_EN				0x01
#define BIT_I2C_DIS					0x10

byte MPU_SPI_read(byte reg);
void MPUWrite(byte reg, byte data);
void MPU_data_int();
void InitMPU();

void MPU_Read6(int*, int*, int*, int*, int*, int*);
void MPU_Gyro(int*, int*, int*);
void MPU_Accl(int*, int*, int*);
int MPU_Ax();
int MPU_Ay();
int MPU_Az();
int MPU_Gx();
int MPU_Gy();
int MPU_Gz();
#endif
