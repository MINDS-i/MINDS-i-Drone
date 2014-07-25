#include <SPI.h>
#include "MPU.h"
#include "Arduino.h"


volatile uint8_t MPU6000_newdata;
int accelX;
int accelY;
int accelZ;

int gyroX;
int gyroY;
int gyroZ;

int byte_H;
int byte_L;

byte MPU_SPI_read(byte reg)
{
    reg |= 0x80; //Set read flag
    digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
    SPI.transfer(reg);
    byte return_value = SPI.transfer(0);
    digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
    return return_value;
}

void MPUWrite(byte reg, byte data)
{
    digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
    SPI.transfer(reg);
    SPI.transfer(data);
    digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
    delay(1);
}

void InitMPU()
{
    SPI.begin();
    MPUWrite(REG_PWR_MGMT_1, BIT_H_RESET); //chip reset
    delay(100);
    MPUWrite(REG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); //set GyroZ clock (Necessary?)
    MPUWrite(REG_USER_CTRL, BIT_I2C_DIS); //Disable I2C as recommended on datasheet
    MPUWrite(REG_SMPLRT_DIV, ((1000/SAMPLE_RATE)-1) ); // Set Sample rate; 1khz/(value+1) = (rate)Hz
    MPUWrite(REG_CONFIG, BITS_DLPF_CFG_20HZ); //set low pass filter to 20hz
    MPUWrite(REG_GYRO_CONFIG, BITS_FS_2000DPS); //Gyro scale 2000º/s
    MPUWrite(REG_ACCEL_CONFIG, 0x08); //Accel scale 4g (Make Define)
}

void MPU_Read6(int* Ax, int* Ay, int* Az, int* Gx, int* Gy, int* Gz)
{
    byte_H = MPU_SPI_read(REG_ACCEL_XOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_XOUT_L);
    *Ax = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_ACCEL_YOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_YOUT_L);
    *Ay = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_ACCEL_ZOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_ZOUT_L);
    *Az = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_GYRO_XOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_XOUT_L);
    *Gx = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_GYRO_YOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_YOUT_L);
    *Gy = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_GYRO_ZOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_ZOUT_L);
    *Gz = (byte_H<<8)| byte_L;
}

void MPU_Gyro(int* Gx, int* Gy, int* Gz){
    byte_H = MPU_SPI_read(REG_GYRO_XOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_XOUT_L);
    *Gx = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_GYRO_YOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_YOUT_L);
    *Gy = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_GYRO_ZOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_ZOUT_L);
    *Gz = (byte_H<<8)| byte_L;
}

void MPU_Accl(int* Ax, int* Ay, int* Az){
    byte_H = MPU_SPI_read(REG_ACCEL_XOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_XOUT_L);
    *Ax = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_ACCEL_YOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_YOUT_L);
    *Ay = (byte_H<<8)| byte_L;

    byte_H = MPU_SPI_read(REG_ACCEL_ZOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_ZOUT_L);
    *Az = (byte_H<<8)| byte_L;
}

int MPU_Ax(){
    byte_H = MPU_SPI_read(REG_ACCEL_XOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_XOUT_L);
    return (byte_H<<8)| byte_L;
}
int MPU_Ay(){
    byte_H = MPU_SPI_read(REG_ACCEL_YOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_YOUT_L);
    return (byte_H<<8)| byte_L;
}
int MPU_Az(){
    byte_H = MPU_SPI_read(REG_ACCEL_ZOUT_H);
    byte_L = MPU_SPI_read(REG_ACCEL_ZOUT_L);
    return (byte_H<<8)| byte_L;
}
int MPU_Gx(){
    byte_H = MPU_SPI_read(REG_GYRO_XOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_XOUT_L);
    return (byte_H<<8)| byte_L;
}
int MPU_Gy(){
    byte_H = MPU_SPI_read(REG_GYRO_YOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_YOUT_L);
    return (byte_H<<8)| byte_L;
}
int MPU_Gz(){
    byte_H = MPU_SPI_read(REG_GYRO_ZOUT_H);
    byte_L = MPU_SPI_read(REG_GYRO_ZOUT_L);
    return (byte_H<<8)| byte_L;
}
