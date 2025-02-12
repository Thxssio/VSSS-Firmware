#ifndef IMU_H
#define IMU_H

#include "stm32g4xx_hal.h"

// Definição dos registradores da MPU-9250
#define IMU_WHO_AM_I      0x75
#define IMU_PWR_MGMT_1    0x6B
#define IMU_GYRO_CONFIG   0x1B
#define IMU_ACCEL_CONFIG  0x1C

#define IMU_ACCEL_XOUT_H  0x3B
#define IMU_ACCEL_XOUT_L  0x3C
#define IMU_ACCEL_YOUT_H  0x3D
#define IMU_ACCEL_YOUT_L  0x3E
#define IMU_ACCEL_ZOUT_H  0x3F
#define IMU_ACCEL_ZOUT_L  0x40

#define IMU_GYRO_XOUT_H   0x43
#define IMU_GYRO_XOUT_L   0x44
#define IMU_GYRO_YOUT_H   0x45
#define IMU_GYRO_YOUT_L   0x46
#define IMU_GYRO_ZOUT_H   0x47
#define IMU_GYRO_ZOUT_L   0x48

void IMU_Init(void);
void IMU_WriteReg(uint8_t reg, uint8_t data);
uint8_t IMU_ReadReg(uint8_t reg);
void IMU_ReadAccelData(int16_t *accelData);
void IMU_ReadGyroData(int16_t *gyroData);
void IMU_GetConvertedData(float *accel, float *gyro);

#endif // IMU_H
