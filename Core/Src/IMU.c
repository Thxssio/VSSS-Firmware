#include "IMU.h"
#include "stm32g4xx_hal.h"
#include <math.h>

extern SPI_HandleTypeDef hspi2;

#define IMU_SPI   &hspi2
#define CS_PORT   GPIOB
#define CS_PIN    GPIO_PIN_12


float GYRO_SCALE = 65.5;
float ACCEL_SCALE = 8192.0;

void IMU_Init(void) {
    uint8_t check;
    uint8_t data;

    check = IMU_ReadReg(IMU_WHO_AM_I);
    if (check == 0x70) {
        IMU_WriteReg(IMU_PWR_MGMT_1, 0x00);
        IMU_WriteReg(IMU_GYRO_CONFIG, (1 << 3));
        IMU_WriteReg(IMU_ACCEL_CONFIG, (1 << 3));

        uint8_t gyro_config = IMU_ReadReg(IMU_GYRO_CONFIG);
        uint8_t fs_sel = (gyro_config >> 3) & 0x03;

        switch(fs_sel) {
            case 0: GYRO_SCALE = 131.0; break;  // ±250°/s
            case 1: GYRO_SCALE = 65.5;  break;  // ±500°/s
            case 2: GYRO_SCALE = 32.8;  break;  // ±1000°/s
            case 3: GYRO_SCALE = 16.4;  break;  // ±2000°/s
        }

        uint8_t accel_config = IMU_ReadReg(IMU_ACCEL_CONFIG);
        uint8_t afs_sel = (accel_config >> 3) & 0x03;

        switch(afs_sel) {
            case 0: ACCEL_SCALE = 16384.0; break;  // ±2g
            case 1: ACCEL_SCALE = 8192.0;  break;  // ±4g
            case 2: ACCEL_SCALE = 4096.0;  break;  // ±8g
            case 3: ACCEL_SCALE = 2048.0;  break;  // ±16g
        }
    }
}

void IMU_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t txData[2] = {reg, data};
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(IMU_SPI, txData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
}

uint8_t IMU_ReadReg(uint8_t reg) {
    uint8_t txData = reg | 0x80;
    uint8_t rxData;
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(IMU_SPI, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(IMU_SPI, &rxData, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
    return rxData;
}

void IMU_ReadAccelData(int16_t *accelData) {
    uint8_t rawData[6];
    rawData[0] = IMU_ReadReg(IMU_ACCEL_XOUT_H);
    rawData[1] = IMU_ReadReg(IMU_ACCEL_XOUT_L);
    rawData[2] = IMU_ReadReg(IMU_ACCEL_YOUT_H);
    rawData[3] = IMU_ReadReg(IMU_ACCEL_YOUT_L);
    rawData[4] = IMU_ReadReg(IMU_ACCEL_ZOUT_H);
    rawData[5] = IMU_ReadReg(IMU_ACCEL_ZOUT_L);

    accelData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    accelData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    accelData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void IMU_ReadGyroData(int16_t *gyroData) {
    uint8_t rawData[6];
    rawData[0] = IMU_ReadReg(IMU_GYRO_XOUT_H);
    rawData[1] = IMU_ReadReg(IMU_GYRO_XOUT_L);
    rawData[2] = IMU_ReadReg(IMU_GYRO_YOUT_H);
    rawData[3] = IMU_ReadReg(IMU_GYRO_YOUT_L);
    rawData[4] = IMU_ReadReg(IMU_GYRO_ZOUT_H);
    rawData[5] = IMU_ReadReg(IMU_GYRO_ZOUT_L);

    gyroData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    gyroData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    gyroData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void IMU_GetConvertedData(float *accel, float *gyro) {
    int16_t accelData[3], gyroData[3];
    IMU_ReadAccelData(accelData);
    IMU_ReadGyroData(gyroData);

    accel[0] = (float)accelData[0] / ACCEL_SCALE * 9.81;
    accel[1] = (float)accelData[1] / ACCEL_SCALE * 9.81;
    accel[2] = (float)accelData[2] / ACCEL_SCALE * 9.81;

    gyro[0] = (float)gyroData[0] / GYRO_SCALE * (M_PI / 180.0);
    gyro[1] = (float)gyroData[1] / GYRO_SCALE * (M_PI / 180.0);
    gyro[2] = (float)gyroData[2] / GYRO_SCALE * (M_PI / 180.0);
}
