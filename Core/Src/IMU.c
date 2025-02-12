#include "IMU.h"
#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi2;

#define IMU_SPI &hspi2
#define CS_PORT   GPIOB
#define CS_PIN    GPIO_PIN_12

void IMU_Init(void) {
    uint8_t check;
    uint8_t data;

    check = IMU_ReadReg(IMU_WHO_AM_I);
    if (check == 0x70) {
        data = 0x00;
        IMU_WriteReg(IMU_PWR_MGMT_1, data);
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


