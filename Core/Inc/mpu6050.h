#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// MPU6050 I2C 地址 (AD0 接 GND 時為 0x68，HAL 需要左移一位變成 0xD0)
#define MPU6050_ADDR 0xD0

// MPU6050 暫存器地址
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43

// 軟體模擬 I2C 腳位定義 (使用 PB8 作為 SCL, PB9 作為 SDA)
#define SCL_PIN GPIO_PIN_8
#define SCL_PORT GPIOB
#define SDA_PIN GPIO_PIN_9
#define SDA_PORT GPIOB

// 腳位輸出巨集
#define I2C_SCL_HIGH() HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW()  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_HIGH() HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW()  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ() HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN)

// 函式宣告
void MPU6050_I2C_Init(void);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t reg);
void MPU6050_Read_Data(short *accel, short *gyro);
void MPU6050_Get_Angle(float *Pitch, float *Roll, float dt);

#endif
