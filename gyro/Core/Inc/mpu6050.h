#ifndef MPU6050_H_
#define MPU6050_H_
//------------------------------------------------
#include "stm32f0xx_hal.h"
#include "stdint.h"
//#include "math.h"
//#include "stdio.h"
#include "stdlib.h"
//------------------------------------------------
#define MPU6050_ADDR 0xD0  // 0x68 << 1
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_USERCTRL_DMP_RESET_BIT 0x04
#define MPU6050_USERCTRL_FIFO_RESET_BIT 0x08
#define MPU6050_RA_ZA_OFFS_H 0x0A
//------------------------------------------------
void MPU6050_Init (I2C_HandleTypeDef *hi2c);
float MPU6050_Read_Gyro (I2C_HandleTypeDef *hi2c);
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, uint8_t Loops);
//------------------------------------------------
#endif /* MPU6050_H_ */
