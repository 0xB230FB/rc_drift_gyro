#include "mpu6050.h"

void MPU6050_Init (I2C_HandleTypeDef *hi2c)
{
	uint8_t check = 0;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

	if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 100);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 100);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 100);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);
	}
}

float MPU6050_Read_Gyro (I2C_HandleTypeDef *hi2c)
{
	uint8_t Rec_Data[2];
	int16_t Gyro_Z_RAW = 0;
	float gyro_rate;

	HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 100);
	Gyro_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

	gyro_rate = Gyro_Z_RAW/131.0;
	return gyro_rate;
}

void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, uint8_t Loops) 
{
	uint8_t Data_buf[2];
	uint16_t Data = 0;
	int32_t cal_sum = 0;

	for (uint8_t i = 0; i < Loops; i++)
	{
		HAL_I2C_Mem_Read (hi2c, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Data_buf, 2, 100);
		Data = (int16_t)(Data_buf[0] << 8 | Data_buf[1]);
		cal_sum += Data;
		HAL_Delay(20);
	}

	Data = -(cal_sum / Loops) / 4;

	Data_buf[0] = Data & 0xff;
	Data_buf[1] = Data >> 8;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_RA_ZA_OFFS_H, 1, Data_buf, 2, 100);
}
