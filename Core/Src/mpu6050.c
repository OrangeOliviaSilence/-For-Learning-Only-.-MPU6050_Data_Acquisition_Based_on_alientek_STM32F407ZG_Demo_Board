/*
 * mpu6050.c
 *
 *  Created on: Mar 10, 2021
 *      Author: Administrator
 */
#include "mpu6050.h"



/**
  * @brief Config the Sample Rate of MPU6050's Gyro by Calculating the SMPLART_DIV
  * @param  gyroOuputF Gyro data's outputs Frequency (could be 1kHz or 8kHz)
  * @param  SampleRate The sample rate of gyro you want to config
  * @retval uint8_t
  */
uint8_t MPU_GetSMPLART_DIV(uint8_t gyroOuputF, uint16_t SampleRate){
	uint8_t SMPLART_DIV;

	if(SampleRate < 4)
		SampleRate = 4;
	else if(SampleRate > 1000)
		SampleRate = 1000;

	if(gyroOuputF == 1)
		SMPLART_DIV = 1000 / SampleRate - 1;
	else if(gyroOuputF == 8)
		SMPLART_DIV = 8000 / SampleRate - 1;

	return SMPLART_DIV;
}



//获得加速度
void MPU_GetAcceler(short *accx, short *accy, short *accz){
	uint8_t data[6];
	extern I2C_HandleTypeDef hi2c1;
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Read(&hi2c1, (MPU_Device_Addr<<1) | 1, MPU_AcceOutputReg_StartAddr, I2C_MEMADD_SIZE_8BIT, data, 6, 0xffff) != HAL_OK);
	*accx = ((uint16_t)data[0] << 8) | data[1];
	*accy = ((uint16_t)data[2] << 8) | data[3];
	*accz = ((uint16_t)data[4] << 8) | data[5];
}



//获得加速度陀螺仪
void MPU_GetGyro(short *gx, short *gy, short *gz){
	uint8_t data[6];
	extern I2C_HandleTypeDef hi2c1;
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Read(&hi2c1, (MPU_Device_Addr<<1) | 1, MPU_GyroOutputReg_StartAddr, I2C_MEMADD_SIZE_8BIT, data, 6, 0xffff) != HAL_OK);
	*gx = ((uint16_t)(data[0]) << 8) | data[1];
	*gy = ((uint16_t)(data[2]) << 8) | data[3];
	*gz = ((uint16_t)(data[4]) << 8) | data[5];
}



//获得温度（摄氏度）
//temper1存放未处理的数据（是16位short型）；temper2存放的是处理后得到的摄氏度（是float型）
void MPU_GetTemper(short *temper1, float *temper2){
	uint8_t data[2];
	extern I2C_HandleTypeDef hi2c1;
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Read(&hi2c1, (MPU_Device_Addr<<1) | 1, MPU_TempOutputReg_StartAddr, I2C_MEMADD_SIZE_8BIT, data, 2, 0xffff) != HAL_OK);
	*temper1 = (((uint16_t)(data[0])) << 8) | data[1];
	*temper2 = 36.53 + ((float)(*temper1))/340.0;		//此处已经对数据进行处理，得到了摄氏度（此步不要放在中断回调函数中，否则会增加中断回调函数的处理时长）
}



//以下2个函数是为了方便使用移植过来的DMP库及其c文件而编写的
//正常传输完毕则return 0，否则return 1
uint8_t MPU_WriteLenByte(uint8_t addr, uint8_t reg,uint8_t len, uint8_t *buf){
	extern I2C_HandleTypeDef hi2c1;
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Write(&hi2c1, (addr<<1) | 0, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xffff) != HAL_OK);
	return 0;
}
uint8_t MPU_ReadLenByte(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf){
	extern I2C_HandleTypeDef hi2c1;
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Read(&hi2c1, (addr<<1) | 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xffff) != HAL_OK);
	return 0;
}





