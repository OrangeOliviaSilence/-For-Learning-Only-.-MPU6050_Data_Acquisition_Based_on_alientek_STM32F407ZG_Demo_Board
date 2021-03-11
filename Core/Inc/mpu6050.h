/*
 * MPU6050.h
 *
 *  Created on: Mar 10, 2021
 *      Author: Administrator
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

//以下是和初始化MPU6050有关的寄存器的地址
#define MPU_Device_Addr 0x68		//MPU6050的设备地址，可能为0x68或是0x69
#define MPU_PowerManageReg1_Addr 0x6b
#define MPU_PowerManageReg2_Addr 0x6c		//可以使能陀螺仪、加速度传感器
#define MPU_GyroConfigReg_Addr 0x1b
#define MPU_AcceConfigReg_Addr 0x1c
#define MPU_InterruEnableReg_Addr	0x38
#define MPU_UserControlReg_Addr	0X6a	//置零可以使得AUX_IIC无效
#define MPU_FIFOConfigReg_Addr 0x23
#define MPU_GyroSampleRateReg_Addr 0x19
#define MPU_DLPFConfigReg_Addr 0x1a
#define MPU_GyroOutputReg_StartAddr 0x43
#define MPU_AcceOutputReg_StartAddr 0x3b
#define MPU_TempOutputReg_StartAddr 0x41
#define MPU_DeviceIDReg_Addr 0x75		//这个寄存器存储的值是MPU6050的设备地址，可能为0x68或是0x69


uint8_t MPU_GetSMPLART_DIV(uint8_t gyroOuputF, uint16_t SampleRate);
void MPU_GetAcceler(short *accx, short *accy, short *accz);
void MPU_GetGyro(short *gx, short *gy, short *gz);
void MPU_GetTemper(short *temper1, float *temper2);

//以下两个函数是为了方便使用移植过来的DMP库而编写的
uint8_t MPU_WriteLenByte(uint8_t addr, uint8_t reg,uint8_t len, uint8_t *buf);
uint8_t MPU_ReadLenByte(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif /* INC_MPU6050_H_ */
