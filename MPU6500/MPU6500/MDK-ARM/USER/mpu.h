#ifndef __USER_MPU
#define __USER_MPU

#include "stm32f4xx_hal.h"

#define MPU6500_CS_ON  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET)
#define MPU6500_CS_OFF HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET)

#define WHO_AM_I      0x75		//ID 0x70
#define PWR_MGMT_1    0x6B    //电源管理  取0x00正常使用

#define	SMPLRT_DIV		0x19	  //陀螺仪采样率  0x07(125Hz)
#define	CONFIG				0x1A	  //低通滤波频率  0x06(5Hz)
#define	GYRO_CONFIG		0x1B	  //陀螺仪自检及测量范围
#define	ACCEL_CONFIG	0x1C	  //加速计自检、测量范围及高通滤波频率
#define USER_CTRL     0x6A		//设置为SPI模式 0x11


#define	ACCEL_XOUT_H	0x3B    //加速度数据读取地址
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	GYRO_XOUT_H		0x43    //角度数据读取地址
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48


#define GYRO_RANGE    0x18    /*2000deg/s  0x18  角速度量程
															  1000deg/s  0x10
																500deg/s   0x08
																250deg/s   0x00  */
															
#define ACCEL_RANGE   0x09    /*16g        0x19  加速度量程
																8g         0x11
																4g         0x09
																2g         0x01  */
																
#define LPFq2         0.01f  //角速度低通滤波系数
#define LPFq3         0.1f  //加速度低通滤波系数

uint8_t MPU_Read_Byte(uint8_t reg);
void MPU_Write_Byte(uint8_t reg, uint8_t data);
void MPU_INIT(void);
static void MPU_READ_RAW_GYRO(void);
static void MPU_READ_RAW_ACCEL(void);
void MPU_READ_GYRO(float *GyroData, float *offset);
void MPU_READ_ACCEL(float *AccelData, float *offset);
void MPU_Calibration(float *GyroOffset, float *AccelOffset);




#endif  //__USER_MPU

