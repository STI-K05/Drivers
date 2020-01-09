#include "mpu.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "imu.h"

struct RAW_DATA {
	short gyro[3];
	short accel[3];
	float fgyro[3];
	float faccel[3];
}RAW_DATA;

struct DATA {
	float gyro[3];
	float accel[3];
};
struct DATA DATA = {0};

struct LAST_OUT {
	float gyro[3];
	float accel[3];
};
struct LAST_OUT LAST_OUT = {0};

uint8_t MPU_Read_Byte(uint8_t reg) {
	uint8_t temp = 0;
	uint8_t none = 0xff;
	reg = reg | 0x80;                  //将最高位置1，即读信号
	
	MPU6500_CS_ON;
	HAL_SPI_TransmitReceive(&hspi1, &reg, &temp, 1, 1000);
	HAL_SPI_TransmitReceive(&hspi1, &none, &temp, 1, 1000);
	MPU6500_CS_OFF;
	
	return temp;
}

void MPU_Write_Byte(uint8_t reg, uint8_t data) {
	uint8_t none = 0;
	
	MPU6500_CS_ON;
	HAL_SPI_TransmitReceive(&hspi1, &reg, &none, 1, 1000);
	HAL_SPI_TransmitReceive(&hspi1, &data, &none, 1, 1000);
	MPU6500_CS_OFF;
}

void MPU_INIT(void) {
	MPU_Write_Byte(PWR_MGMT_1, 0x00);
	HAL_Delay(100);
	MPU_Write_Byte(USER_CTRL, 0x11);
	HAL_Delay(100);
	MPU_Write_Byte(SMPLRT_DIV, 0x07);
	HAL_Delay(100);
	MPU_Write_Byte(CONFIG, 0x06);
	HAL_Delay(100);
	MPU_Write_Byte(GYRO_CONFIG, GYRO_RANGE);
	HAL_Delay(100);
	MPU_Write_Byte(ACCEL_CONFIG, ACCEL_RANGE);
	HAL_Delay(100);
	
	if(MPU_Read_Byte(WHO_AM_I) != 0x70) {
		while (1) {
			printf("ERROR ID\n");
			HAL_Delay(500);
		}
	}
}

static void MPU_READ_RAW_GYRO(void) {
	short temp1 = 0; 
	short temp2 = 0;
	int k = 0;
	
	switch(GYRO_RANGE) {
		case 0x18: k = 2000;
				break;
		case 0x10: k = 1000;
				break;
		case 0x08: k = 500;
				break;
		case 0x00: k = 250;
				break;
		}	
	temp1 = MPU_Read_Byte(GYRO_XOUT_H);
	temp2 = MPU_Read_Byte(GYRO_XOUT_L);
	RAW_DATA.gyro[0] = (temp1<<8) | temp2;
	RAW_DATA.fgyro[0] = k * (float)RAW_DATA.gyro[0] / 32786;
	DATA.gyro[0] = LOWPASS_FILTER(&LAST_OUT.gyro[0], RAW_DATA.fgyro[0], LPFq2);
	
	temp1 = MPU_Read_Byte(GYRO_YOUT_H);
	temp2 = MPU_Read_Byte(GYRO_YOUT_L);
	RAW_DATA.gyro[1] = (temp1<<8) | temp2;
	RAW_DATA.fgyro[1] = k * (float)RAW_DATA.gyro[1] / 32786;
	DATA.gyro[1] = LOWPASS_FILTER(&LAST_OUT.gyro[1], RAW_DATA.fgyro[1], LPFq2);
		
	temp1 = MPU_Read_Byte(GYRO_ZOUT_H);
	temp2 = MPU_Read_Byte(GYRO_ZOUT_L);
	RAW_DATA.gyro[2] = (temp1<<8) | temp2;
	RAW_DATA.fgyro[2] = k * (float)RAW_DATA.gyro[2] / 32786;
	DATA.gyro[2] = LOWPASS_FILTER(&LAST_OUT.gyro[2], RAW_DATA.fgyro[2], LPFq2);
}

static void MPU_READ_RAW_ACCEL(void) {
	short temp1 = 0; 
	short temp2 = 0;
	int k = 0;
	
	switch(ACCEL_RANGE) {
		case 0x19: k = 16;
				break;
		case 0x11: k = 8;
				break;
		case 0x09: k = 4;
				break;
		case 0x01: k = 2;
				break;
		}
	temp1 = MPU_Read_Byte(ACCEL_XOUT_H);
	temp2 = MPU_Read_Byte(ACCEL_XOUT_L);
	RAW_DATA.accel[0] = (temp1<<8) | temp2;
	RAW_DATA.faccel[0] = k * (float)RAW_DATA.accel[0] / 32786;
	DATA.accel[0] = LOWPASS_FILTER(&LAST_OUT.accel[0], RAW_DATA.faccel[0], LPFq3);
	
	temp1 = MPU_Read_Byte(ACCEL_YOUT_H);
	temp2 = MPU_Read_Byte(ACCEL_YOUT_L);
	RAW_DATA.accel[1] = (temp1<<8) | temp2;
	RAW_DATA.faccel[1] = k * (float)RAW_DATA.accel[1] / 32786;
	DATA.accel[1] = LOWPASS_FILTER(&LAST_OUT.accel[1], RAW_DATA.faccel[1], LPFq3);
	
	temp1 = MPU_Read_Byte(ACCEL_ZOUT_H);
	temp2 = MPU_Read_Byte(ACCEL_ZOUT_L);
	RAW_DATA.accel[2] = (temp1<<8) | temp2;
	RAW_DATA.faccel[2] = k * (float)RAW_DATA.accel[2] / 32786;
	DATA.accel[2] = LOWPASS_FILTER(&LAST_OUT.accel[2], RAW_DATA.faccel[2], LPFq3);
}

void MPU_READ_GYRO(float *GyroData, float *offset) {
	MPU_READ_RAW_GYRO();
	
	GyroData[0] = RAW_DATA.fgyro[0] - offset[0];
	GyroData[1] = RAW_DATA.fgyro[1] - offset[1];
	GyroData[2] = RAW_DATA.fgyro[2] - offset[2];
//	printf("X轴角速度%.1fdeg/s\n", GyroData[0]);
//	printf("Y轴角速度%.1fdeg/s\n", GyroData[1]);
//	printf("Z轴角速度%.1fdeg/s\n", GyroData[2]);
//	printf("\n");
}

void MPU_READ_ACCEL(float *AccelData, float *offset) {
	MPU_READ_RAW_ACCEL();
	
	AccelData[0] = RAW_DATA.faccel[0] - offset[0];
	AccelData[1] = RAW_DATA.faccel[1] - offset[1];
	AccelData[2] = RAW_DATA.faccel[2] - offset[2];
	AccelData[2] += 1;
//	printf("X轴加速度%.2fg\n", AccelData[0]);
//	printf("Y轴加速度%.2fg\n", AccelData[1]);
//	printf("Z轴加速度%.2fg\n", AccelData[2]);
//	printf("\n");
}

void MPU_Calibration(float *GyroOffset, float *AccelOffset) {
	int i = 0;
	int k = 0;
	
	for(i = 0;i < 3;i++) {
		GyroOffset[i] = 0;
		AccelOffset[i] = 0;
	}
	for(i = 0;i < 500;i++) {
		HAL_Delay(2);
		MPU_READ_RAW_GYRO();
		MPU_READ_RAW_ACCEL();
		
		for(k = 0;k < 3;k++) {
			GyroOffset[k] += RAW_DATA.fgyro[k];
			AccelOffset[k] += RAW_DATA.faccel[k];
		}
	}
	for(i = 0;i < 3;i++) {
		GyroOffset[i] /= 500;
		AccelOffset[i] /= 500;
	}
}



