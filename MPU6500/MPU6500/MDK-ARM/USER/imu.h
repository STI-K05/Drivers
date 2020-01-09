#ifndef __USER_IMU
#define __USER_IMU

#include "stm32f4xx_hal.h"
#include "math.h"

#define Kp 			1000.0f
#define Ki			0.008f
#define halfT		0.0005f		//1000Hz,0.5ms
#define LPFq1   0.01f	

void IMU_INIT(void);
void IMU_COMPUTE(float *gyro, float *accel, float *angle);
void IMU_UPDATE(float *gyro, float *accel, float *angle);
float LOWPASS_FILTER(float *last_out, float in, float q);


#endif //__USER_IMU

