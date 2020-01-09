#include "stm32f4xx_hal.h"
#include "imu.h"
#include "usart.h"
#include "mpu.h"

static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float exInt = 0, eyInt = 0, ezInt = 0;
float out = 0, last_out[3] = {0};

struct imu_t {
	float Accel[3];
	float Gyro[3];
	float Mag[3];
	float Angle[3];
	float AccelOffset[3];
	float GyroOffset[3];
	float MagOffset[3];
};
struct imu_t imu = {0};

void IMU_INIT(void) {
	MPU_INIT();
	
	printf("请等待校准");
	MPU_Calibration(imu.GyroOffset, imu.AccelOffset);    //静态校准
}

void IMU_COMPUTE(float *gyro, float *accel, float *angle) {
	MPU_READ_GYRO(imu.Gyro, imu.GyroOffset);
	MPU_READ_ACCEL(imu.Accel, imu.AccelOffset);
	
	IMU_UPDATE(imu.Gyro, imu.Accel, imu.Angle);
	
	accel[0] = imu.Accel[0];
	accel[1] = imu.Accel[1];
	accel[2] = imu.Accel[2];
	
	gyro[0] = imu.Gyro[0];
	gyro[1] = imu.Gyro[1];
	gyro[2] = imu.Gyro[2];
	
	angle[0] = imu.Angle[0];
	angle[1] = imu.Angle[1];
	angle[2] = imu.Angle[2];
}

void IMU_UPDATE(float *gyro, float *accel, float *angle)
{
	float norm;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;
	
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	if(accel[0] * accel[1] * accel[2] == 0)
		return;

	//normalise the measurements
	norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] = accel[0] / norm;
	accel[1] = accel[1] / norm;
	accel[2] = accel[2] / norm;

	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	ex = accel[1] * vz - accel[2] * vy ;
	ey = accel[2] * vx - accel[0] * vz ;
	ez = accel[0] * vy - accel[1] * vx ;
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	gyro[0] = gyro[0] + Kp*ex + exInt;
	gyro[1] = gyro[1] + Kp*ey + eyInt;
	gyro[2] = gyro[2] + Kp*ez + ezInt;

	q0 = q0 + (-q1*gyro[0] - q2*gyro[1] - q3*gyro[2])*halfT;
	q1 = q1 + (q0*gyro[0] + q2*gyro[2] - q3*gyro[1])*halfT;
	q2 = q2 + (q0*gyro[1] - q1*gyro[2] + q3*gyro[0])*halfT;
	q3 = q3 + (q0*gyro[2] + q1*gyro[1] - q2*gyro[0])*halfT;

	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	angle[2] += gyro[2]*0.001f;

	angle[0] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //roll
	angle[1] = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
//	angle[2] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
	
	angle[0] = LOWPASS_FILTER(&last_out[0], angle[0], LPFq1);
	angle[1] = LOWPASS_FILTER(&last_out[1], angle[1], LPFq1);
//	angle[2] = LOWPASS_FILTER(&last_out[2], angle[2], LPFq1);

//	if(angle[0]>90||angle[0]<-90)
//	{
//		if(angle[1]>0)
//			angle[1]=180-angle[1];
//		if(angle[1]<0)
//			angle[1]=-(180+angle[1]);
//	}
}

float LOWPASS_FILTER(float *last_out, float in, float q) {
	float out = 0;
	out = q * in + (1 - q) * last_out[0];
	last_out[0] = out;
	return out;
}



