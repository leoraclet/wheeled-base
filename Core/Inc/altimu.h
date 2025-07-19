#ifndef INC_ALTIMU_H_
#define INC_ALTIMU_H_

#include <stdio.h>

#include "main.h"
#include "log.h"
#include "defines.h"
#include "stm32f4xx_hal_gpio.h"
#include "fusion.h"


#define N  100


extern I2C_HandleTypeDef hi2c2;


typedef struct {
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
	float Mx, My, Mz;
	float T, P;
	float q[4];
	float yaw;
	float pitch;
	float roll;
	float magB[4];
	float magS[4];
	float calGyr[3];
	float calAcc[3];
	float pitchF;
	float rollF;
	uint8_t calibrated;
} AltIMU;


uint8_t LSM303D_Init();
uint8_t L3GD20H_Init();
uint8_t LPS25H_Init();

void init_altimu();
void AltIMUv4_Read_Accel();
void AltIMUv4_Read_Gyro();
void AltIMUv4_Read_Temp_1();
void AltIMUv4_Read_Temp_2();
void AltIMUv4_Read_Temp_3();
void AltIMUv4_Read_Mag();
void AltIMUv4_Read_Pressure();
void AltIMUv4_Read_All();
void AltIMUv4_Calibrate_GyroAcc();
void AltIMUv4_Calibrate_Mag();
void AltIMUv4_Compass();


#endif /* INC_ALTIMU_H_ */
