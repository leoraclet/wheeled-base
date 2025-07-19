#include "imu.h"


// ########################################################### //
// CONSTANTS
// ########################################################### //

// Accelerometer
static float Ax, Ay, Az;
// Gyroscope
static float Gx, Gy, Gz;
// Temperature
static float T = 0;

// Kalman angles
static double KalmanAngleX = 0.0;
static double KalmanAngleY = 0.0;

// Timer
static uint32_t timer;

// Kalman
static Kalman_t KalmanX = {
	.Q_angle = 0.001f,
	.Q_bias = 0.003f,
	.R_measure = 0.03f
};

static Kalman_t KalmanY = {
	.Q_angle = 0.001f,
	.Q_bias = 0.003f,
	.R_measure = 0.03f,
};

// Calibrations
static float cal_acc[3] = {0};
static float cal_gyr[3] = {0};
static uint8_t calibrated = 0;

// ==================================================================
// INITIALISATION
// ==================================================================

void init_mpu()
{
	// Initialisation du MPU
	if (MPU6050_Init())
	{
		log_debug("OK - (MPU_INIT)");
		return;
	}

	log_error("FAIL - (MPU_INIT)");
}

// ==================================================================
// MPU SENSOR UTILITY FUNCTIONS
// ==================================================================

void MPU6050_Calibrate()
{
	calibrated = 1;
	uint8_t Rec_Data[14];

	for(uint8_t i = 0; i < N; i++)
	{
		// Read 14 BYTES of data starting from ACCEL_XOUT_H register
		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 100);

		int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 |  Rec_Data[1]);
		int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 |  Rec_Data[3]);
		int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 |  Rec_Data[5]);

		int16_t Gyro_X_RAW = (int16_t)(Rec_Data[8]  << 8 |  Rec_Data[9]);
		int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
		int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

		cal_acc[0] += (float)Accel_X_RAW / 16384.0;  // Ax
		cal_acc[1] += (float)Accel_Y_RAW / 16384.0;  // Ay
		cal_acc[2] += (float)Accel_Z_RAW / 16384.0;  // Az

		cal_gyr[1] += (float)Gyro_X_RAW / 131.0;  // Gx
		cal_gyr[2] += (float)Gyro_Y_RAW / 131.0;  // Gy
		cal_gyr[3] += (float)Gyro_Z_RAW / 131.0;  // Gz
	}

	// On calcule la moyenne
	cal_acc[0] /= N;
	cal_acc[1] /= N;
	cal_acc[2] /= N;

	cal_gyr[0] /= N;
	cal_gyr[1] /= N;
	cal_gyr[2] /= N;

	log_info("MPU6050 Gyro/Acc calibrated succesfully !");
}

uint8_t MPU6050_Init()
{
	uint8_t check;
	uint8_t Data;

	// Read WHO_AM_I register
	if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 1000) != HAL_OK)
		return 0;

	// 0x68 will be returned by the sensor if everything goes well
	if (check == 0x72)
	{
		// Power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &Data, 1, 100);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x19, 1, &Data, 1, 100);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g -> 16384 LSB/g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 100);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s -> 131 LSB/(°/s)
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);

		return 1;
	}

	return 0;
}

void MPU6050_Read_Accel()
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 100);

	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = (Accel_X_RAW / 16384.0) - cal_acc[0];      // x
	Ay = (Accel_Y_RAW / 16384.0) - cal_acc[1];      // y
	Az = (Accel_Z_RAW / 16384.0) - cal_acc[2] + 1;  // z
}

void MPU6050_Read_Gyro()
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H (0x43) register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 100);

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	Gx = (Gyro_X_RAW / 131.0) - cal_gyr[0];
	Gy = (Gyro_Y_RAW / 131.0) - cal_gyr[1];
	Gz = (Gyro_Z_RAW / 131.0) - cal_gyr[2];
}

void MPU6050_Read_Temp()
{
	uint8_t Rec_Data[2];

	// Read 6 BYTES of data starting from GYRO_XOUT_H (0x41) register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x41, 1, Rec_Data, 2, 100);

	int16_t Temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	T = (float)Temp / 340.0 + 36.53;
}

void MPU6050_Read_All()
{
	// If not already calibrated, calibrate gyroscope and accelerometer
	// using a single point procedure calibration
	if (!calibrated)
		MPU6050_Calibrate();

	uint8_t Rec_Data[14];

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 100);

	int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 |  Rec_Data[1]);
	int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 |  Rec_Data[3]);
	int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 |  Rec_Data[5]);

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[8]  << 8 |  Rec_Data[9]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	Ax = (Accel_X_RAW / 16384.0) - cal_acc[0];
	Ay = (Accel_Y_RAW / 16384.0) - cal_acc[1];
	Az = (Accel_Z_RAW / 16384.0) - cal_acc[2] + 1; // +1 To account for gravity, because Acceleration toward Z shouldn't move during calibration

	Gx = (Gyro_X_RAW / 131.0) - cal_gyr[0];
	Gy = (Gyro_Y_RAW / 131.0) - cal_gyr[1];
	Gz = (Gyro_Z_RAW / 131.0) - cal_gyr[2];

	///////////////////////////////////////
	// Kalman angle solve
	///////////////////////////////////////

	double dt = (double) (HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();

	double roll;
	double roll_sqrt = sqrt(Accel_X_RAW * Accel_X_RAW + Accel_Z_RAW * Accel_Z_RAW);

	if (roll_sqrt != 0.0) {
		roll = atan(Accel_Y_RAW / roll_sqrt) * RAD2DEG;
	} else {
		roll = 0.0;
	}

	double pitch = atan2(-Accel_X_RAW, Accel_Z_RAW) * RAD2DEG;

	if ((pitch < -90 && KalmanAngleY > 90) || (pitch > 90 && KalmanAngleY < -90)) {
		KalmanY.angle = pitch;
		KalmanAngleY = pitch;
	} else {
		KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, Gy, dt);
	}

	if (fabs(KalmanAngleY) > 90)
		Gx = -Gx;

	KalmanAngleX = Kalman_getAngle(&KalmanX, roll, Gy, dt);
}

// ==================================================================
// KALMAN UTILITY FUNCTIONS
// ==================================================================

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
	double rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	double S = Kalman->P[0][0] + Kalman->R_measure;
	double K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	double y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	double P00_temp = Kalman->P[0][0];
	double P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}

// ==================================================================
// ==================================================================

