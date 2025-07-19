#include "altimu.h"


// ########################################################### //
// CONSTANTS
// ########################################################### //

// Accelerometer (g)
static float Ax, Ay, Az;
// Gyroscope (dps)
static float Gx, Gy, Gz;
// Magnetometer (gauss)
static float Mx, My, Mz;
// Temperature (°C)
static float T;
// Pressure (hPa)
static float P;

// Calibrations
static float cal_acc[3] = {0, 0, 0};
static float cal_gyr[3] = {0, 0, 0};
static uint8_t calibrated = 0;
static float magB[3] = {0, 0, 0};
static float magS[3] = {1, 1, 1};

// Euler angles
static float yaw, pitch, roll;
static float pitch_f, roll_f;

// ==================================================================
// INITIALISATION
// ==================================================================

extern AltIMU altimu;


void init_altimu()
{
	if (!LSM303D_Init())
	{
		log_error("FAIL - (LSM303D_INIT)");
		return;
	}

	log_debug("OK - (LSM303D_INIT)");
	HAL_Delay(100);

	if (!L3GD20H_Init())
	{
		log_error("FAIL - (LG3D20H_INIT)");
		return;
	}

	log_debug("OK - (LG3D20H_INIT)");
	HAL_Delay(100);

	if (!LPS25H_Init())
	{
		log_error("FAIL - (LPS25H_INIT)");
		return;
	}

	log_debug("OK - (LPS25H_INIT)");
	log_debug("OK - (ALTIMU_INIT)");
}

uint8_t LSM303D_Init()
{
	uint8_t check;
	uint8_t Data;

	// Read WHO_AM_I register
	HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x0F, 1, &check, 1, 1000);

	// 0x68 will be returned by the sensor if everything goes well
	if (check == 0x49)
	{
		// CTRL1 Register
		Data = 0b10010111;
		HAL_I2C_Mem_Write(&hi2c2, LSM303D_ADDR, 0x20, 1, &Data, 1, 1000);

		// CTRL2 Register
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, LSM303D_ADDR, 0x21, 1, &Data, 1, 1000);

		// CTRL5 Register
		Data = 0b11110000;
		HAL_I2C_Mem_Write(&hi2c2, LSM303D_ADDR, 0x24, 1, &Data, 1, 1000);

		// CTRL6 Register
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, LSM303D_ADDR, 0x25, 1, &Data, 1, 1000);

		// CTRL7 Register
		Data = 0x00000000;
		HAL_I2C_Mem_Write(&hi2c2, LSM303D_ADDR, 0x26, 1, &Data, 1, 1000);

		return 1;
	}

	return 0;
}

uint8_t L3GD20H_Init()
{
	uint8_t check;
	uint8_t Data;

	// Read WHO_AM_I register
	HAL_I2C_Mem_Read(&hi2c2, L3GD20H_ADDR, 0x0F, 1, &check, 1, 100);

	// 0x68 will be returned by the sensor if everything goes well
	if (check == 0xD7)
	{
		// CTRL1 Register
		Data = 0xFF;
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x20, 1, &Data, 1, 100);

		// CTRL2,3,4,5 Registers
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x21, 1, &Data, 1, 100);
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x22, 1, &Data, 1, 100);
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x23, 1, &Data, 1, 100);
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x24, 1, &Data, 1, 100);

		// LOW_ODR Register
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c2, L3GD20H_ADDR, 0x39, 1, &Data, 1, 100);

		return 1;
	}

	return 0;
}

uint8_t LPS25H_Init()
{
	uint8_t check;
	uint8_t Data;

	// Read WHO_AM_I register
	HAL_I2C_Mem_Read(&hi2c2, LPS25H_ADDR, 0x0F, 1, &check, 1, 100);

	// 0x68 will be returned by the sensor if everything goes well
	if (check == 0xBD)
	{

		// CTRL1 Register
		Data = 0b10010010;
		HAL_I2C_Mem_Write(&hi2c2, LPS25H_ADDR, 0x20, 1, &Data, 1, 100);

		// CTRL2,3,4 Registers
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, LPS25H_ADDR, 0x21, 1, &Data, 1, 100);
		HAL_I2C_Mem_Write(&hi2c2, LPS25H_ADDR, 0x22, 1, &Data, 1, 100);
		HAL_I2C_Mem_Write(&hi2c2, LPS25H_ADDR, 0x23, 1, &Data, 1, 100);

		return 1;
	}

	return 0;
}

// ==================================================================
// MPU AND MAGNETOMETER SENSORS UTILITY FUNCTIONS
// ==================================================================

void AltIMUv4_Read_Accel()
{
	if (calibrated == 0)
		AltIMUv4_Calibrate_GyroAcc();

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from OUT_X_L_A (0x28) register
	HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x28 | (1 << 7), 1, Rec_Data, 6, 100);

	int16_t Acc_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
	int16_t Acc_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
	int16_t Acc_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

	Ax = (Acc_X_RAW / 16384.0) - cal_acc[0];      // x
	Ay = (Acc_Y_RAW / 16384.0) - cal_acc[1];      // y
	Az = (Acc_Z_RAW / 16384.0) - cal_acc[2] + 1;  // z

	altimu.Ax = Ax;
	altimu.Ay = Ay;
	altimu.Az = Az;
}

void AltIMUv4_Read_Gyro()
{
	if (calibrated == 0)
		AltIMUv4_Calibrate_GyroAcc();

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from OUT_X_L_A (0x28) register
	HAL_I2C_Mem_Read(&hi2c2, L3GD20H_ADDR, 0x28 | (1 << 7), 1, Rec_Data, 6, 100);

	int16_t Gyro_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
	int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
	int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

	Gx = (Gyro_X_RAW / 114.0) - cal_gyr[0];  // x
	Gy = (Gyro_Y_RAW / 114.0) - cal_gyr[1];  // y
	Gz = (Gyro_Z_RAW / 114.0) - cal_gyr[2];  // z

	altimu.Gx = Gx;
	altimu.Gy = Gy;
	altimu.Gz = Gz;
}

void AltIMUv4_Read_Temp_1()
{
	uint8_t Rec_Data[2];

	// Read 2 BYTES of data starting from TEMP_OUT_L (0x05) register
	HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x05 | (1 << 7), 1, Rec_Data, 2, 100);

	T = ((int16_t)(Rec_Data[1] << 8 | Rec_Data[0]) / 8.0) + 25.0;
	altimu.T = T;
}

void AltIMUv4_Read_Temp_2()
{
	uint8_t Rec_Data[2];

	// Read 2 BYTES of data starting from TEMP_OUT_L (0x2B) register
	HAL_I2C_Mem_Read(&hi2c2, LPS25H_ADDR, 0x2B | (1 << 7), 1, Rec_Data, 2, 100);

	T = ((int16_t)(Rec_Data[1] << 8 | Rec_Data[0]) / 480.0) + 42.5;
	altimu.T = T;
}

void AltIMUv4_Read_Temp_3()
{
	uint8_t Rec_Data;

	// Read 1 BYTE of data starting from OUT_TEMP (0x26) register
	HAL_I2C_Mem_Read(&hi2c2, L3GD20H_ADDR, 0x26, 1, &Rec_Data, 1, 100);

	T = -(int8_t)Rec_Data;
	altimu.T = T;
}

void AltIMUv4_Read_Mag()
{
	if (calibrated != 2)
		AltIMUv4_Calibrate_Mag();

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from OUT_X_L_M (0x08) register
	HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x08 | (1 << 7), 1, Rec_Data, 6, 100);

	int16_t Mag_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
	int16_t Mag_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
	int16_t Mag_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

	Mx = (Mag_X_RAW / 16384.0) * magS[0] - magB[0];  // x
	My = (Mag_Y_RAW / 16384.0) * magS[1] - magB[1];  // y
	Mz = (Mag_Z_RAW / 16384.0) * magS[2] - magB[2];  // z

	altimu.Mx = Mx;
	altimu.My = My;
	altimu.Mz = Mz;
}

void AltIMUv4_Read_Pressure()
{
	uint8_t Rec_Data[3];

	// Read 3 BYTES of data starting from PRESS_OUT_XL (0x28) register
	HAL_I2C_Mem_Read(&hi2c2, LPS25H_ADDR, 0x28 | (1 << 7), 1, Rec_Data, 3, 100);

	P = (int16_t)((Rec_Data[2] << 16) | (Rec_Data[1] << 8) | Rec_Data[0]) / 4096.0;
	altimu.P = P;
}

void AltIMUv4_Read_All()
{
	AltIMUv4_Read_Accel();
	AltIMUv4_Read_Mag();
	AltIMUv4_Read_Gyro();
	AltIMUv4_Read_Pressure();
	AltIMUv4_Read_Temp_2();
}

void AltIMUv4_Calibrate_GyroAcc()
{
	log_debug("Calibrating Gyroscope & Accelerometer ...");
	calibrated = 1;

	int16_t Acc_X_RAW, Acc_Y_RAW, Acc_Z_RAW;
	int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

	uint8_t Rec_Data[6];

	for(uint8_t i = 0; i < N; i++)
	{
		// Read 6 BYTES of data starting from OUT_X_L_A (0x28) register
		HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x28 | (1 << 7), 1, Rec_Data, 6, 100);

		Acc_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
		Acc_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
		Acc_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

		cal_acc[0] += Acc_X_RAW / 16384.0;
		cal_acc[1] += Acc_Y_RAW / 16384.0;
		cal_acc[2] += Acc_Z_RAW / 16384.0;

		// Read 6 BYTES of data starting from OUT_X_L_A (0x28) register
		HAL_I2C_Mem_Read(&hi2c2, L3GD20H_ADDR, 0x28 | (1 << 7), 1, Rec_Data, 6, 100);

		Gyro_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

		cal_gyr[1] += Gyro_X_RAW / 114.0;
		cal_gyr[2] += Gyro_Y_RAW / 114.0;
		cal_gyr[3] += Gyro_Z_RAW / 114.0;

		HAL_Delay(5);
	}

	// On calcule la moyenne
	cal_acc[0] /= N;
	cal_acc[1] /= N;
	cal_acc[2] /= N;

	cal_gyr[0] /= N;
	cal_gyr[1] /= N;
	cal_gyr[2] /= N;

	log_info("Accelerometer Bias: (Ax = %.2f, Ay = %.2f, Az = %.2f)", cal_acc[0], cal_acc[1], cal_acc[2]);
	log_info("Gyroscope Bias (Gx = %.2f, Gy = %.2f, Gz = %.2f)", cal_acc[0], cal_acc[1], cal_acc[2]);
	log_debug("AltIMU Gyro/Acc calibrated succesfully !");
}

void AltIMUv4_Calibrate_Mag()
{
	log_debug("Calibrating Magnetometer ...");
	calibrated = 2;

	float max_Bx = 0;
	float max_By = 0;
	float max_Bz = 0;
	float min_Bx = 10000;
	float min_By = 10000;
	float min_Bz = 10000;
	float Rx, Ry, Rz, R;

	int16_t Mag_X_RAW, Mag_Y_RAW, Mag_Z_RAW;
	uint8_t Rec_Data[6];

	for (uint16_t i = 0; i < 500; i++)
	{
		// Read 6 BYTES of data starting from OUT_X_L_M (0x08) register
		HAL_I2C_Mem_Read(&hi2c2, LSM303D_ADDR, 0x08 | (1 << 7), 1, Rec_Data, 6, 100);

		Mag_X_RAW = (int16_t)(Rec_Data[1] << 8 | Rec_Data[0]);
		Mag_Y_RAW = (int16_t)(Rec_Data[3] << 8 | Rec_Data[2]);
		Mag_Z_RAW = (int16_t)(Rec_Data[5] << 8 | Rec_Data[4]);

		Mx = Mag_X_RAW / 16384.0;
		My = Mag_Y_RAW / 16384.0;
		Mz = Mag_Z_RAW / 16384.0;

		if (Mx > max_Bx) max_Bx = Mx;
		if (My > max_By) max_By = My;
		if (Mz > max_Bz) max_Bz = Mz;

		if (Mx < min_Bx) min_Bx = Mx;
		if (My < min_By) min_By = My;
		if (Mz < min_Bz) min_Bz = Mz;

		HAL_Delay(20);
	}

	magB[0] = (min_Bx + max_Bx) / 2.0;
	magB[1] = (min_By + max_By) / 2.0;
	magB[2] = (min_Bz + max_Bz) / 2.0;

	Rx = (max_Bx - min_Bx) / 2.0;
	Ry = (max_By - min_By) / 2.0;
	Rz = (max_Bz - min_Bz) / 2.0;

	R = (Rx + Ry + Rz) / 3.0;

	magS[0] = R / Rx;
	magS[1] = R / Ry;
	magS[2] = R / Rz;

	log_info("Magnetometer Bias (Bx = %.2f, By = %.2f, Bz = %.2f)", magB[0], magB[1], magB[2]);
	log_info("Magnetometer Scale (Sx = %.2f, Sy = %.2f, Sz = %.2f)", magS[0], magS[1], magS[2]);
	log_debug("AltIMU Magnetometer calibrated succesfully !");
}

/*
 * Tilt-compsented eCompass from Accelerometer and Magnetometer
 */

void AltIMUv4_Compass()
{
	float cosR, sinR, sinP;

	roll = atan2f(Ay, Az);
	cosR = cos(roll);
	sinR = sin(roll);

	pitch = atan2f(-Ax, Ay*sinR + Az*cosR);

	if (pitch > M_PI / 2) pitch = M_PI - pitch;
	else if (pitch < -M_PI / 2) pitch = -M_PI - pitch;

	sinP = sin(pitch);
	yaw = atan2f(Mz*sinR - My*cosR, Mx*cos(pitch) + My*sinP*sinR + Mz*sinP*cosR);

	roll  *= 180 / M_PI;
    pitch *= 180 / M_PI;
	yaw   *= 180 / M_PI;

	altimu.roll = roll;
	altimu.pitch = pitch;
	altimu.yaw = yaw;
}


void Complementary_Filter(float alpha, float dt)
{
	pitch_f = alpha * (pitch_f + Gx * dt) + (1 - alpha) * pitch;
	roll_f = alpha * (roll_f + Gy * dt) + (1 - alpha) * roll;

	altimu.rollF = roll_f;
	altimu.pitchF = pitch_f;
}

// ==================================================================
// ==================================================================

