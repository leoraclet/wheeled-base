#include "moteurs.h"


int16_t lastCommande[4] = {0, 0, 0, 0};

// ==================================================================
// INITIALISATION
// ==================================================================

void init_moteurs()
{
	// Initialisation des PWMs
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

// ==================================================================
// ROBOT BASIC MOVEMENTS
// ==================================================================

void move_forward(float consigne[4], float commande[4], float speed)
{
	commande[0] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

void move_backward(float consigne[4], float commande[4], float speed)
{
	commande[0] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = -fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

void move_right(float consigne[4], float commande[4], float speed)
{
	commande[0] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

void move_left(float consigne[4], float commande[4], float speed)
{
	commande[0] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = -fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

// Clock wise
void rotate_clockwise(float consigne[4], float commande[4], float speed)
{
	commande[0] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = -fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = -fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

// Counter clock wise
void rotate_counterclockwise(float consigne[4], float commande[4], float speed)
{
	commande[0] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[1] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[2] = fminf(fabs(VMAX_PWM), fabs(speed));
	commande[3] = fminf(fabs(VMAX_PWM), fabs(speed));

	consigne[0] = commande[0];
	consigne[1] = commande[1];
	consigne[2] = commande[2];
	consigne[3] = commande[3];

	// Contrôle moteur
	controle_moteurs(commande);
}

void move_m1(float speed)
{
	float commande[4] = {speed, 0, 0, 0};
	controle_moteurs(commande);
}

void move_m2(float speed)
{
	float commande[4] = {0, speed, 0, 0};
	controle_moteurs(commande);
}

void move_m3(float speed)
{
	float commande[4] = {0, 0, speed, 0};
	controle_moteurs(commande);
}

void move_m4(float speed)
{
	float commande[4] = {0, 0, 0, speed};
	controle_moteurs(commande);
}

void move_f(float speed)
{
	float commande[4] = {speed, -speed, -speed, speed};
	controle_moteurs(commande);
}

void move_b(float speed)
{
	float commande[4] = {-speed, speed, speed, -speed};
	controle_moteurs(commande);
}

void move_l(float speed)
{
	float commande[4] = {speed, speed, -speed, -speed};
	controle_moteurs(commande);
}

void move_r(float speed)
{
	float commande[4] = {-speed, -speed, speed, speed};
	controle_moteurs(commande);
}

void move_fr(float speed)
{
	float commande[4] = {0, -speed, 0, speed};
	controle_moteurs(commande);
}

void move_bl(float speed)
{
	float commande[4] = {0, speed, 0, -speed};
	controle_moteurs(commande);
}

void move_fl(float speed)
{
	float commande[4] = {speed, 0, -speed, 0};
	controle_moteurs(commande);
}

void move_br(float speed)
{
	float commande[4] = {-speed, 0, speed, 0};
	controle_moteurs(commande);
}

void rotate_cw(float speed)
{
	float commande[4] = {-speed, -speed, -speed, -speed};
	controle_moteurs(commande);
}

void rotate_ccw(float speed)
{
	float commande[4] = {speed, speed, speed, speed};
	controle_moteurs(commande);
}

void stop()
{
	float commande[4] = {0, 0, 0, 0};
	controle_moteurs(commande);
}

// ==================================================================
// MOTOR UNIT CONVERSIONS
// ==================================================================

// The multiplicator take into account the weight of the robot, adn the PWM to tick conversion
// It also take into account the sampling time (default: 10ms)

int16_t M1_Ticks2PWM(int16_t x)
{
	return (int16_t)(2 * x * DIFF_DELTA_T);
}

int16_t M2_Ticks2PWM(int16_t x)
{
	return (int16_t)(2 * x * DIFF_DELTA_T);
}

int16_t M3_Ticks2PWM(int16_t x)
{
	return (int16_t)(2 * x * DIFF_DELTA_T);
}

int16_t M4_Ticks2PWM(int16_t x)
{
	return (int16_t)(2 * x * DIFF_DELTA_T);
}

// ==================================================================
// MOTOR CONTROL
// ==================================================================

void controle_moteur_M2(uint8_t direction, uint8_t vitesse)
{
	if (direction == SENS_AVANT)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, vitesse);
	}
	else if (direction == SENS_ARRIERE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, vitesse);
	}
	else
	{
		// Fast motor stop
		if (FAST_STOP_ENABLE)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 256);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
		}
	}
}

void controle_moteur_M1(uint8_t direction, uint8_t vitesse)
{
	if (direction == SENS_AVANT)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, vitesse);
	}
	else if (direction == SENS_ARRIERE)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, vitesse);
	}
	else
	{
		// Fast motor stop
		if (FAST_STOP_ENABLE)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 256);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
		}
	}
}

void controle_moteur_M4(uint8_t direction, uint8_t vitesse)
{
	if (direction == SENS_AVANT)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, vitesse);
	}
	else if (direction == SENS_ARRIERE)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, vitesse);
	}
	else
	{
		// Fast motor stop
		if (FAST_STOP_ENABLE)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 256);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
		}
	}
}

void controle_moteur_M3(uint8_t direction, uint8_t vitesse)
{
	if (direction == SENS_AVANT)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, vitesse);
	}
	else if (direction == SENS_ARRIERE)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, vitesse);
	}
	else
	{
		// Fast motor stop
		if (FAST_STOP_ENABLE)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 256);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
		}
	}
}

void controle_moteurs(const float commande[4])
{
	///////////////////////////////////////////////////
	// M1 CONTROL
	///////////////////////////////////////////////////
	if (lastCommande[0] != (int16_t)commande[0])
	{
		// Contrôle moteur M1
		if (commande[0] > 0)
			controle_moteur_M1(SENS_AVANT, (uint8_t)commande[0]);
		else if (commande[0] < 0)
			controle_moteur_M1(SENS_ARRIERE, (uint8_t)(-commande[0]));
		else
			controle_moteur_M1(STOP, 0);

		lastCommande[0] = (int16_t)commande[0];
	}

	///////////////////////////////////////////////////
	// M2 CONTROL
	///////////////////////////////////////////////////
	if (lastCommande[1] != (int16_t)commande[1])
	{
		// Contrôle moteur M2
		if (commande[1] > 0)
			controle_moteur_M2(SENS_AVANT, (uint8_t)commande[1]);
		else if (commande[1] < 0)
			controle_moteur_M2(SENS_ARRIERE, (uint8_t)(-commande[1]));
		else
			controle_moteur_M2(STOP, 0);

		lastCommande[1] = (int16_t)commande[1];
	}

	///////////////////////////////////////////////////
	// M3 CONTROL
	///////////////////////////////////////////////////
	if (lastCommande[2] != (int16_t)commande[2])
	{
		// Contrôle moteur M3
		if (commande[2] > 0)
			controle_moteur_M3(SENS_AVANT, (uint8_t)commande[2]);
		else if (commande[2] < 0)
			controle_moteur_M3(SENS_ARRIERE, (uint8_t)(-commande[2]));
		else
			controle_moteur_M3(STOP, 0);

		lastCommande[2] = (int16_t)commande[2];
	}

	///////////////////////////////////////////////////
	// M4 CONTROL
	///////////////////////////////////////////////////
	if (lastCommande[3] != (int16_t)commande[3])
	{
		// Contrôle moteur M4
		if (commande[3] > 0)
			controle_moteur_M4(SENS_AVANT, (uint8_t)commande[3]);
		else if (commande[3] < 0)
			controle_moteur_M4(SENS_ARRIERE, (uint8_t)(-commande[3]));
		else
			controle_moteur_M4(STOP, 0);

		lastCommande[3] = (int16_t)commande[3];
	}
}

// ==================================================================
// INVERSE AND FORWARD KINEMATIC
// ==================================================================

void inverse_kinematic(float *robot_speed, float *motor_speed)
{
	static float diff_vxy, plus_vxy, thetaScaled;

	diff_vxy = robot_speed[0] - robot_speed[1];
	plus_vxy = robot_speed[0] + robot_speed[1];
	thetaScaled = robot_speed[2] * DISTANCE_ROUES_CENTRE;

	motor_speed[0] = INV_RAYON_ROUE * ((-0.35355 * diff_vxy) + thetaScaled);
	motor_speed[1] = INV_RAYON_ROUE * ((-0.35355 * plus_vxy) + thetaScaled);
	motor_speed[2] = INV_RAYON_ROUE * ((+0.35355 * diff_vxy) + thetaScaled);
	motor_speed[3] = INV_RAYON_ROUE * ((+0.35355 * plus_vxy) + thetaScaled);
}

float max_speed(float *motor_speed)
{
	return fmaxf(
		fmaxf(fabs(motor_speed[0]), fabs(motor_speed[1])),
		fmaxf(fabs(motor_speed[2]), fabs(motor_speed[3]))
	);
}

void normalize_motor_speeds(float *motor_speed)
{
	float max_v = max_speed(motor_speed);

	motor_speed[0] /= max_v;
	motor_speed[1] /= max_v;
	motor_speed[2] /= max_v;
	motor_speed[3] /= max_v;
}

// ==================================================================
// ==================================================================
