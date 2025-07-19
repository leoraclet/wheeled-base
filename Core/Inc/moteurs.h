#ifndef SRC_MOTEURS_H_
#define SRC_MOTEURS_H_

#include <math.h>

#include "main.h"
#include "defines.h"
#include "stm32f4xx_hal_gpio.h"
#include "encoders.h"
#include "uart.h"
#include "odometrie.h"

// PWMs
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;


void init_moteurs();


void move_m1(float speed);
void move_m2(float speed);
void move_m3(float speed);
void move_m4(float speed);

void move_forward(float consigne[4], float commande[4], float speed);
void move_backward(float consigne[4], float commande[4], float speed);
void move_right(float consigne[4], float commande[4], float speed);
void move_left(float consigne[4], float commande[4], float speed);
void rotate_clockwise(float consigne[4], float commande[4], float speed);
void rotate_counterclockwise(float consigne[4], float commande[4], float speed);
void move_f(float speed);
void move_b(float speed);
void move_l(float speed);
void move_r(float speed);
void move_fr(float speed);
void move_bl(float speed);
void move_fl(float speed);
void move_br(float speed);
void rotate_cw(float speed);
void rotate_ccw(float speed);
void stop();

int16_t M1_Ticks2PWM(int16_t x);
int16_t M2_Ticks2PWM(int16_t x);
int16_t M3_Ticks2PWM(int16_t x);
int16_t M4_Ticks2PWM(int16_t x);

void controle_moteur_M1(uint8_t direction, uint8_t vitesse);
void controle_moteur_M2(uint8_t direction, uint8_t vitesse);
void controle_moteur_M3(uint8_t direction, uint8_t vitesse);
void controle_moteur_M4(uint8_t direction, uint8_t vitesse);
void controle_moteurs(const float commande[4]);

void inverse_kinematic(float *robot_speed, float *motor_speed);
float max_speed(float* motor_speed);
void normalize_motor_speeds(float* motor_speed);


#endif /* SRC_MOTEURS_H_ */
