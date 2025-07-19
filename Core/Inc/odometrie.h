#ifndef SRC_ODOMETRIE_H_
#define SRC_ODOMETRIE_H_


// Includes
#include <stdbool.h>
#include <math.h>

#include "main.h"
#include "defines.h"
#include "stm32f4xx_hal_gpio.h"
#include "encoders.h"
#include "moteurs.h"
#include "uart.h"
#include "pid.h"


enum Etat_Target {
	TARGET_INIT,
	TARGET_REACHING,
	TARGET_ADJUST_ANGLE,
	TARGET_REACHED,
	TARGET_EMERGENCY_STOP,
};

enum Phase {
	PHASE_ACCELERATION,
	PHASE_FREINAGE,
	PHASE_VITESSE_MAX,
	PHASE_NEUTRE,
};

typedef struct {
	float speed;
	float target_speed;
	float theta_speed;
	float target_theta_speed;
	float vitesse_max;
	float acceleration_max;
	enum Phase phase;
} Robot;

typedef struct {
	float x;
	float y;
	float theta;
	float last_theta;
	float distance;
	float last_distance;
} Position;

typedef struct {
	float x;
	float y;
	float theta;
	float distance;
	float vitesse;
	float acceleration;
	float cap;
	enum Etat_Target state;
	enum Etat_Target previous_state;
} Target;


void set_target_pos(Target *target_pos, float x, float y, float theta, bool use_correction);
void compute_odometry(Position *position, Encoder encodeurs[4]);
void odometry_set_position(Position *position, float x, float y, float theta);
void localisation_update(Position *position, Encoder encodeurs[4]);

// Déplacements
void Move_To(Target *target, float x, float y, float theta, float vitesse, float acceleration);
void Move_Of(Target *target, float dx, float dy, float dtheta, float vitesse, float acceleration);
void Target_Stop(Target *target);

// Other
void send_ACK();
void reset_position();
void reset_target(Target *target);

#endif /* SRC_ODOMETRIE_H_ */
