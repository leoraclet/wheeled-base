#include "odometrie.h"


extern PID pid_moteurs[4];
extern PID pid_distance;
extern PID pid_theta;
extern Position position;


// ######################################## //
// CONSTANTS
// ######################################## //


const static float ODOMETRY_DXY_FACTOR = 0.353553390593274 * TICK_TO_MM;
const static float ODOMETRY_DTHETA_FACTOR = TICK_TO_RAD * RAYON_ROUE / 4.0 / DISTANCE_ROUES_CENTRE;


// ==================================================================
// INITIALISATION
// ==================================================================

void set_target_pos(Target *target_pos, float x, float y, float theta, bool use_correction)
{
	static float dx, dy;

	dx = x - position.x;
	dy = y - position.y;

	if (use_correction)
	{
		target_pos->x = x * TARGET_CORRECTION;                        // Distance on X axis (corrected)
		target_pos->y = y * TARGET_CORRECTION;                        // Distance on Y axis (corrected)
		target_pos->theta = theta;                                    // Angle of the robot on himself
		target_pos->distance = hypotf(dx, dy) * TARGET_CORRECTION;    // Euclidian distance to target position
		target_pos->cap = atan2f(dy, dx);                             // Angle between W axis and target poitn direction
	}
	else
	{
		target_pos->x = x;                        					  // Distance on X axis
		target_pos->y = y;                                            // Distance on Y axis
		target_pos->theta = theta;                                    // Angle of the robot on himself
		target_pos->distance = hypotf(dx, dy);  					  // Euclidian distance to target position
		target_pos->cap = atan2f(dy, dx);                             // Angle between W axis and target poitn direction
	}

	target_pos->state = TARGET_INIT;

	// Update position distance
	position.distance = target_pos->distance;
	position.last_distance = target_pos->distance;
}

// ==================================================================
// DEPLACEMENTS
// ==================================================================

void Move_To(Target *target, float x, float y, float theta, float vitesse, float acceleration)
{
	// Reset PIDs when changing of target position
	for (uint8_t i = 0; i < 4; i++)
		reset_pid(&pid_moteurs[i]);

	reset_pid(&pid_distance);
	reset_pid(&pid_theta);

	// Update current target to new one
	set_target_pos(target, x, y, theta, true);

	target->vitesse = vitesse;
	target->acceleration = acceleration;
}

void Move_Of(Target *target, float dx, float dy, float dtheta, float vitesse, float acceleration)
{
	// Reset PIDs when changing of target position
	for (uint8_t i = 0; i < 4; i++)
		reset_pid(&pid_moteurs[i]);

	reset_pid(&pid_distance);
	reset_pid(&pid_theta);

	// Update current target to new one according to changes (dx, dy, dtheta)
	set_target_pos(target, target->x + dx, target->y + dy, target->theta + dtheta, true);

	target->vitesse = vitesse;
	target->acceleration = acceleration;
}

void Target_Stop(Target *target)
{
	stop();
	target->state = TARGET_EMERGENCY_STOP;
}

// ==================================================================
// ODOMETRIE
// ==================================================================

void compute_odometry(Position *position, Encoder encodeurs[4])
{
	// If no movements, then exit early
	if (encodeurs[0].delta_tick == 0 && encodeurs[1].delta_tick == 0 && encodeurs[2].delta_tick == 0 && encodeurs[3].delta_tick == 0)
		return;

	// On utilise le modèle Kinematic pour calculer la vitesse linéaire du robot
	float dx = ODOMETRY_DXY_FACTOR * (-encodeurs[0].delta_tick - encodeurs[1].delta_tick + encodeurs[2].delta_tick + encodeurs[3].delta_tick);
	float dy = ODOMETRY_DXY_FACTOR * (+encodeurs[0].delta_tick - encodeurs[1].delta_tick - encodeurs[2].delta_tick + encodeurs[3].delta_tick);
	float dtheta = ODOMETRY_DTHETA_FACTOR * (encodeurs[0].delta_tick + encodeurs[1].delta_tick + encodeurs[2].delta_tick + encodeurs[3].delta_tick);

	position->theta += dtheta;

    // On met à jour la position et l'angle global du robot sur la table
	// Uniquement si le mouvement est suffisant pour ne pas être négligeable
	if (fabs(dx) > 0.00001 || fabs(dy) > 0.00001)
	{
		float cosTheta = cosf(position->theta);
		float sinTheta = sinf(position->theta);

		position->x += cosTheta * dx - sinTheta * dy;
		position->y += cosTheta * dy + sinTheta * dx;
	}

    // On limite l'angle entre PI et -PI
    if (position->theta > M_PI)
    	position->theta -= M_TWOPI;
    else if (position->theta < -M_PI)
    	position->theta += M_TWOPI;
}

void odometry_set_position(Position *position, float x, float y, float theta)
{
	position->x = x * TARGET_CORRECTION;
	position->y = y * TARGET_CORRECTION;
	position->theta = theta;
	position->last_theta = theta;

	log_info("Position updated (%.1f, %.1f, %.1f)", x, y, theta);
}


void reset_position()
{
	odometry_set_position(&position, 0, 0, 0);
}

void reset_target(Target *target)
{
	set_target_pos(target, 0, 0, 0, true);
	target->state = TARGET_REACHED;
	target->vitesse = 0;
	target->acceleration = 0;
	target->cap = 0;
}

void localisation_update(Position *position, Encoder encodeurs[4])
{
	update_encoder(&encodeurs[0], &htim3); // M1
	update_encoder(&encodeurs[1], &htim2); // M2
	update_encoder(&encodeurs[2], &htim4); // M3
	update_encoder(&encodeurs[3], &htim1); // M4

	// Calcule de l'odométrie
	compute_odometry(position, encodeurs);
}

// ==================================================================
// ==================================================================

void send_ACK()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	log_warn("Sent ACK !");
}

// ==================================================================
// ==================================================================

