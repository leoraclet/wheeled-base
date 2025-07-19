#include "asserv.h"


extern uint8_t nb_targets;
extern Point targets[10];
extern uint8_t flag_ack;

// ==================================================================
// INITIALISATION
// ==================================================================

void init_robot(Robot *robot)
{
	robot->speed = 0;
	robot->target_speed = 0;
	robot->target_theta_speed = 0;
	robot->theta_speed = 0;
	robot->acceleration_max = AMAX_LIN;
	robot->vitesse_max = VMAX_LIN;
	robot->phase = PHASE_NEUTRE;
}

void init_target(Target *target)
{
	target->state = TARGET_REACHED;
	target->x = 0;
	target->y = 0;
	target->theta = 0;
	target->vitesse = 0;
	target->acceleration = 0;
	target->distance = 0;
}

// ==================================================================
// CONSTANTS
// ==================================================================

const float INV_TWO_AMAX_LIN = 1 / (2 * AMAX_LIN);
const float INV_TWO_AMAX_ANG = 1 / (2 * AMAX_ANG);


// ==================================================================
// PROFILS DE VITESSE
// ==================================================================

void rampe_vitesse_lin(Position *pos, Robot *robot, float dt)
{
	if (pos->last_distance == pos->distance && pos->distance < 10)
		return;

	// Compute robot speed in mm/s
	robot->speed = fabs(pos->last_distance - pos->distance) / dt;
	pos->last_distance = pos->distance;

	float dist_tostop = robot->speed * robot->speed * INV_TWO_AMAX_LIN;

	// Freinage
	if (pos->distance < dist_tostop)
	{
		robot->target_speed = robot->target_speed - AMAX_LIN * dt;
		if (robot->phase != PHASE_FREINAGE)
		{
			robot->phase = PHASE_FREINAGE;
			//log_info("Slowing down (PHASE_FREINAGE)");
		}
	}
	// Accéleration
	else if (robot->speed < VMAX_LIN)
	{
		robot->target_speed = robot->target_speed + AMAX_LIN * dt;
		if (robot->phase != PHASE_ACCELERATION)
		{
			robot->phase = PHASE_ACCELERATION;
			//log_info("Aceclerating (PHASE_ACCELERATION)");
		}
	}
	// Régime permanent
	else
	{
		robot->target_speed = VMAX_PWM;
		if (robot->phase != PHASE_VITESSE_MAX)
		{
			robot->phase = PHASE_VITESSE_MAX;
			//log_info("Maximum speed (PHASE_VITESSE_MAX)");
		}
	}

	robot->target_speed = fminf(robot->target_speed, VMAX_PWM);
}

void rampe_vitesse_ang(Position *pos, Robot *robot, float dt)
{
	// Theta aboslute difference, because theta is in [-PI, PI]
	float theta_diff = pos->last_theta - pos->theta;
	theta_diff += (theta_diff > M_PI) ? -M_TWOPI : (theta_diff < -M_PI) ? M_TWOPI : 0;

	if (pos->theta == pos->last_theta && theta_diff < 0.001)
			return;

	robot->theta_speed = fabs(theta_diff) / dt;
	pos->last_theta = pos->theta;

	float dist_tostop = robot->theta_speed * robot->theta_speed * INV_TWO_AMAX_ANG;

	// Freinage
	if (pos->theta < dist_tostop)
	{
		robot->target_theta_speed = robot->target_theta_speed - AMAX_ANG * dt;
		if (robot->phase != PHASE_FREINAGE)
		{
			robot->phase = PHASE_FREINAGE;
			//log_info("Slowing down (PHASE_FREINAGE)");
		}
	}
	// Accéleration
	else if (robot->theta_speed < VMAX_ANG)
	{
		robot->target_theta_speed = robot->target_theta_speed + AMAX_ANG * dt;

		if (robot->phase != PHASE_ACCELERATION)
		{
			robot->phase = PHASE_ACCELERATION;
			//log_info("Aceclerating (PHASE_ACCELERATION)");
		}
	}
	// Régime permanent
	else
	{
		robot->target_theta_speed = VMAX_ANG;
		if (robot->phase != PHASE_VITESSE_MAX)
		{
			robot->phase = PHASE_VITESSE_MAX;
			//log_info("Maximum speed (PHASE_VITESSE_MAX)");
		}
	}

	robot->target_theta_speed = fminf(robot->target_theta_speed, VMAX_PWM);
}

// ==================================================================
// ASSERVISSEMENT
// ==================================================================


void Asserv_Position(Robot *robot, Target *target, Position *position, PID *pid_distance, PID *pid_theta, PID pid_moteurs[4], Encoder encodeurs[4])
{
	if (target->state == TARGET_EMERGENCY_STOP)
		return;
	if (target->state == TARGET_REACHED)
		return;

	static float Consigne[4];
	static float robot_speed[3];
	static float motor_speed[4];
	static float cosTheta, sinTheta, angle_diff, max_v, vx, vy;

	// On calcule les erreurs en X et en Y par rapport à la cible
	vx = target->x - position->x;
	vy = target->y - position->y;

	// Distance euclidienne
	position->distance = hypotf(vx, vy);

	// Différence absolue d'angle, car l'angle est dans [-PI, PI]
	angle_diff = target->theta - position->theta;
	angle_diff += (angle_diff > M_PI) ? -M_TWOPI : (angle_diff < -M_PI) ? M_TWOPI : 0;

	// PID position et angle du robot dans son référentiel
	compute_pid(pid_distance, 0.0, -position->distance, VMAX_PWM);
	compute_pid(pid_theta, 0.0, -angle_diff, 1);

	if (fabs(pid_distance->output) > 10 && target->state != TARGET_ADJUST_ANGLE)
	{
		if (target->state != TARGET_REACHING)
		{
			target->state = TARGET_REACHING;
			target->previous_state = TARGET_REACHING;
			log_info("Current Position: (%.1f, %.1f, %.1f)", position->x, position->y, position->theta);
			log_info("Reaching for (%.1f, %.1f, %.1f)", target->x, target->y, target->theta);
		}

		// Normalisation des vitesses. On prend la valeur absolue
		// car les vitesse peuvent être négatives
		max_v = fmaxf(fabs(vx), fabs(vy));

		// Consignes vitesse robot
		cosTheta = cosf(position->theta);
		sinTheta = sinf(position->theta);

		robot_speed[0] = (cosTheta * vx + sinTheta * vy) / max_v;
		robot_speed[1] = (cosTheta * vy - sinTheta * vx) / max_v;
		robot_speed[2] = pid_theta->output;

		// Calcul des consignes de vitesse pour les moteurs
		inverse_kinematic(robot_speed, motor_speed);

		// Normalisation des vitesses
		normalize_motor_speeds(motor_speed);

		// Mis à jour des consignes
		/*
		Consigne[0] = motor_speed[0] * fminf(pid_distance->output, robot->target_speed);
		Consigne[1] = motor_speed[1] * fminf(pid_distance->output, robot->target_speed);
		Consigne[2] = motor_speed[2] * fminf(pid_distance->output, robot->target_speed);
		Consigne[3] = motor_speed[3] * fminf(pid_distance->output, robot->target_speed);
		*/

		Consigne[0] = motor_speed[0] * pid_distance->output;
		Consigne[1] = motor_speed[1] * pid_distance->output;
		Consigne[2] = motor_speed[2] * pid_distance->output;
		Consigne[3] = motor_speed[3] * pid_distance->output;
	}
	else if (fabs(pid_theta->output) > 0.01)
	{
		if (target->state != TARGET_ADJUST_ANGLE)
		{
			target->state = TARGET_ADJUST_ANGLE;
			target->previous_state = TARGET_ADJUST_ANGLE;
			log_info("Adjusting angle");
			log_info("Current Position: (%.1f, %.1f, %.1f)", position->x, position->y, position->theta);
		}

		Consigne[0] = pid_theta->output * VMAX_PWM;
		Consigne[1] = pid_theta->output * VMAX_PWM;
		Consigne[2] = pid_theta->output * VMAX_PWM;
		Consigne[3] = pid_theta->output * VMAX_PWM;

		/*
		Consigne[0] = pid_theta->output * fminf(VMAX_PWM, robot->target_theta_speed);
		Consigne[1] = pid_theta->output * fminf(VMAX_PWM, robot->target_theta_speed);
		Consigne[2] = pid_theta->output * fminf(VMAX_PWM, robot->target_theta_speed);
		Consigne[3] = pid_theta->output * fminf(VMAX_PWM, robot->target_theta_speed);
		*/
	}
	else {
		if (target->state == TARGET_REACHED || flag_ack == 1)
			return;

		target->state = TARGET_REACHED;

		log_info("Target reached !");
		log_info("Current Position: (%.1f, %.1f, %.1f)", position->x, position->y, position->theta);

		// Stop motors
		Consigne[0] = 0;
		Consigne[1] = 0;
		Consigne[2] = 0;
		Consigne[3] = 0;
		controle_moteurs(Consigne);

		// FLAG ack
		flag_ack = 1;

		// Early return
		return;
	}

	// Asservissement en vitesse
	Asserv_Vitesse(target, Consigne, pid_moteurs, encodeurs, robot);
}

void Asserv_Vitesse(Target *target, float Consigne[4], PID pid_moteurs[4], Encoder encodeurs[4], Robot *robot)
{
	float Commande[4];

	// Converssion des vitesse en PWM
	encodeurs[0].vitesse_pwm = M1_Ticks2PWM(encodeurs[0].delta_tick);
	encodeurs[1].vitesse_pwm = M2_Ticks2PWM(encodeurs[1].delta_tick);
	encodeurs[2].vitesse_pwm = M3_Ticks2PWM(encodeurs[2].delta_tick);
	encodeurs[3].vitesse_pwm = M4_Ticks2PWM(encodeurs[3].delta_tick);

	// PIDs moteurs (Vitesse)
	// Carte violette
	compute_pid(&pid_moteurs[0], Consigne[0], (float)encodeurs[0].vitesse_pwm, VMAX_PWM);
	compute_pid(&pid_moteurs[1], Consigne[1], (float)encodeurs[1].vitesse_pwm, VMAX_PWM);
	compute_pid(&pid_moteurs[2], Consigne[2], (float)encodeurs[2].vitesse_pwm, VMAX_PWM);
	compute_pid(&pid_moteurs[3], Consigne[3], (float)encodeurs[3].vitesse_pwm, VMAX_PWM);

	// Commandes moteurs
	if (target->state == TARGET_ADJUST_ANGLE)
	{
		Commande[0] = pid_moteurs[0].output * 0.65;
		Commande[1] = pid_moteurs[1].output * 0.65;
		Commande[2] = pid_moteurs[2].output * 0.65;
		Commande[3] = pid_moteurs[3].output * 0.65;
	}
	else
	{
		Commande[0] = pid_moteurs[0].output;
		Commande[1] = pid_moteurs[1].output;
		Commande[2] = pid_moteurs[2].output;
		Commande[3] = pid_moteurs[3].output;
	}

	// Contrôle moteur
	controle_moteurs(Commande);
}

// ==================================================================
// QUADRAMP
// ==================================================================

float QuadRampDerivee_filtre(float consigne, float position, float vitesse)
{
	return 1.0;
}

// ==================================================================
// ==================================================================
