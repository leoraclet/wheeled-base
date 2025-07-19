#include "pid.h"


// ==================================================================
// INITIALISATION
// ==================================================================

void init_pid(PID *pid, float Kp, float Ki, float Kd, float Kc)
{
	pid->Ep = 0;
	pid->Ei = 0;
	pid->Ed = 0;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kc = Kc;
	pid->output = 0;
	pid->error = 0;
	pid->mode = PID_AUTO;
	pid->sampleTime = 5;
	pid->maxIntegral = MAX_INTEGRAL_ERROR;
	pid->maxOutput = MAX_OUTPUT;
}

// ==================================================================
// PID UTILITY FUNCTION
// ==================================================================

void reset_pid(PID *pid)
{
	pid->Ep = 0;
	pid->Ei = 0;
	pid->Ed = 0;
	pid->output = 0;
	pid->error = 0;
}

void set_tunnings(PID *pid, float Kp, float Ki, float Kd)
{
	float sample_time_sec = pid->sampleTime / 1000.0;

	pid->Kp = Kp;
	pid->Ki = Ki * sample_time_sec;
	pid->Kd = Kd / sample_time_sec;
}

void set_sample_time(PID *pid, uint16_t sample_time)
{
	if (sample_time > 0)
	{
		float ratio = sample_time / (float)pid->sampleTime;

		pid->Ki *= ratio;
		pid->Kd /= ratio;
		pid->sampleTime = sample_time;
	}
}

void set_pid_mode(PID *pid, uint8_t set_mode, float output, float input)
{
	// Initialize
	if (pid->mode == PID_MANUAL && set_mode == PID_AUTO)
	{
		pid->Ep = -input;
		pid->Ei = output;
	}

	// Set PID mode
	pid->mode = set_mode;
}


void compute_pid(PID *pid, float consigne, float input, float outputLimit)
{
	// Mode de fonctionnement du PID
	if (pid->mode == PID_MANUAL)
		return;

	// Calcul de l'erreur proportionnelle
	pid->error = consigne - input;

	// Si c'est uniquement un correcteur P
	if (pid->Ki == 0 && pid->Kd == 0)
	{
		pid->output = pid->Kp * pid->error;
	}
	// Sinon, si PID
	else
	{
		// Calcul de l'erreur dérivée
		pid->Ed = input - pid->Ep;
		pid->Ep = input;

		// Calcul de l'erreur integrale pondérée
		pid->Ei += pid->Ki * pid->error;

		// On limite la saturation de l'erreur intégrale
		if (pid->Ei > pid->maxIntegral)
			pid->Ei = pid->maxIntegral;
		else if (pid->Ei< -pid->maxIntegral)
			pid->Ei = -pid->maxIntegral;

		// Calcul de la commande
		pid->output = (pid->Kp * pid->error) + pid->Ei - (pid->Kd * pid->Ed);
	}

	// Si PID + QSM (Quasi-Sliding Model)
	if (pid->Kc != 0)
		pid->output -= pid->Kc * pid->error / (pid->error + QSM_EPSILON);

	// On limite la valeur de la commande en sortie
	if (pid->output > outputLimit)
		pid->output = outputLimit;
	else if (pid->output < -outputLimit)
		pid->output = -outputLimit;
}

// ==================================================================
// ==================================================================
