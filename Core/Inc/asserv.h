#ifndef INC_ASSERV_H_
#define INC_ASSERV_H_


#include "defines.h"
#include "pid.h"
#include "trajectoires.h"
#include "odometrie.h"


void init_robot(Robot *robot);
void init_target(Target *target);

void rampe_vitesse_lin(Position *pos, Robot *robot, float dt);
void rampe_vitesse_ang(Position *pos, Robot *robot, float dt);
void Asserv_Vitesse(Target *target, float Consigne[4], PID pid_moteurs[4], Encoder encodeurs[4], Robot *robot);
void Asserv_Position(Robot *robot, Target *target, Position *position, PID *pid_distance, PID *pid_theta, PID pid_moteurs[4], Encoder encodeurs[4]);
float QuadRampDerivee_filtre(float consigne, float position, float vitesse);

#endif /* INC_ASSERV_H_ */
