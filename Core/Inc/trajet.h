#ifndef INC_TRAJET_H_
#define INC_TRAJET_H_


#include "main.h"
#include "defines.h"
#include "trajectoires.h"


enum Etat_Trajet {
    TRAJET_EN_COURS,
    TRAJET_TERMINE
};

typedef struct {
	float vitesse_max_mm_s;
	float acceleration_max_mm_s;
	float distance_fin_trajet_mm;
	Trajectoire trajectoire;
	float abscisse;
} Trajet;


void Trajet_init();
enum Etat_Trajet Trajet_Avance(Trajet *trajet, float dt);
void Trajet_config();
void Trajet_stop(Trajet *trajet);
float Trajet_calcul_vitesse(Trajet *trajet, float dt);
int Trajet_Terminee(Trajet *trajet, float t);
float Trajet_get_obstacle_mm();
void Trajet_inverse();
float Trajet_get_orientation_avance(Trajet *trajet);
float Trajet_get_abscisse();
void Trajet_debut_trajectoire(Trajet *trajet, Trajectoire trajectoire);


#endif /* INC_TRAJET_H_ */
