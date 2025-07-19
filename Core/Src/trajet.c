#include "trajet.h"

/********************************************************/
/* All variables                                        */
/********************************************************/

float abscisse;    // Position entre 0 et 1 sur la trajectoire
float position_mm; // Position en mm sur la trajectoire
float vitesse_mm_s;
float vitesse_max_trajet_mm_s=500;
float acceleration_mm_ss;

float distance_obstacle_mm;
float distance_fin_trajectoire_mm;

const float distance_pas_obstacle = 2000;
const float acceleration_mm_ss_obstacle = 500;

float vitesse_max_contrainte_obstacle;

Point position_consigne;


// ==================================================================
// INITIALISATION
// ==================================================================

void Trajet_init()
{

}

void Trajet_config()
{

}

// ==================================================================
// GESTION DU TRAJET
// ==================================================================

enum Etat_Trajet Trajet_Avance(Trajet *trajet, float dt)
{
	float distance_mm;
	enum Etat_Trajet trajet_etat = TRAJET_EN_COURS;
	Point point;
	Point position;

	// Calcul de la vitesse
	vitesse_mm_s = Trajet_calcul_vitesse(trajet, dt);

	// Calcul de l'avancement en mm
	distance_mm = vitesse_mm_s * dt;
	position_mm += distance_mm;

	// Calcul de l'abscisse sur la trajectoire
	abscisse = trajectoire_avance(&trajet->trajectoire, abscisse, distance_mm);

	// Obtention du point consigne
	point = trajectoire_get_point(&trajet->trajectoire, abscisse);

	position.x = point.x;
	position.y = point.y;
	position.theta = point.theta;

	position_consigne = position;
	//Asser_Position(position);

	if(Trajet_Terminee(trajet, abscisse))
	{
		//Asser_Position_set_Pos_Maintien(position);
		trajet_etat = TRAJET_TERMINE;
	}

	return trajet_etat;
}

void Trajet_stop(Trajet *trajet)
{
    vitesse_mm_s = 0;
	Trajet_Avance(trajet, 0);
}

float Trajet_get_obstacle_mm()
{
	return distance_obstacle_mm;
}

float Trajet_get_orientation_avance(Trajet *trajet)
{
	Point point, point_suivant;
	float avance_abscisse = 0.01;

	if(abscisse >= 1)
		return 0;

	if(abscisse + avance_abscisse >= 1)
		avance_abscisse = 1 - abscisse;

	point = trajectoire_get_point(&trajet->trajectoire, abscisse);
	point_suivant = trajectoire_get_point(&trajet->trajectoire, abscisse + avance_abscisse);

	return atan2f(point_suivant.y - point.y, point_suivant.x - point.x);
}

int Trajet_Terminee(Trajet *trajet, float t)
{
    if(trajet->trajectoire.type != TRAJECTOIRE_BEZIER && trajet->trajectoire.type != TRAJECTOIRE_ROTATION)
		if (t >= 1 || trajet->distance_fin_trajet_mm < 0.1)
			return 1;

	if (t >= 0.99)
		return 1;

	return 0;
}

float Trajet_calcul_vitesse(Trajet *trajet, float dt)
{

	// Calcul de la vitesse avec acceleration
	float vitesse = vitesse_mm_s + acceleration_mm_ss * dt;

	// Calcul de la vitesse maximale due à la contrainte en fin de trajectoire (0 mm/s)
	// https://poivron-robotique.fr/Consigne-de-vitesse.html
	float distance_contrainte = trajectoire_longueur(&trajet->trajectoire) - position_mm;
	distance_fin_trajectoire_mm = distance_contrainte;

	// En cas de dépassement, on veut garder la contrainte, pour l'instant
    float vitesse_max_contrainte = 0;
	if(distance_contrainte > 0)
		vitesse_max_contrainte = sqrtf(2 * acceleration_mm_ss * distance_contrainte);

	float distance_contrainte_obstacle = Trajet_get_obstacle_mm();

	if(distance_contrainte_obstacle != DISTANCE_INVALIDE)
	{
		vitesse_max_contrainte_obstacle = sqrtf(2 * acceleration_mm_ss_obstacle * distance_contrainte_obstacle);
		if(vitesse_max_contrainte_obstacle < vitesse_max_contrainte)
			vitesse_max_contrainte = vitesse_max_contrainte_obstacle;
	}

	// Selection de la vitesse la plus faible
	return fmaxf(fmaxf(vitesse, vitesse_max_contrainte), vitesse_max_trajet_mm_s);
}

void Trajet_debut_trajectoire(Trajet *trajet, Trajectoire trajectoire)
{
	abscisse = 0;
	vitesse_mm_s = 0;
	position_mm = 0;
	trajet->trajectoire = trajectoire;
	distance_obstacle_mm = DISTANCE_INVALIDE;
}


void Trajet_inverse(Trajet *trajet)
{
    float old_abscisse = abscisse;
    float old_position_mm = position_mm;

    trajectoire_inverse(&trajet->trajectoire);
    Trajet_debut_trajectoire(trajet, trajet->trajectoire);

    abscisse = 1 - old_abscisse;

    position_mm = trajectoire_longueur(&trajet->trajectoire) - old_position_mm;
}



// ==================================================================
// ==================================================================
