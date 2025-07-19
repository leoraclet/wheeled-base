#ifndef INC_TRAJECTOIRES_H_
#define INC_TRAJECTOIRES_H_

#include <math.h>

#include "main.h"
#include "defines.h"

// ==================================================================
// ENUMERATIONS
// ==================================================================

enum Trajectoire_Type
{
	TRAJECTOIRE_BEZIER,
	TRAJECTOIRE_CERCLE,
	TRAJECTOIRE_CHEMIN,
	TRAJECTOIRE_DROITE,
	TRAJECTOIRE_ROTATION,
};

// ==================================================================
// STRUCTURES
// ==================================================================

typedef struct
{
	float x;
	float y;
	float theta;
} Point;

typedef struct
{
	float angle_debut;
	float angle_fin;
	float rayon;
	Point centre;
} Trajectoire_Cercle;

typedef struct
{
	Point p1, p2, p3, p4;
	float ax, ay;
	float bx, by;
	float cx, cy;
	uint8_t precomputed;
} Trajectoire_Bezier;

typedef struct
{
	Point points[NB_POINTS_MAX];
	uint8_t nb_points;
} Trajectoire_Chemin;

typedef struct
{
	float angle_debut;
	float angle_fin;
	Point pos;
} Trajectoire_Rotation;

typedef struct
{
	Point p1, p2;
} Trajectoire_Droite;

typedef struct
{
	enum Trajectoire_Type type;
	float longueur;
	float orientation_debut;
	float orientation_fin;
	union
	{
		Trajectoire_Cercle cercle;
		Trajectoire_Bezier bezier;
		Trajectoire_Chemin chemin;
		Trajectoire_Droite droite;
		Trajectoire_Rotation rotation;
	};
} Trajectoire;

// ==================================================================
// FUNCTIONS
// ==================================================================

void init_trajectoire_droite(
	Trajectoire *traj,
	float p1_x,
	float p1_y,
	float p2_x,
	float p2_y,
	float orientation_debut,
	float orientation_fin);

void init_trajectoire_rotation(
	Trajectoire *traj,
	float p1_x,
	float p1_y,
	float orientation_debut,
	float orientation_fin);

void init_trajectoire_cercle(
	Trajectoire *traj,
	float centre_x,
	float centre_y,
	float angle_debut_degre,
	float angle_fin_degre,
	float rayon,
	float orientation_debut_rad,
	float orientation_fin_rad);

void init_trajectoire_bezier(
	Trajectoire *traj,
	float p1_x,
	float p1_y,
	float p2_x,
	float p2_y,
	float p3_x,
	float p3_y,
	float p4_x,
	float p4_y,
	float orientation_debut_rad,
	float orientation_fin_rad);

void init_trajectoire_chemin(
	Trajectoire *traj,
	Point points[],
	uint8_t nb_points,
	float orientation_debut_rad,
	float orientation_fin_rad);

// ==================================================================
// GESTION DES TRAJECTOIRES
// ==================================================================

Point trajectoire_get_point(Trajectoire *traj, float t);
void trajectoire_inverse(Trajectoire *traj);
float trajectoire_avance(Trajectoire *traj, float t, float distance);
float trajectoire_longueur(Trajectoire *traj);
float trajectoire_get_orientation(Trajectoire *traj, float t);

// ==================================================================
// TRAJECTORY SPECIFIC FUNCTIONS
// ==================================================================

// Cercle
Point trajectoire_cercle_get_point(Trajectoire *traj, float t);
void trajectoire_cercle_longueur(Trajectoire *traj);
// Bezier
Point trajectoire_bezier_get_point(Trajectoire *traj, float t);
void trajectoire_bezier_longueur(Trajectoire *traj);
// Droite
Point trajectoire_droite_get_point(Trajectoire *traj, float t);
void trajectoire_droite_longueur(Trajectoire *traj);
// Rotation
Point trajectoire_rotation_get_point(Trajectoire *traj, float t);
void trajectoire_rotation_longueur(Trajectoire *traj);
// Chemin
Point trajectoire_chemin_get_point(Trajectoire *traj, float t);
void trajectoire_chemin_longueur(Trajectoire *traj);

#endif /* INC_TRAJECTOIRES_H_ */
