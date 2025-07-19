#include "trajectoires.h"

// Wiki sur les trajectoires
// https://www.eurobot.org/wiki/fr/informatics/trajectoires_complexes#l_arc_de_cercle

// ==================================================================
// INITIALISATION
// ==================================================================

void init_trajectoire_droite(
	Trajectoire *traj,
	float p1_x,
	float p1_y,
	float p2_x,
	float p2_y,
	float orientation_debut_rad,
	float orientation_fin_rad)
{
	traj->type = TRAJECTOIRE_DROITE;
	traj->droite.p1.x = p1_x;
	traj->droite.p1.y = p1_y;
	traj->droite.p2.x = p2_x;
	traj->droite.p2.y = p2_y;
	traj->longueur = -1;
	traj->orientation_debut = orientation_debut_rad;
	traj->orientation_fin = orientation_fin_rad;
}

void init_trajectoire_rotation(
	Trajectoire *traj,
	float p1_x,
	float p1_y,
	float orientation_debut_rad,
	float orientation_fin_rad)
{
	traj->type = TRAJECTOIRE_ROTATION;
	traj->rotation.pos.x = p1_x;
	traj->rotation.pos.y = p1_y;
	traj->rotation.angle_debut = orientation_debut_rad;
	traj->rotation.angle_fin = orientation_fin_rad;
	traj->longueur = -1;
}

void init_trajectoire_cercle(
	Trajectoire *traj,
	float centre_x,
	float centre_y,
	float angle_debut_degre,
	float angle_fin_degre,
	float rayon,
	float orientation_debut_rad,
	float orientation_fin_rad)
{
	traj->type = TRAJECTOIRE_CERCLE;
	traj->cercle.centre.x = centre_x;
	traj->cercle.centre.y = centre_y;
	traj->cercle.angle_debut = angle_debut_degre;
	traj->cercle.angle_fin = angle_fin_degre;
	traj->cercle.rayon = rayon;
	traj->longueur = -1;
	traj->orientation_debut = orientation_debut_rad;
	traj->orientation_fin = orientation_fin_rad;
}

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
	float orientation_fin_rad)
{
	traj->type = TRAJECTOIRE_BEZIER;
	traj->bezier.p1.x = p1_x;
	traj->bezier.p1.y = p1_y;
	traj->bezier.p2.x = p2_x;
	traj->bezier.p2.y = p2_y;
	traj->bezier.p3.x = p3_x;
	traj->bezier.p3.y = p3_y;
	traj->bezier.p4.x = p4_x;
	traj->bezier.p4.y = p4_y;
	traj->longueur = -1;
	traj->bezier.precomputed = 0;
	traj->orientation_debut = orientation_debut_rad;
	traj->orientation_fin = orientation_fin_rad;
}

void init_trajectoire_chemin(
	Trajectoire *traj,
	Point points[],
	uint8_t nb_points,
	float orientation_debut_rad,
	float orientation_fin_rad)
{
	if (nb_points > NB_POINTS_MAX)
		return;

	traj->type = TRAJECTOIRE_CHEMIN;
	traj->chemin.nb_points = nb_points;
	traj->longueur = -1;
	traj->orientation_debut = orientation_debut_rad;
	traj->orientation_fin = orientation_fin_rad;

	for(; nb_points >= 0; nb_points--)
		traj->chemin.points[nb_points] = points[nb_points];
}

// ==================================================================
// GESTION DES TRAJECTOIRES
// ==================================================================

float trajectoire_get_orientation(Trajectoire *traj, float t)
{
	return traj->orientation_debut * (1 - t) + traj->orientation_fin * t;
}

float trajectoire_avance(Trajectoire *traj, float t, float distance)
{
	if (distance == 0)
		return t;

	float delta_t = distance / trajectoire_longueur(traj);

	if (traj->type != TRAJECTOIRE_BEZIER)
		return delta_t + t;

    // Sur les trajectoires de bézier, il peut être nécessaire d'affiner
    // Les cas où l'algorythme diverge ne devraient pas se produire car distance_cm << longeur_trajectoire.

	Point p1 = trajectoire_get_point(traj, t);
	Point p2 = trajectoire_get_point(traj, t + delta_t);

	float delta_dist = hypotf(p2.x - p1.x, p2.y - p1.y);
	float erreur_relative = 1 - delta_dist / distance;

	while (fabs(erreur_relative) > PRECISION_ABSCISSE)
	{
		delta_t = delta_t * distance / delta_dist;

		p1 = trajectoire_get_point(traj, t);
		p2 = trajectoire_get_point(traj, t + delta_t);

		delta_dist = hypotf(p2.x - p1.x, p2.y - p1.y);
		erreur_relative = 1 - delta_dist / distance;
	}

	return t + delta_t;
}

Point trajectoire_get_point(Trajectoire *traj, float t)
{
	Point p;

	switch (traj->type)
	{
	case TRAJECTOIRE_DROITE:
		p = trajectoire_droite_get_point(traj, t);
		break;
	case TRAJECTOIRE_CERCLE:
		p = trajectoire_cercle_get_point(traj, t);
		break;
	case TRAJECTOIRE_BEZIER:
		p = trajectoire_bezier_get_point(traj, t);
		break;
	case TRAJECTOIRE_ROTATION:
		p = trajectoire_rotation_get_point(traj, t);
		break;
	case TRAJECTOIRE_CHEMIN:
		p = trajectoire_chemin_get_point(traj, t);
		break;
	}

	return p;
}

float trajectoire_longueur(Trajectoire *traj)
{
	if (traj->longueur > 0)
		return traj->longueur;

	switch (traj->type)
	{
	case TRAJECTOIRE_DROITE:
		trajectoire_droite_longueur(traj);
		break;
	case TRAJECTOIRE_CERCLE:
		trajectoire_cercle_longueur(traj);
		break;
	case TRAJECTOIRE_BEZIER:
		trajectoire_bezier_longueur(traj);
		break;
	case TRAJECTOIRE_ROTATION:
		trajectoire_rotation_longueur(traj);
		break;
	case TRAJECTOIRE_CHEMIN:
		trajectoire_chemin_longueur(traj);
		break;
	}

	return traj->longueur;
}

void trajectoire_inverse(Trajectoire *traj)
{
	Trajectoire old_traj = *traj;

	switch (traj->type)
	{
	case TRAJECTOIRE_DROITE:
		traj->droite.p1 = old_traj.droite.p2;
		traj->droite.p2 = old_traj.droite.p1;
		break;
	case TRAJECTOIRE_CERCLE:
		traj->cercle.angle_debut = old_traj.cercle.angle_fin;
		traj->cercle.angle_fin = old_traj.cercle.angle_debut;
		break;
	case TRAJECTOIRE_BEZIER:
		traj->bezier.p1 = old_traj.bezier.p4;
		traj->bezier.p2 = old_traj.bezier.p3;
		traj->bezier.p3 = old_traj.bezier.p2;
		traj->bezier.p4 = old_traj.bezier.p1;
		break;
	case TRAJECTOIRE_ROTATION:
		traj->cercle.angle_debut = old_traj.cercle.angle_fin;
		traj->cercle.angle_fin = old_traj.cercle.angle_debut;
		break;
	case TRAJECTOIRE_CHEMIN:
		break;
	}
}

// ==================================================================
// TRAJECTOIRE CERCLE
// ==================================================================

Point trajectoire_cercle_get_point(Trajectoire *traj, float t)
{
	float angle_degre = M_PI / 180.0 * (traj->cercle.angle_debut * (1 - t) + traj->cercle.angle_fin * t);

	Point p = {
		cos(angle_degre) * traj->cercle.rayon,
		sin(angle_degre) * traj->cercle.rayon,
		0};

	return p;
}

void trajectoire_cercle_longueur(Trajectoire *traj)
{
	float distance_angulaire;
	if (traj->cercle.angle_debut > traj->cercle.angle_fin)
		distance_angulaire = traj->cercle.angle_debut - traj->cercle.angle_fin;
	else
		distance_angulaire = traj->cercle.angle_fin - traj->cercle.angle_debut;

	traj->longueur = M_TWOPI * traj->cercle.rayon * (distance_angulaire / 360);
}

// ==================================================================
// TRAJECTOIRE BEZIER
// ==================================================================

// Pre-calculate all points on a Bezier curve
// http://www.pennelynn.com/Documents/CUJ/HTML/15.11/BARTLEY/BARTLEY.HTM

Point trajectoire_bezier_get_point(Trajectoire *traj, float t)
{
	float t2 = t * t;
	float t3 = t2 * t;

	// If values are not already precomputed
	if (traj->bezier.precomputed == 0)
	{
		// A (t cubed term)
		traj->bezier.ax = -traj->bezier.p1.x + 3 * traj->bezier.p2.x - 3 * traj->bezier.p3.x + traj->bezier.p4.x;
		traj->bezier.ay = -traj->bezier.p1.y + 3 * traj->bezier.p2.y - 3 * traj->bezier.p3.y + traj->bezier.p4.y;

		// B  (t squared term)
		traj->bezier.bx = 3 * traj->bezier.p1.x - 6 * traj->bezier.p2.x + 3 * traj->bezier.p3.x;
		traj->bezier.by = 3 * traj->bezier.p1.y - 6 * traj->bezier.p2.y + 3 * traj->bezier.p3.y;

		// C (t term)
		traj->bezier.cx = -3 * traj->bezier.p1.x + 3 * traj->bezier.p2.x;
		traj->bezier.cy = -3 * traj->bezier.p1.y + 3 * traj->bezier.p2.y;

		traj->bezier.precomputed = 1;
	}

	// P(t) = At^3 + Bt^2 + Ct + D
	Point p = {
		t3 * traj->bezier.ax + t2 * traj->bezier.bx + t * traj->bezier.cx + traj->bezier.p1.x,
		t3 * traj->bezier.ay + t2 * traj->bezier.by + t * traj->bezier.cy + traj->bezier.p1.y,
		0};

	return p;
}

// Calculate Bezier curve arc lenght
// https://raphlinus.github.io/curves/2018/12/28/bezier-arclength.html

void trajectoire_bezier_longueur(Trajectoire *traj)
{
	Point p;
	Point p_old = traj->bezier.p1;

	traj->longueur = 0;

	for (uint16_t i = 0; i <= BEZIER_NB_PAS; i++)
	{
		p = trajectoire_bezier_get_point(traj, (float)i / BEZIER_NB_PAS);
		traj->longueur += hypotf(p.x - p_old.x, p.y - p_old.y);
		p_old = p;
	}
}

// ==================================================================
// TRAJECTOIRE ROTATION
// ==================================================================

Point trajectoire_rotation_get_point(Trajectoire *traj, float t)
{
	return traj->rotation.pos;
}

void trajectoire_rotation_longueur(Trajectoire *traj)
{
	traj->longueur = fabs(traj->rotation.angle_debut - traj->rotation.angle_fin);
}

// ==================================================================
// TRAJECTOIRE DROITE
// ==================================================================

Point trajectoire_droite_get_point(Trajectoire *traj, float t)
{
	Point p = {
		traj->droite.p1.x * (1 - t) + traj->droite.p2.x * t,
		traj->droite.p1.y * (1 - t) + traj->droite.p2.y * t,
		0};

	return p;
}

void trajectoire_droite_longueur(Trajectoire *traj)
{
	traj->longueur = hypotf(traj->droite.p1.x - traj->droite.p2.x, traj->droite.p1.y - traj->droite.p2.y);
}

// ==================================================================
// TRAJECTOIRE CHEMIN
// ==================================================================

Point trajectoire_chemin_get_point(Trajectoire *traj, float t)
{
	float longueur = 0;
	Point p;

	if (traj->longueur == 0)
		trajectoire_chemin_longueur(traj);

	for (uint8_t i = 1; i < traj->chemin.nb_points; i++)
	{
		longueur += hypotf(
			traj->chemin.points[i].x - traj->chemin.points[i - 1].x,
			traj->chemin.points[i].y - traj->chemin.points[i - 1].y
		);

		if (longueur / traj->longueur > t)
			p = traj->chemin.points[i];
	}

	return p;
}

void trajectoire_chemin_longueur(Trajectoire *traj)
{
	traj->longueur = 0;
	for (uint8_t i = 1; i < traj->chemin.nb_points; i++)
	{
		traj->longueur += hypotf(
			traj->chemin.points[i].x - traj->chemin.points[i - 1].x,
			traj->chemin.points[i].y - traj->chemin.points[i - 1].y
		);
	}
}

// ==================================================================
// ==================================================================
