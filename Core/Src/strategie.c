#include "strategie.h"


// ######################################## //
// CONSTANTS
// ######################################## //

const Point POINT_DEPART_JAUNE = {.x = 0, .y = 1225, .theta = 0};
const Point POINT_DEPART_BLEUE = {.x = 0, .y = 1775, .theta = 0};

const Point POINT_ARRIVEE_JAUNE = {.x = 1600, .y = 375,  .theta = 0};
const Point POINT_ARRIVEE_BLEUE = {.x = 1600, .y = 2625, .theta = 0};

const Point LISTE_CANETTES_JAUNE[20] = {
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
};
const Point LISTE_CANETTES_BLEUE[20] = {
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
	{.x = 0,  .y = 0,  .theta = 0},
};

const Point ZONE_CONSTRUCTION_JAUNE = {.x = 150, .y = 775,  .theta = 0};
const Point ZONE_CONSTRUCTION_BLEUE = {.x = 150, .y = 2225, .theta = 0};

const Point ARUCO_TAGS[4] = {
	{.x = 600,  .y = 600,  .theta = 0},
	{.x = 600,  .y = 2400, .theta = 0},
	{.x = 1400, .y = 600,  .theta = 0},
	{.x = 1400, .y = 2400, .theta = 0},
};


// ==================================================================
// INITIALISATION
// ==================================================================

void init_strategie()
{

}


// ==================================================================
// GESTION DE LA STRATEGIE
// ==================================================================


void Strategie_2025(Strategie *strategie)
{
	switch (strategie->etape)
	{
	default:
		break;

	}
}


// ==================================================================
// ==================================================================
