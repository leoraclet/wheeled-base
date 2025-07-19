#ifndef INC_STRATEGIE_H_
#define INC_STRATEGIE_H_


#include "main.h"
#include "trajectoires.h"


enum Etat_Robot {
	ETAT_ATTENTE,
	ETAT_ARRET,
	ETAT_DEPLACEMENT,
	ETAT_ROTATION,
};

enum Etape_Strategie {
	ETAPE_DEBUT,
	ETAPE_FIN,
	ETAPE_EVITEMENT,
	ETAPE_CALAGE,
	ETAPE_CONSTRUCTION,
	ETAPE_RECHARGE,
	ETAPE_PLANCHE_PRISE,
	ETAPE_PLANCHE_LACHE,
	ETAPE_CANETTE_PRISE,
	ETAPE_CANETTE_LACHE,
	ETAPE_BANNIERE,
	ETATE_SEPARATION_PLANCHES,

};

typedef struct {
	enum Etat_Robot etat_robot;
	enum Etape_Strategie etape;
	uint32_t temps_ecoule;
} Strategie;


void init_strategie();


#endif /* INC_STRATEGIE_H_ */
