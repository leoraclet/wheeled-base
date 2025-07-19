#include "encoders.h"


// ==================================================================
// INITIALISATION
// ==================================================================

void init_encoders(Encoder encodeurs[4])
{
	// Initialisation des encodeurs
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	encodeurs[0].valeur_precedente = 0;
	encodeurs[1].valeur_precedente = 0;
	encodeurs[2].valeur_precedente = 0;
	encodeurs[3].valeur_precedente = 0;
}

// ==================================================================
// ENCODERS UTILITY FUNCTION
// ==================================================================

void update_encoder(Encoder *encodeur, TIM_HandleTypeDef *htim)
{
    // Lire une seule fois la valeur du compteur et l'autoreload
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(htim);

    // Gestion du cas où le compteur n'a pas changé
    if (temp_counter == encodeur->valeur_precedente)
    {
        encodeur->delta_tick = 0;
        encodeur->vitesse_mms = 0;
    }
    else
    {
        // Vérifier si le timer est en mode décompte ou incrément
    	uint8_t counting_down = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);

        if (temp_counter > encodeur->valeur_precedente)
        {
        	// En mode décompte avec dépassement
            if (counting_down) {
            	encodeur->delta_tick = -encodeur->valeur_precedente - (auto_reload - temp_counter);
			// En mode incrémentation simple
            } else {
                encodeur->delta_tick = temp_counter - encodeur->valeur_precedente;
            }
        }
        else
        {
        	// En mode décompte avec sous-passement
            if (counting_down) {
            	encodeur->delta_tick = temp_counter - encodeur->valeur_precedente;
			// En mode incrémentation avec sous-passement
            } else {
            	encodeur->delta_tick = temp_counter + (auto_reload - encodeur->valeur_precedente);
            }
        }

        encodeur->valeur_precedente = temp_counter;
        encodeur->vitesse_mms = encodeur->delta_tick * TICK_TO_MM_S;
    }
}

// ==================================================================
// ==================================================================

