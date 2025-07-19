#ifndef SRC_ENCODERS_H_
#define SRC_ENCODERS_H_

#include <stdint.h>

#include "main.h"
#include "defines.h"
#include "stm32f4xx_hal_gpio.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


typedef struct {
    uint16_t valeur_precedente;
    int16_t delta_tick;
    int16_t vitesse_pwm;
    float vitesse_mms;
} Encoder;


void update_encoder(Encoder *encodeur, TIM_HandleTypeDef *htim);
void init_encoders(Encoder encodeurs[4]);


#endif /* SRC_ENCODERS_H_ */
