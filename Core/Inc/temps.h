#ifndef INC_TEMPS_H_
#define INC_TEMPS_H_


#include "main.h"


void Init_SysTick();
void temps_wait_us(uint32_t time_us);
uint32_t Get_Temps_us();


#endif /* INC_TEMPS_H_ */
