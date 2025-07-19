#include "temps.h"


// Timer
extern TIM_HandleTypeDef htim5;


// ==================================================================
// INITIALISATION
// ==================================================================

void Init_SysTick()
{
	// Configure SysTick with a 1 MHz frequency
	HAL_NVIC_DisableIRQ(SysTick_IRQn);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 100000);
	HAL_NVIC_EnableIRQ(SysTick_IRQn);
}

// ==================================================================
// GESTION DU TEMPS
// ==================================================================

void Temps_Delay_us(uint32_t delay_us)
{
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
	if (delay_us < 1000)
		while (__HAL_TIM_GET_COUNTER(&htim5) - start < delay_us);
	else
		HAL_Delay(delay_us / 1000);
}

uint32_t Get_Temps_us()
{
	return __HAL_TIM_GET_COUNTER(&htim5) * 10;
}

// ==================================================================
// ==================================================================
