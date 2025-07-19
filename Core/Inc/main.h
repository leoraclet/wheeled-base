/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Encodeur_M1_VA_Pin GPIO_PIN_0
#define Encodeur_M1_VA_GPIO_Port GPIOA
#define Encodeur_M1_VB_Pin GPIO_PIN_1
#define Encodeur_M1_VB_GPIO_Port GPIOA
#define PWM_M1_Pin GPIO_PIN_2
#define PWM_M1_GPIO_Port GPIOA
#define PWM_M2_Pin GPIO_PIN_3
#define PWM_M2_GPIO_Port GPIOA
#define Input_M1_1_Pin GPIO_PIN_4
#define Input_M1_1_GPIO_Port GPIOA
#define Input_M1_2_Pin GPIO_PIN_5
#define Input_M1_2_GPIO_Port GPIOA
#define Encodeur_M2_VA_Pin GPIO_PIN_6
#define Encodeur_M2_VA_GPIO_Port GPIOA
#define Encodeur_M2_VB_Pin GPIO_PIN_7
#define Encodeur_M2_VB_GPIO_Port GPIOA
#define Input_M2_1_Pin GPIO_PIN_0
#define Input_M2_1_GPIO_Port GPIOB
#define Input_M2_2_Pin GPIO_PIN_1
#define Input_M2_2_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define Input_M3_1_Pin GPIO_PIN_12
#define Input_M3_1_GPIO_Port GPIOB
#define Input_M3_2_Pin GPIO_PIN_13
#define Input_M3_2_GPIO_Port GPIOB
#define Input_M4_1_Pin GPIO_PIN_14
#define Input_M4_1_GPIO_Port GPIOB
#define Input_M4_2_Pin GPIO_PIN_15
#define Input_M4_2_GPIO_Port GPIOB
#define Encodeur_M3_VA_Pin GPIO_PIN_8
#define Encodeur_M3_VA_GPIO_Port GPIOA
#define Encodeur_M3_VB_Pin GPIO_PIN_9
#define Encodeur_M3_VB_GPIO_Port GPIOA
#define UART1_RX_Pin GPIO_PIN_10
#define UART1_RX_GPIO_Port GPIOA
#define UART6_TX_Pin GPIO_PIN_11
#define UART6_TX_GPIO_Port GPIOA
#define UART6_RX_Pin GPIO_PIN_12
#define UART6_RX_GPIO_Port GPIOA
#define UART1_TX_Pin GPIO_PIN_15
#define UART1_TX_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_3
#define I2C_SDA_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOB
#define Encodeur_M4_VA_Pin GPIO_PIN_6
#define Encodeur_M4_VA_GPIO_Port GPIOB
#define Encodeur_M4_VB_Pin GPIO_PIN_7
#define Encodeur_M4_VB_GPIO_Port GPIOB
#define PWM_M3_Pin GPIO_PIN_8
#define PWM_M3_GPIO_Port GPIOB
#define PWM_M4_Pin GPIO_PIN_9
#define PWM_M4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
