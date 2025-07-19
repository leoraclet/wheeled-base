#ifndef SRC_UART_H_
#define SRC_UART_H_

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "imu.h"
#include "log.h"
#include "altimu.h"
#include "defines.h"
#include "odometrie.h"
#include "moteurs.h"
#include "stm32f4xx_hal_gpio.h"


// Variadic functions wrappers
#define UART1_Printf(...) UART_Printf(&huart1, __VA_ARGS__)
#define UART6_Printf(...) UART_Printf(&huart6, __VA_ARGS__)


void init_uart();

void UART_routine(uint8_t *data);
void UART_send_float(UART_HandleTypeDef *huart, const float data[], uint8_t data_size);
void UART_send_int16(UART_HandleTypeDef *huart, const int16_t data[], uint8_t data_size);
void UART_send_int8(UART_HandleTypeDef *huart, const int8_t data[], uint8_t data_size);
void UART_Print(UART_HandleTypeDef *huart, const char *str);
void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);

void UART1_Debug(const char* str);
void UART6_Debug(const char* str);
void UART1_send_float(const float data[], uint8_t data_size);
void UART1_send_int16(const int16_t data[], uint8_t data_size);
void UART1_send_int8(const int8_t data[], uint8_t data_size);
void UART6_send_float(const float data[], uint8_t data_size);
void UART6_send_int16(const int16_t data[], uint8_t data_size);
void UART6_send_int8(const int8_t data[], uint8_t data_size);

void Routine_Test(uint8_t *data);
uint8_t keyfromstring(const char *str);

#endif /* SRC_UART_H_ */
