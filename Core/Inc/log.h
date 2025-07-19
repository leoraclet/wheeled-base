#ifndef INC_LOG_H_
#define INC_LOG_H_


#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "uart.h"
#include "defines.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern RTC_HandleTypeDef hrtc;


enum {
	LOG_TRACE,
	LOG_DEBUG,
	LOG_INFO,
	LOG_WARN,
	LOG_ERROR,
	LOG_FATAL
};


#if defined(DEBUG_LOG)
#if defined(UART1)

#define log_trace(...) log_log(&huart1, LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(&huart1, LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(&huart1, LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(&huart1, LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(&huart1, LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(&huart1, LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

#elif defined(UART6)

#define log_trace(...) log_log(&huart6, LOG_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(&huart6, LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(&huart6, LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(&huart6, LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(&huart6, LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(&huart6, LOG_FATAL, __FILE__, __LINE__, __VA_ARGS__)

#endif
#else

#define log_trace(...) do_nothing()
#define log_debug(...) do_nothing()
#define log_info(...)  do_nothing()
#define log_warn(...)  do_nothing()
#define log_error(...) do_nothing()
#define log_fatal(...) do_nothing()

#endif


void do_nothing();
void log_log(UART_HandleTypeDef *huart, uint8_t level, const char *file, int line, const char *fmt, ...);


#endif /* INC_LOG_H_ */
