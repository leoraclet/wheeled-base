#include "log.h"


static const char *level_strings[6] = {
	"TRACE",
	"DEBUG",
	"INFO",
	"WARN",
	"ERROR",
	"FATAL"
};

static const char *level_colors[6] = {
	"\033[94m",
	"\033[36m",
	"\033[32m",
	"\033[33m",
	"\033[31m",
	"\033[35m"
};


void log_log(UART_HandleTypeDef *huart, uint8_t level, const char *file, int line, const char *fmt, ...)
{
	/*
	RTC_TimeTypeDef currentTime;
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	struct tm currTime;
	currTime.tm_hour = currentTime.Hours;
	currTime.tm_min = currentTime.Minutes;
	currTime.tm_sec = currentTime.Seconds;
	*/

	uint32_t seconds = HAL_GetTick() / 1000;

	struct tm currTime;
	currTime.tm_min = seconds / 60;
	currTime.tm_sec = seconds % 60;

	// Format time
	char buf[16];
	buf[strftime(buf, sizeof(buf), "%M:%S", &currTime)] = '\0';

	#ifdef LOG_COLORS
	// Format message prefix
	uint16_t len = snprintf(NULL, 0, "[%s] %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ", buf, level_colors[level], level_strings[level], file, line);
	char prefix_msg[len + 1];
	snprintf(prefix_msg, len + 1, "[%s] %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ", buf, level_colors[level], level_strings[level], file, line);

	#else
	// Format message prefix
	uint16_t len = snprintf(NULL, 0, "[%s] %-5s %s:%d: ", buf, level_strings[level], file, line);
	char prefix_msg[len + 1];
	snprintf(prefix_msg, len + 1, "[%s] %-5s %s:%d: ", buf, level_strings[level], file, line);

	#endif

	// Format message actual information
	va_list args;
	va_start(args, fmt);
	len = vsnprintf(NULL, 0, fmt, args);
	va_end(args);

	char suffix_msg[len + 1];
	va_start(args, fmt);
	vsnprintf(suffix_msg, len + 1, fmt, args);
	va_end(args);

	// Format complete message
	len = snprintf(NULL, 0, "%s%s\r\n", prefix_msg, suffix_msg);
	char msg[len + 1];
	snprintf(msg, len + 1, "%s%s\r\n", prefix_msg, suffix_msg);

	// Transmit message
	HAL_UART_Transmit(huart, (uint8_t *)msg, sizeof(msg), 10);
}

void do_nothing()
{

}
