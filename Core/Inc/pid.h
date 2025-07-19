#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "main.h"
#include "defines.h"
#include "stm32f4xx_hal_gpio.h"


typedef struct {
	float Ki;
	float Kp;
	float Kd;
	float Kc;
	float Ep;
	float Ei;
	float Ed;
	float output;
	float error;
	float maxIntegral;
	float maxOutput;
	uint8_t mode;
	uint16_t sampleTime;
} PID;


void init_pid(PID *pid, float Kp, float Ki, float Kd, float Kc);
void set_tunnings(PID *pid, float Kp, float Ki, float Kd);
void set_sample_time(PID *pid, uint16_t sample_time);
void set_pid_mode(PID *pid, uint8_t set_mode, float output, float input);
void compute_pid(PID *pid, float consigne, float mesure, float outputLimit);
void reset_pid(PID *pid);


#endif /* SRC_PID_H_ */
