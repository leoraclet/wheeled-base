#include "mainprog.h"


#define IMU_ACTIVE   0
#define TEST_ACTIVE  0

// ==================================================================
// ==================================================================

// Timer
extern TIM_HandleTypeDef htim5;
// UART
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

extern uint8_t RX_buff_1[RX_BUFFER_SIZE];
extern uint8_t RX_buff_2[RX_BUFFER_SIZE];

// ==================================================================
// ==================================================================

// Structs instantiations
Robot robot;						  // Robot's current state
Position position;                    // Robot's position
Encoder encodeurs[4];                 // Motor rotatory encoders
Target target;                        // Target to reach
PID pid_moteurs[4];                   // PID motors speed
PID pid_distance;                     // PID distance
PID pid_theta;                        // PID angle
Point targets[10];                    // Target points for complex movement
MPU6050 imu;                          // MPU-6050
AltIMU altimu;                        // AltIMU-v4


volatile uint8_t flag_interrupt = 0;  // FLAG timer interrupt
volatile uint8_t flag_ack = 0;        // FLAG send ACK
volatile uint8_t flag_uart = 0;       // FLAG uart received

uint8_t int_counter = 0;              // Interrupts counter
uint32_t time_cnt = 0;                // Time elapsed
uint8_t nb_targets = 10;              // Number of positions un buffer
uint8_t times_up = 0;                 // Timer to 10 seconds


// ==================================================================
// TIMER CALLBACKS
// ==================================================================

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Interuption toutes les DELTA_T ms
    if (htim->Instance == TIM5)
    {
    	flag_interrupt = 1;

    	////////////////////////////////////
    	// TEST
    	////////////////////////////////////

    	if (TEST_ACTIVE)
    	{
    		localisation_update(&position, encodeurs);
    	}

    	////////////////////////////////////
    	////////////////////////////////////
    }
}


// ==================================================================
// REDIRECTS printf() to UART for easier Debugging
// ==================================================================

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 10);

	return ch;
}

// ==================================================================
// INITIALISATION
// ==================================================================

void init()
{
	// Initialisation de l'IMU
	if (IMU_ACTIVE)
	{
		init_mpu();
		init_altimu();
	}

	// Initialisation des moteurs
	init_moteurs();
	log_debug("OK - (MOTORS_INIT)");

	// Initialisation des encodeurs
	init_encoders(encodeurs);
	log_debug("OK - (ENCODERS_INIT)");

	// Initialisation de la target
	init_target(&target);

	// Initialise target position and heading (theta) in mm
	if (TEST_ACTIVE)
		set_target_pos(&target, 2000, 0, 0, true);

	// Initialisation de la réception UART
	init_uart();

	// Initialisation du robot
	init_robot(&robot);
	log_debug("OK - (ROBOT_INIT)");

	// Initialisation des PIDs
	init_pid(&pid_moteurs[0], 1, 0.15, 0.4, 0);
	init_pid(&pid_moteurs[1], 1, 0.15, 0.4, 0);
	init_pid(&pid_moteurs[2], 1, 0.15, 0.4, 0);
	init_pid(&pid_moteurs[3], 1, 0.15, 0.4, 0);
	init_pid(&pid_distance, 2, 0, 0, 0);
	init_pid(&pid_theta, 4, 0, 0, 0);
	log_debug("OK - (PIDS_INIT)");

	// Initialisation de l'interruption sur l'horloge
	// Permet de mesurer le temps avec précision et d'appeler une routine
	// d'interruption à interval régulier
	HAL_TIM_Base_Start_IT(&htim5);
	log_debug("OK - (TIM5_INIT)");

	// Log
	log_debug("OK - (INIT_ALL)");
}

// ==================================================================
// MAIN FUNCTION
// ==================================================================

void mainprog()
{
	// TODO: Gestion du trajet
	// TODO: Extended Kalman Filter (Sensor Fusion)
	// TODO: eCompass using IMU

	// Variables
	static uint32_t start = 0;
	static uint32_t hal_tick = 0;

	log_info("Main programm started");

	// LED on
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	log_info("Interrupt HIGH");

	while(1)
	{
		////////////////////////////////////
		// LOGGING DU TEMPS
		////////////////////////////////////

		hal_tick = HAL_GetTick();
		if (hal_tick < 100000 && (hal_tick - start >= 10000  || (hal_tick - start >= 1000 && hal_tick > 85000)))
		{
			log_warn("%d seconds remaining !", 100 - (hal_tick / 1000));
			start = hal_tick;
		}
		else if (hal_tick > 100000 && !times_up)
		{
			log_fatal("Times up !!!");
			times_up = 1;
		}

		////////////////////////////////////
		// UART
		////////////////////////////////////

		if (flag_uart != 0)
		{
			if (flag_uart == 1)
			{
				UART_routine(RX_buff_1);
				flag_uart = 0;
			}
			else if (flag_uart == 2)
			{
				UART_routine(RX_buff_2);
				flag_uart = 0;
			}
		}

		////////////////////////////////////
		// GESTION DE L'ACK
		////////////////////////////////////

		if (flag_ack == 1)
		{
			flag_ack = 0;
			send_ACK();

			if (target.state == TARGET_REACHED)
				log_info("Sent ACK for finished movement");
			if (target.state == TARGET_EMERGENCY_STOP)
				log_info("Sent ACK for successfull EMERGENCY STOP");
		}

		////////////////////////////////////
		// GESTION DE L'INTERRUPTION
		////////////////////////////////////

		if (flag_interrupt == 0)
			continue;

		flag_interrupt = 0;

		// CALCULE DE L'ODOMETRIE
		localisation_update(&position, encodeurs);

		// RAMPE DE VITESSE
		rampe_vitesse_ang(&position, &robot, DELTA_T);
		rampe_vitesse_lin(&position, &robot, DELTA_T);

		// ASSERVISSEMENT EN POSITION DU ROBOT
		Asserv_Position(&robot, &target, &position, &pid_distance, &pid_theta, pid_moteurs, encodeurs);
	}
}

// =========================================================================================================
// =========================================================================================================
