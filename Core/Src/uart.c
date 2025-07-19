#include "uart.h"

// UART
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

// UART Receive buffers
uint8_t RX_buff_1[RX_BUFFER_SIZE] = {0};
uint8_t RX_buff_2[RX_BUFFER_SIZE] = {0};

// UART Transmit buffers
char TX_buff_1[TX_BUFFER_SIZE] = {0};
char TX_buff_2[TX_BUFFER_SIZE] = {0};

// Data received buffer and counter
static uint8_t data_recvd;
static uint8_t counter;

// Global variables
extern Target target;
extern PID pid_moteurs[4];
extern Position targets[10];
extern uint8_t nb_targets;
extern Encoder encodeurs[4];
extern Position position;
extern MPU6050 imu;
extern AltIMU altimu;
extern Robot robot;
extern PID pid_distance;
extern PID pid_theta;

// FLAG variables
extern uint8_t flag_ack;
extern uint8_t flag_uart;

// ==================================================================
// INITIALISATION
// ==================================================================

const unsigned char NBKEYS = 255;

enum
{
	HELP,
	MOVE_F,
	MOVE_B,
	MOVE_L,
	MOVE_R,
	MOVE_FL,
	MOVE_FR,
	MOVE_BL,
	MOVE_BR,
	ROTATE_CW,
	ROTATE_CCW,
	SEND_POS,
	SEND_SPEED,
	SEND_EULER,
	TRAJ_CERCLE,
	TRAJ_BEZIER,
	TRAJ_DROITE,
	SEND_SPEED_M1,
	SEND_SPEED_M2,
	SEND_SPEED_M3,
	SEND_SPEED_M4,
	SEND_SPEED_ROBOT,
	STOP_ROBOT,
	READ_PIDS,
};

char *lut[255] = {
	"HELP",
	"MOVE_F",
	"MOVE_B",
	"MOVE_L",
	"MOVE_R",
	"MOVE_FL",
	"MOVE_FR",
	"MOVE_BL",
	"MOVE_BR",
	"ROTATE_CW",
	"ROTATE_CCW",
	"SEND_POS",
	"SEND_SPEED",
	"SEND_EULER",
	"TRAJ_CERCLE",
	"TRAJ_BEZIER",
	"TRAJ_DROITE",
	"SEND_SPEED_M1",
	"SEND_SPEED_M2",
	"SEND_SPEED_M3",
	"SEND_SPEED_M4",
	"SEND_SPEED_ROBOT",
	"STOP_ROBOT",
	"READ_PIDS",
};

void init_uart()
{
	// Initialisation de la réception UART en mode interruption
	HAL_UART_Receive_IT(&huart1, &data_recvd, 1);
	HAL_UART_Receive_IT(&huart6, &data_recvd, 1);

	log_debug("OK - (UART_INIT)");
}

// ==================================================================
// UART UTLITY FUNCTIONS
// ==================================================================

void UART_send_int8(UART_HandleTypeDef *huart, const int8_t data[], uint8_t data_size)
{
	int len = 1;
	TX_buff_1[0] = '$';

	for (int i = 0; i < data_size; i++)
	{
		len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "%d;", data[i]);
		// Check if the buffer is full, avoid overflow
		if (len >= sizeof(TX_buff_1))
			break;
	}

	// Remove the last comma and replace with a semicolon
	if (len > 0)
		TX_buff_1[len - 1] = ';';

	// Add newline and carriage return
	len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "\n\r");

	// Transmit the buffer via UART
	HAL_UART_Transmit(huart, (uint8_t *)TX_buff_1, len, 10);

	// Clear TX Buffer
	memset(TX_buff_1, 0, TX_BUFFER_SIZE);
}

void UART_send_int16(UART_HandleTypeDef *huart, const int16_t data[], uint8_t data_size)
{
	int len = 1;
	TX_buff_1[0] = '$';

	for (int i = 0; i < data_size; i++)
	{
		len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "%d ", data[i]);

		// Check if the buffer is full, avoid overflow
		if (len >= sizeof(TX_buff_1))
			break;
	}

	// Remove the last comma and replace with a semicolon
	if (len > 0)
		TX_buff_1[len - 1] = ';';

	// Add newline and carriage return
	len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "\n\r");

	// Transmit the buffer via UART
	HAL_UART_Transmit(huart, (uint8_t *)TX_buff_1, len, 10);

	// Clear TX Buffer
	memset(TX_buff_1, 0, TX_BUFFER_SIZE);
}

void UART_send_float(UART_HandleTypeDef *huart, const float data[], uint8_t data_size)
{
	int len = 1;
	TX_buff_1[0] = '$';

	for (int i = 0; i < data_size; i++)
	{
		// Formatting double as float with two decimal places
		len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "%.2f;", data[i]);

		// Check if the buffer is full, avoid overflow
		if (len >= sizeof(TX_buff_1))
			break;
	}

	// Remove the last comma and replace with a semicolon
	if (len > 0)
		TX_buff_1[len - 1] = ';';

	// Add newline and carriage return
	len += snprintf(&TX_buff_1[len], sizeof(TX_buff_1) - len, "\n\r");

	// Transmit the buffer via UART
	HAL_UART_Transmit(huart, (uint8_t *)TX_buff_1, len, 10);

	// Clear TX Buffer
	memset(TX_buff_1, 0, TX_BUFFER_SIZE);
}

void UART_Print(UART_HandleTypeDef *huart, const char *str)
{
	HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 10);
	// HAL_UART_Transmit_IT(huart, (uint8_t *)str, strlen(str));
}

void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
	// Determine required buffer size
	va_list args;
	va_start(args, fmt);
	uint8_t len = vsnprintf(NULL, 0, fmt, args);
	va_end(args);
	if (len < 0)
		return;

	// Format message
	char msg[len + 1];
	va_start(args, fmt);
	vsnprintf(msg, len + 1, fmt, args);
	va_end(args);

	HAL_UART_Transmit(huart, (uint8_t *)msg, sizeof(msg), 10);
	// HAL_UART_Transmit_IT(huart, (uint8_t *)msg, sizeof(msg));
}

// ==================================================================
// UART WRAPPERS
// ==================================================================

void UART1_Debug(const char *str)
{
	UART_Print(&huart1, str);
}
void UART6_Debug(const char *str)
{
	UART_Print(&huart6, str);
}
void UART1_send_float(const float data[], uint8_t data_size)
{
	UART_send_float(&huart1, data, data_size);
}
void UART1_send_int16(const int16_t data[], uint8_t data_size)
{
	UART_send_int16(&huart1, data, data_size);
}
void UART1_send_int8(const int8_t data[], uint8_t data_size)
{
	UART_send_int8(&huart1, data, data_size);
}
void UART6_send_float(const float data[], uint8_t data_size)
{
	UART_send_float(&huart6, data, data_size);
}
void UART6_send_int16(const int16_t data[], uint8_t data_size)
{
	UART_send_int16(&huart6, data, data_size);
}
void UART6_send_int8(const int8_t data[], uint8_t data_size)
{
	UART_send_int8(&huart6, data, data_size);
}

// ==================================================================
// UART RECEIVE AND TRANSMIT CALLBACK FUNCTIONS
// ==================================================================

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// Clear buffer when transmission is complete
	if (huart->Instance == USART1)
	{
		memset(TX_buff_1, 0, strlen(TX_buff_1));
	}
	else if (huart->Instance == USART6)
	{
		memset(TX_buff_2, 0, strlen(TX_buff_2));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (data_recvd == '\r')
		{
			RX_buff_1[counter++] = '\0';
			counter = 0;
			flag_uart = 1; // Set flag to indicate data received
			//UART_routine(RX_buff_1);
		}
		else
		{
			RX_buff_1[counter++] = data_recvd;
		}

		HAL_UART_Receive_IT(&huart1, &data_recvd, 1);
	}
	else if (huart->Instance == USART6)
	{
		if (data_recvd == '\r')
		{
			RX_buff_2[counter++] = '\0';
			counter = 0;
			flag_uart = 2; // Set flag to indicate data received
			//UART_routine(RX_buff_2);
		}
		else
		{
			RX_buff_2[counter++] = data_recvd;
		}

		HAL_UART_Receive_IT(&huart6, &data_recvd, 1);
	}
}

// ==================================================================
// UART RECEIVE ROUTINES
// ==================================================================

void UART_routine(uint8_t *data)
{
	log_debug("<-- 0x%x", *data);

	// Do something with the data
	switch (data[0])
	{

	///////////////////////////////////////////
	// MOVE TO X, Y, THETA
	///////////////////////////////////////////

	case 0x01:
		log_info("New position received");
		if (target.state != TARGET_REACHING)
		{
			Move_To(&target,
					(float)(int16_t)((data[1] << 8) + data[2]),
					(float)(int16_t)((data[3] << 8) + data[4]),
					(float)(int16_t)((data[5] << 8) + data[6]) * DEG2RAD,
					VMAX_LIN, // Default
					AMAX_LIN  // Default
			);
			log_info("New target set: (%.1f, %.1f, %1.f)", target.x, target.y, target.theta);
		}
		else
		{
			/*
			if (nb_targets >= 1)
			{
				nb_targets--;
				targets[nb_targets].x = (float)((data[1] << 8) + data[2]);
				targets[nb_targets].y = (float)((data[3] << 8) + data[4]);
				targets[nb_targets].theta = (float)((data[5] << 8) + data[6]) * DEG2RAD;
				log_info("New target stored: (%.1f, %.1f, %1.f)", target.x, target.y, target.theta);
			}
			*/
		}
		break;

	///////////////////////////////////////////
	// MOVE TO X, Y, THETA, SPEED
	///////////////////////////////////////////

	case 0x02:
		log_info("New position with speed received");
		if (target.state != TARGET_REACHING)
		{
			Move_To(&target,
					(float)(int16_t)((data[1] << 8) + data[2]),
					(float)(int16_t)((data[3] << 8) + data[4]),
					(float)(int16_t)((data[5] << 8) + data[6]) * DEG2RAD,
					(float)(int16_t)((data[7] << 8) + data[8]),
					AMAX_LIN // Default
			);
			log_info("New target set: (%.1f, %.1f, %1.f)", target.x, target.y, target.theta);
		}
		else
		{
			/*
			if (nb_targets >= 1)
			{
				nb_targets--;
				targets[nb_targets].x = (float)((data[1] << 8) + data[2]);
				targets[nb_targets].y = (float)((data[3] << 8) + data[4]);
				targets[nb_targets].theta = (float)((data[5] << 8) + data[6]) * DEG2RAD;
				log_info("New target stored: (%.1f, %.1f, %1.f)", target.x, target.y, target.theta);
			}
			*/
		}
		break;

	///////////////////////////////////////////
	// MOVE OF X, Y, THETA
	///////////////////////////////////////////

	case 0x03:
		log_info("New movement received");
		if (target.state != TARGET_REACHING)
		{
			Move_Of(&target,
					(float)(int16_t)((data[1] << 8) + data[2]),
					(float)(int16_t)((data[3] << 8) + data[4]),
					(float)(int16_t)((data[5] << 8) + data[6]) * DEG2RAD,
					VMAX_LIN, // Default
					AMAX_LIN  // Default
			);
			log_info("New target set: (%.1f, %.1f, %1.f)", target.x, target.y, target.theta);
		}
		break;

	///////////////////////////////////////////
	///////////////////////////////////////////

	case 0x04:
		break;

	///////////////////////////////////////////
	// ACK (PING)
	///////////////////////////////////////////

	case 0x05:
		log_info("ACK - Ping");
		flag_ack = 1;
		break;

	///////////////////////////////////////////
	// SEND POSITION & ORIENTATION
	///////////////////////////////////////////

	case 0x06:
		uint8_t tmp_buffer[7] = {
			0x06,
			(int16_t)(position.x) >> 8,
			(int16_t)(position.x) & 0xFF,
			(int16_t)(position.y) >> 8,
			(int16_t)(position.y) & 0xFF,
			(int16_t)(position.theta) >> 8,
			(int16_t)(position.theta) && 0xFF,
		};

		HAL_UART_Transmit(&huart1, tmp_buffer, 7, 100);
		break;

	///////////////////////////////////////////
	// SET ROBOT'S CURRENT POSITION
	///////////////////////////////////////////

	case 0x07:
		odometry_set_position(
			&position,
			(float)(int16_t)((data[1] << 8) + data[2]),
			(float)(int16_t)((data[3] << 8) + data[4]),
			(float)(int16_t)((data[5] << 8) + data[6]) * DEG2RAD
		);
		flag_ack = 1;
		break;

	///////////////////////////////////////////
	// REMOVE TARGET POINTS
	///////////////////////////////////////////

	case 0xF0:
		log_info("Removed all target points stored !");
		memset(targets, 0, 10 * sizeof(targets[0]));
		flag_ack = 1;
		break;

	///////////////////////////////////////////
	// RESUME
	///////////////////////////////////////////

	case 0xF2:
		log_info("Resume after STOP");
		target.state = target.previous_state;
		break;

	case 0x2F:
		log_info("Resume after STOP");
		target.state = target.previous_state;
		break;

	///////////////////////////////////////////
	// ROBOT STOP
	///////////////////////////////////////////

	case 0x0F:
		log_fatal("EMERGENCY STOP (LiDAR)");
		Target_Stop(&target);
		flag_ack = 1;
		break;

	///////////////////////////////////////////
	// RESET
	///////////////////////////////////////////

	case 0xFF:
		log_fatal("Reset received !!");

		// Reset pids
		reset_pid(&pid_moteurs[0]);
		reset_pid(&pid_moteurs[1]);
		reset_pid(&pid_moteurs[2]);
		reset_pid(&pid_moteurs[3]);
		reset_pid(&pid_theta);
		reset_pid(&pid_distance);
		log_debug("Reset PID -> Done");

		// Reset current position
		reset_position();
		log_debug("Reset POS -> Done");

		// Reset current target
		reset_target(&target);
		log_debug("Reset TARGET -> Done");

		// ACK
		flag_ack = 1;

		break;

	///////////////////////////////////////////
	// OTHER IS DEBUG
	///////////////////////////////////////////

	default:
		Routine_Test(data);
		break;
	}

	// Reset flag after processing
	flag_uart = 0;
	// Then clear the buffer
	memset(data, 0, RX_BUFFER_SIZE);
}

// ==================================================================
// ROUTINE COMMUNICATION
// ==================================================================

void Routine_Test(uint8_t *data)
{
	// Echo received command
	log_debug("<-- %s", (char *)data);

	// Special case treatment
	if (strcmp((char *)data, "ping") == 0)
	{
		log_info("pong");
		return;
	}
	else if (strcmp((char *)data, "test") == 0)
	{
		log_trace("PI = %.3f", 3.1415);
		log_error("PI = %.3f", 3.1415);
		log_debug("PI = %.3f", 3.1415);
		log_fatal("PI = %.3f", 3.1415);
		log_warn("PI = %.3f", 3.1415);
		log_info("PI = %.3f", 3.1415);
		return;
	}

	switch (keyfromstring((char *)data))
	{
	//////////////////////////////////////////
	case HELP:
		log_info("##################################");
		log_info("# HELP");
		log_info("##################################");
		for (uint8_t i = 0; i < NBKEYS; i++)
			if (snprintf(NULL, 0, "%s", lut[i]) > 0)
				log_info("%d - %s", i, lut[i]);
		log_info("##################################");
		log_info("##################################");
		break;
	//////////////////////////////////////////
	case MOVE_F:
		log_debug("MOVE_FORWARD");
		move_f(127);
		break;
	//////////////////////////////////////////
	case MOVE_B:
		log_debug("MOVE_BACKWARD");
		move_b(127);
		break;
	//////////////////////////////////////////
	case MOVE_L:
		log_debug("MOVE_LEFT");
		move_l(127);
		break;
	//////////////////////////////////////////
	case MOVE_R:
		log_debug("MOVE_RIGHT");
		move_r(127);
		break;
	//////////////////////////////////////////
	case MOVE_FL:
		log_debug("MOVE_FORWARD_LEFT");
		move_fl(127);
		break;
	//////////////////////////////////////////
	case MOVE_FR:
		log_debug("MOVE_FORWARD_RIGHT");
		move_fr(127);
		break;
	//////////////////////////////////////////
	case MOVE_BL:
		log_debug("MOVE_BACKWARD_LEFT");
		move_bl(127);
		break;
	//////////////////////////////////////////
	case MOVE_BR:
		log_debug("MOVE_BACKWARD_RIGHT");
		move_br(127);
		break;
	//////////////////////////////////////////
	case ROTATE_CW:
		log_debug("ROTATE_CW");
		rotate_cw(127);
		break;
	//////////////////////////////////////////
	case ROTATE_CCW:
		log_debug("ROTATE_CCW");
		rotate_ccw(127);
		break;
	//////////////////////////////////////////
	case SEND_POS:
		log_info("x = %.1f, y = %.1f, theta = %.1f)", position.x, position.y, position.theta);
		break;
	//////////////////////////////////////////
	case SEND_SPEED:
		log_info(
			"v1 = %.1f, v2 = %.1f, v3 = %.1f, v4 = %.1f)",
			encodeurs[0].vitesse_mms,
			encodeurs[1].vitesse_mms,
			encodeurs[2].vitesse_mms,
			encodeurs[3].vitesse_mms);
		break;
	//////////////////////////////////////////
	case SEND_EULER:
		break;
	//////////////////////////////////////////
	case TRAJ_CERCLE:
		break;
	//////////////////////////////////////////
	case TRAJ_BEZIER:
		break;
	//////////////////////////////////////////
	case TRAJ_DROITE:
		break;
	//////////////////////////////////////////
	case SEND_SPEED_M1:
		log_info("(M1) - v1 = %.1f mm/s", encodeurs[0].vitesse_mms);
		break;
	//////////////////////////////////////////
	case SEND_SPEED_M2:
		log_info("(M2) - v2 = %.1f mm/s", encodeurs[1].vitesse_mms);
		break;
	//////////////////////////////////////////
	case SEND_SPEED_M3:
		log_info("(M3) - v3 = %.1f mm/s", encodeurs[2].vitesse_mms);
		break;
	//////////////////////////////////////////
	case SEND_SPEED_M4:
		log_info("(M4) - v4 = %.1f mm/s", encodeurs[3].vitesse_mms);
		break;
	//////////////////////////////////////////
	case SEND_SPEED_ROBOT:
		log_info("v_robot = %.1f mm/s", robot.speed);
		break;
	//////////////////////////////////////////
	case STOP_ROBOT:
		log_info("Stopped motors !");
		stop();
		break;
	//////////////////////////////////////////
	case READ_PIDS:
		log_info("(M1) - Kp = %.2f, Ki = %.2f, Kd = %.2f", pid_moteurs[0].Kp, pid_moteurs[0].Ki, pid_moteurs[0].Kd);
		log_info("(M2) - Kp = %.2f, Ki = %.2f, Kd = %.2f", pid_moteurs[1].Kp, pid_moteurs[1].Ki, pid_moteurs[1].Kd);
		log_info("(M3) - Kp = %.2f, Ki = %.2f, Kd = %.2f", pid_moteurs[2].Kp, pid_moteurs[2].Ki, pid_moteurs[2].Kd);
		log_info("(M4) - Kp = %.2f, Ki = %.2f, Kd = %.2f", pid_moteurs[3].Kp, pid_moteurs[3].Ki, pid_moteurs[3].Kd);
		break;
	//////////////////////////////////////////
	default:
		log_error("Wrong command !");
		break;
	}
}

// ==================================================================
// UTILITY FUNCTIONS
// ==================================================================

uint8_t keyfromstring(const char *str)
{
	uint8_t i;
	for (i = 0; i < NBKEYS; i++)
		if (strcmp(str, lut[i]) == 0)
			break;
	return i;
}

// ==================================================================
// ==================================================================
