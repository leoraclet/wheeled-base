#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_


#include <math.h>


// ==================================================================
// UART
// ==================================================================

#define RX_BUFFER_SIZE  50
#define TX_BUFFER_SIZE  50
#define UART1           1
#define LOG_COLORS      1
//#define DEBUG_LOG       1

// ==================================================================
// ENCODERS
// ==================================================================

#define DELTA_T                   0.005 // Time between each interruption
#define RAYON_ROUE                0.03  // Wheel radius
#define RAYON_ROUE_MM             30    // wheel raidus (in mm)
#define DISTANCE_ROUES_CENTRE     0.185 // Distance between wheels and center of robot's frame
#define RESOLUTION_CODEUSES       3200  // Ticks per revolution on motor's rotatory encoders

#define DIFF_DELTA_T              (0.01 / DELTA_T) // 10 ms was default for PWM conversion and PID tuning
#define INV_RAYON_ROUE            (1 / RAYON_ROUE)
#define PERIMETRE_ROUE_MM         (M_TWOPI * RAYON_ROUE_MM)
#define PERIMETRE_ROUE_M          (PERIMETRE_ROUE_MM / 1000)
#define TICK_TO_RAD               (M_TWOPI / RESOLUTION_CODEUSES)
#define TICK_TO_MM				  (PERIMETRE_ROUE_MM / RESOLUTION_CODEUSES)
#define TICK_TO_M                 (PERIMETRE_ROUE_M / RESOLUTION_CODEUSES)
#define TICK_TO_M_S               (TICK_TO_M / DELTA_T)
#define TICK_TO_MM_S              (TICK_TO_MM / DELTA_T)

// ==================================================================
// MOTORS
// ==================================================================

#define STOP              0
#define SENS_AVANT        1
#define SENS_ARRIERE      2
#define FAST_STOP_ENABLE  0

// ==================================================================
// RAMPE
// ==================================================================

#define VMAX_PWM   200.0   // Minimal motor speed (PWM)
#define VMIN_PWM   0       // Minimal motor speed (PWM)
#define AMAX_LIN   200.0   // Maximal linear acceleration
#define VMAX_LIN   200.0   // Maximal linear speed
#define AMAX_ANG   2.0     // Maximal angular acceleration
#define VMAX_ANG   M_PI_2  // Maximal angular speed

// ==================================================================
// PID
// ==================================================================

#define QSM_EPSILON          0.001
#define PID_MANUAL           0
#define PID_AUTO             1
#define MAX_INTEGRAL_ERROR   VMAX_PWM
#define MAX_OUTPUT           VMAX_PWM

// ==================================================================
// MPU-6050
// ==================================================================

#define MPU6050_ADDR      0xD0
#define ACCEL_CONFIG_REG  0x1C
#define GYRO_CONFIG_REG   0x1B

// ==================================================================
// AltIMU-10 v4
// ==================================================================

#define L3GD20H_ADDR  0b11010110
#define LSM303D_ADDR  0b00111010
#define LPS25H_ADDR   0b10111010

// ==================================================================
// TRAJECTOIRES
// ==================================================================

#define BEZIER_NB_PAS       500
#define PRECISION_ABSCISSE  0.001
#define NB_POINTS_MAX       20

// ==================================================================
// TRAJET
// ==================================================================

#define TRAJECT_CONFIG_AVANCE_DROIT      1000, 500  // Vitesse et acceleration pour translation pure (en mm/s et mm/s²)
#define TRAJECT_CONFIG_AVANCE_ET_TOURNE  300, 500   // Vitesse et acceleration pour un mouvement complexe (en mm et mm/s²)
#define TRAJECT_CONFIG_STD               500, 500   // Vitesse et acceleration - standard (en mm et mm/s²)
#define TRAJECT_CONFIG_ROTATION_PURE     2, 2       // Vitesse et acceleration pour une rotation (rad/s et rad/s²)
#define DISTANCE_INVALIDE                -1

// ==================================================================
// MATH
// ==================================================================

#define DEGRE2RADIAN  (M_PI / 180.)
#define RADIAN2DEGRE  (1. / DEGRE2RADIAN)
#define DEG2RAD       (DEGRE2RADIAN)
#define RAD2DEG       (RADIAN2DEGRE)

// ==================================================================
// ODOMETRIE
// ==================================================================

//#define APPROXIMATION_AVEC_ARC  1
//#define APPROXIMATION_SANS_ARC  1

// ==================================================================
// STRATEGIE
// ==================================================================

#define TEMPS_MATCH_S   (100)
#define TEMPS_MATCH_MS  (100000)
#define TEMPS_PAMI_S    (85)
#define TEMPS_PAMI_MS   (85000)

// ==================================================================
// SOMETHING ...
// ==================================================================

#define TARGET_CORRECTION  (200.0 / 187.0)
#define OPTIMISATION       (0)
#define DEBBUGING          (1)

// ==================================================================
// ==================================================================


#endif /* INC_DEFINES_H_ */
