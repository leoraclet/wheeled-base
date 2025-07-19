#ifndef SRC_MAINPROG_H_
#define SRC_MAINPROG_H_


#include "defines.h"
#include "imu.h"
#include "uart.h"
#include "moteurs.h"
#include "encoders.h"
#include "pid.h"
#include "odometrie.h"
#include "asserv.h"
#include "tinyefk.h"
#include "trajectoires.h"
#include "altimu.h"
#include "trajet.h"
#include "strategie.h"
#include "temps.h"
#include "fusion.h"
#include "log.h"


void init();
void mainprog();


#endif /* SRC_MAINPROG_H_ */
