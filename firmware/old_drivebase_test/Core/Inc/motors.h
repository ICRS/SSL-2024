/*
 * motors.h
 *
 *  Created on: Nov 7, 2023
 *      Author: bens1
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stdint.h"
void SSL_Motors_Init();

void SSL_Motor_Set_Speed(uint8_t motor, int16_t speed);

#endif /* INC_MOTORS_H_ */
