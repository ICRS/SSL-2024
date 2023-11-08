/*
 * motors.h
 *
 *  Created on: Nov 7, 2023
 *      Author: bens1
 */

#ifndef INC_SSL_MOTORS_H_
#define INC_SSL_MOTORS_H_

#include "stdint.h"

void SSL_Motors_Init();

void SSL_Motor_Set_Speed(uint8_t motor, int16_t speed); /* Set speed of individual motor between -1000 and +1000 */
void SSL_Motor_Set_Speed_All(int16_t motor1_speed, int16_t motor2_speed, int16_t motor3_speed); /* Set speed of all motors between -1000 and +1000 */


void SSL_Robot_Set_Speed(int16_t speed); /* Set speed of robot between -1000 and +1000 */
void SSL_Robot_Set_Heading(int16_t heading); /* Set heading of robot between -180 and +180 degrees */
void SSL_Robot_Set_Facing(int16_t facing); /* Set facing of robot between -180 and +180 degrees */

#endif /* INC_SSL_MOTORS_H_ */
