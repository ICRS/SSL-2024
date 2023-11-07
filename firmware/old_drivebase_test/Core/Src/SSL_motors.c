/*
 * motors.c
 *
 *  Created on: Nov 7, 2023
 *      Author: bens1
 */

#include <SSL_motors.h>
#include "main.h"

void SSL_Motors_Init(){

	/* Set GPIO to known states */
	HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, RESET);
	HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, RESET);
	HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, RESET);
	HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, RESET);
	HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, RESET);
	HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, RESET);

	/* Start timers for PWM */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* M1_EN */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* M2_EN */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); /* M3_EN */

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

/* Set speed between -1000 and +1000 */
void SSL_Motor_Set_Speed(uint8_t motor, int16_t speed){

	switch (motor){

		case 1:
			if (speed == 0){
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, RESET);
			}
			else if (speed > 0){
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
			}
			else{
				HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, SET);
				HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -speed);
			}


			break;

		case 2:
			if (speed == 0){
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, RESET);
			}
			else if (speed > 0){
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
			}
			else{
				HAL_GPIO_WritePin(M2_IN1_GPIO_Port, M2_IN1_Pin, SET);
				HAL_GPIO_WritePin(M2_IN2_GPIO_Port, M2_IN2_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -speed);
			}


			break;

		case 3:
			if (speed == 0){
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, RESET);
			}
			else if (speed > 0){
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, RESET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);
			}
			else{
				HAL_GPIO_WritePin(M3_IN1_GPIO_Port, M3_IN1_Pin, SET);
				HAL_GPIO_WritePin(M3_IN2_GPIO_Port, M3_IN2_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -speed);
			}


			break;

		default:

		}
}


/* TODO: Set how fast the robot is moving */
void SSL_Robot_Set_Speed(int16_t speed){

}

/* TODO: Set which way the robot is moving */
void SSL_Robot_Set_Heading(int16_t heading){

}

/* TODO: Set which way the robot is facing */
void SSL_Robot_Set_Facing(int16_t facing){

}

