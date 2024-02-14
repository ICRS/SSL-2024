/*
 * config.h
 *
 *  Created on: Feb 12, 2024
 *      Author: bens1
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_



/* NRF24 SPI Communication */

#define NRF24_SPI hspi2


/* Motor CAN Communication */

#define CHASSIS_CAN hfdcan1
#define GIMBAL_CAN hfdcan2

#define SSL_MOTOR_ID 0x200
#define CAN_6020_M4_ID 0x208


#endif /* INC_CONFIG_H_ */
