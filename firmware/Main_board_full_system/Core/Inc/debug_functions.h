/*
 * debug_functions.h
 *
 *  Created on: Feb 11, 2024
 *      Author: bens1
 */

#ifndef INC_DEBUG_FUNCTIONS_H_
#define INC_DEBUG_FUNCTIONS_H_

#include "stdint.h"
#include "stm32h7xx_hal.h"

void UART_SendChar(char b);
void UART_SendStr(char *string);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendHex8(uint16_t num);
void UART_SendInt(int32_t num);

#define debugUARTHandle huart1

#endif /* INC_DEBUG_FUNCTIONS_H_ */
