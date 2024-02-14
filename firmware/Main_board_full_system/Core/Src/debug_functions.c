/*
 * debug_functions.c
 *
 *  Created on: Feb 11, 2024
 *      Author: bens1
 */

#include "debug_functions.h"
#include "memory.h"
#include "main.h"

extern UART_HandleTypeDef debugUARTHandle;

//extern UART_HandleTypeDef huart6;

#define HEX_CHARS      "0123456789ABCDEF"

void UART_SendChar(char b) {
	HAL_UART_Transmit(&debugUARTHandle, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
	HAL_UART_Transmit(&debugUARTHandle, (uint8_t *) string, (uint16_t) strlen(string), 200);
}

void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4) % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}

void UART_SendHex8(uint16_t num) {
	UART_SendChar(HEX_CHARS[(num >> 4) % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = (char) (num % 10 + '0'); while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}
