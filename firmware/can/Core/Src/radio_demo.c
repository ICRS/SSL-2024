#include <memory.h>
#include "support.h"
#include "nrf24.h"

#include "CAN_receive.h"
#include "bsp_can.h"
//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width


#define HEX_CHARS      "0123456789ABCDEF"

extern UART_HandleTypeDef huart6;

void UART_SendChar(char b) {
	HAL_UART_Transmit(&huart6, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
	HAL_UART_Transmit(&huart6, (uint8_t *) string, (uint16_t) strlen(string), 200);
}

void Toggle_LED() {
	HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
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


uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

// Length of received payload
uint8_t payload_length;

#define CUSTOM_ESB_RX		1


int runRadio(void) {
	UART_SendStr("\r\nRX is online.\r\n");

	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	UART_SendStr("nRF24L01+ check: ");

	if (!nRF24_Check()) {
		UART_SendStr("FAIL\r\n");
		while (1) {
			Toggle_LED();
			Delay_ms(50);
		}
	}

	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

	// This is simple receiver with Enhanced ShockBurst:
	//   - RX address: 'ESB'
	//   - payload: 10 bytes
	//   - RF channel: 40 (2440MHz)
	//   - data rate: 2Mbps
	//   - CRC scheme: 2 byte

	// The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)

	// Set RF channel
	nRF24_SetRFChannel(40);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_2Mbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure RX PIPE
	static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
	nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for pipe
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10); // Auto-ACK: enabled, payload length: 10 bytes

	// Set TX power for Auto-ACK (maximum, to ensure that transmitter will hear ACK reply)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Set operational mode (PRX == receiver)
	nRF24_SetOperationalMode(nRF24_MODE_RX);

	// Clear any pending IRQ flags
	nRF24_ClearIRQFlags();

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	// Enable DPL
	nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);

	nRF24_SetPayloadWithAck(1);


	// Put the transceiver to the RX mode
	nRF24_CE_H();


	// The main loop
	while (1) {
		//
		// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
		//
		// This is far from best solution, but it's ok for testing purposes
		// More smart way is to use the IRQ pin :)
		//

		uint8_t payload;

		if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
			// Get a payload from the transceiver
			pipe = nRF24_ReadPayloadDpl(&payload, &payload_length);
			if(payload_length > 0) {
				Toggle_LED();
				nRF24_WriteAckPayload(pipe, "aCk PaYlOaD",11);
			}

			// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();

			switch (payload){

			case (0x30):
					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, RESET);
					break;
			case (0x31):
					CAN_cmd_chassis(100, 100, 100, 100);
					HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SET);
					break;

			case (0x32):
            			break;
			case (0x33):
						break;
			default:

			}


			// Print a payload contents to UART
			UART_SendStr("RCV PIPE#");
			UART_SendInt(pipe);
			UART_SendStr(" PAYLOAD:>");
			UART_SendBufHex((char *) nRF24_payload, payload_length);
			UART_SendStr("<\r\n");

		}
	}
}
