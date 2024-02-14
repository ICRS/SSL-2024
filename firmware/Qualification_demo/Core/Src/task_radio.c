#include "library_can.h"
#include "library_nrf24.h"
#include "debug_functions.h"
#include "tasks.h"

extern SPI_HandleTypeDef NRF24_SPI;



// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

// Length of received payload
uint8_t payload_length;

#define CUSTOM_ESB_RX		1

void startRadioTask(void *argument) {

	uint8_t nRF24_payload[32];

	NRF24_HandleTypeDef nrf24;

	can_filter_init();


	nRF24_AssignCEPin(&nrf24, NRF_CE_GPIO_Port, NRF_CE_Pin);
	nRF24_AssignCSNPin(&nrf24, NRF_CSN_GPIO_Port, NRF_CSN_Pin);
	nRF24_AssignLEDPin(&nrf24, LED_GREEN_GPIO_Port, LED_GREEN_Pin);

	nRF24_Init(&nrf24, &NRF24_SPI);



	// RX/TX disabled
	nRF24_RX_OFF(&nrf24);


	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state


	// This is simple receiver with Enhanced ShockBurst:
	//   - RX address: 'ESB'
	//   - payload: 10 bytes
	//   - RF channel: 40 (2440MHz)
	//   - data rate: 2Mbps
	//   - CRC scheme: 2 byte

	// The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)

	// Set RF channel
	nRF24_SetRFChannel(&nrf24, 40);

	// Set data rate
	nRF24_SetDataRate(&nrf24, nRF24_DR_2Mbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(&nrf24, nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(&nrf24, 3);

	// Configure RX PIPE
	static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
	nRF24_SetAddr(&nrf24, nRF24_PIPE1, nRF24_ADDR); // program address for pipe
	nRF24_SetRXPipe(&nrf24, nRF24_PIPE1, nRF24_AA_ON, 10); // Auto-ACK: enabled, payload length: 10 bytes

	// Set TX power for Auto-ACK (maximum, to ensure that transmitter will hear ACK reply)
	nRF24_SetTXPower(&nrf24, nRF24_TXPWR_0dBm);

	// Set operational mode (PRX == receiver)
	nRF24_SetOperationalMode(&nrf24, nRF24_MODE_RX);

	// Clear any pending IRQ flags
	nRF24_ClearIRQFlags(&nrf24);

	// Wake the transceiver
	nRF24_SetPowerMode(&nrf24, nRF24_PWR_UP);

	// Enable DPL
	nRF24_SetDynamicPayloadLength(&nrf24, nRF24_DPL_ON);

	nRF24_SetPayloadWithAck(&nrf24, 1);


	// Put the transceiver to the RX mode
	nRF24_RX_ON(&nrf24);


	// The main loop
	while (1) {
		//
		// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
		//
		// This is far from best solution, but it's ok for testing purposes
		// More smart way is to use the IRQ pin :)
		//

		uint8_t payload;

		if (nRF24_GetStatus_RXFIFO(&nrf24) != nRF24_STATUS_RXFIFO_EMPTY) {
			// Get a payload from the transceiver
			pipe = nRF24_ReadPayloadDpl(&nrf24, &payload, &payload_length);
			if(payload_length > 0) {
				Toggle_LED(&nrf24);
				nRF24_WriteAckPayload(&nrf24, pipe, "aCk PaYlOaD",11);
			}

			// Clear all pending IRQ flags
			nRF24_ClearIRQFlags(&nrf24);

			switch (payload){

			case ('0'):

					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
					break;
			case ('b'):
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
					CAN_cmd_chassis(DEFAULT_SPEED, 0, -DEFAULT_SPEED , 0);
					osDelay(500);
					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
					break;

			case ('f'):
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
					CAN_cmd_chassis(-DEFAULT_SPEED, 0, DEFAULT_SPEED , 0);
					osDelay(500);
					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
					break;
			case ('r'):
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
					CAN_cmd_chassis(DEFAULT_SPEED, DEFAULT_SPEED, DEFAULT_SPEED, 0);
					osDelay(100);
					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
					break;
			case ('l'):
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
					CAN_cmd_chassis(-DEFAULT_SPEED, -DEFAULT_SPEED, -DEFAULT_SPEED, 0);
					osDelay(100);
					CAN_cmd_chassis(0, 0, 0, 0);
					HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
					break;
			case ('t'):
					HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, SET);
					osDelay(500);
					HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, RESET);
					break;
			case (0x36):

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

const float invsqrt3 = 1 / sqrt(3);
const float sqrt3 = sqrt(3);

void calculateIK(float velocity, float angle, float angular_velocity, float* motor_commands) {
	float vsin = velocity * sin(angle);
	float vcos = velocity * cos(angle);

	motor_commands[0] = angular_velocity + 2 * vsin;
	motor_commands[1] = angular_velocity + sqrt3 * vcos - vsin;
	motor_commands[2] = angular_velocity - sqrt3 * vcos - vsin;


}
