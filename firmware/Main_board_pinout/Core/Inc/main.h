/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAG_DRDY_Pin GPIO_PIN_3
#define MAG_DRDY_GPIO_Port GPIOE
#define MAG_INT_Pin GPIO_PIN_13
#define MAG_INT_GPIO_Port GPIOC
#define GPIO3_Pin GPIO_PIN_6
#define GPIO3_GPIO_Port GPIOF
#define GPIO4_Pin GPIO_PIN_7
#define GPIO4_GPIO_Port GPIOF
#define GPIO2_Pin GPIO_PIN_8
#define GPIO2_GPIO_Port GPIOF
#define GPIO1_Pin GPIO_PIN_9
#define GPIO1_GPIO_Port GPIOF
#define BTN1_Pin GPIO_PIN_0
#define BTN1_GPIO_Port GPIOA
#define TRIG2_Pin GPIO_PIN_1
#define TRIG2_GPIO_Port GPIOA
#define TRIG1_Pin GPIO_PIN_5
#define TRIG1_GPIO_Port GPIOA
#define CAN2_SILENT_Pin GPIO_PIN_6
#define CAN2_SILENT_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_4
#define BTN2_GPIO_Port GPIOC
#define CAN1_SILENT_Pin GPIO_PIN_11
#define CAN1_SILENT_GPIO_Port GPIOE
#define NRF24_CE_Pin GPIO_PIN_8
#define NRF24_CE_GPIO_Port GPIOD
#define NRF24_MODE_Pin GPIO_PIN_9
#define NRF24_MODE_GPIO_Port GPIOD
#define NRF24_IRQ_Pin GPIO_PIN_11
#define NRF24_IRQ_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_7
#define IMU_INT1_GPIO_Port GPIOC
#define EEPROM_WC_Pin GPIO_PIN_8
#define EEPROM_WC_GPIO_Port GPIOC
#define IMU_INT2_Pin GPIO_PIN_2
#define IMU_INT2_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOG
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_7
#define BOOT_GPIO_Port GPIOB
#define SDMMC2_CD_Pin GPIO_PIN_1
#define SDMMC2_CD_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
