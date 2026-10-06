/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define VIN_Pin GPIO_PIN_2
#define VIN_GPIO_Port GPIOB
#define LED_BLINK_Pin GPIO_PIN_8
#define LED_BLINK_GPIO_Port GPIOA
#define PEN_Pin GPIO_PIN_7
#define PEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// Define maximum ADC value (12-bit resolution)
#define ADC_MAX_VALUE 4095
#define ADC_THRESHOLD_LOW (ADC_MAX_VALUE * 0.15f)  // 20% of max value
#define ADC_THRESHOLD_HIGH (ADC_MAX_VALUE * 0.85f)  // 80% of max value

// Define buffer sizes
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define CMD_BUFFER_SIZE 32

// Define analog input count
#define TOTAL_ANALOG_INPUTS 20

// Define debug
//#define DEBUG 0

// Define digital input count
#define TOTAL_DIGITAL_INPUTS 13

// Command definitions
#define CMD_GET_ALL_ANALOG "AT-GETA"
#define CMD_GET_ALL_DIGITAL "AT-GETD"
#define CMD_CONTINUOUS "AT-CONTINUA"
#define CMD_RUN "AT-RUN"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
