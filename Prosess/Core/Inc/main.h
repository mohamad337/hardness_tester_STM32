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
#define THE_CS_Pin GPIO_PIN_3
#define THE_CS_GPIO_Port GPIOE
#define EXIN3_Pin GPIO_PIN_13
#define EXIN3_GPIO_Port GPIOC
#define h_ADC_Pin GPIO_PIN_5
#define h_ADC_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define ETH_RST_Pin GPIO_PIN_10
#define ETH_RST_GPIO_Port GPIOB
#define STEP2_PULS_Pin GPIO_PIN_6
#define STEP2_PULS_GPIO_Port GPIOH
#define STEP2_EN_Pin GPIO_PIN_7
#define STEP2_EN_GPIO_Port GPIOH
#define STEP2_DIR_Pin GPIO_PIN_8
#define STEP2_DIR_GPIO_Port GPIOH
#define LENZ_R_Pin GPIO_PIN_9
#define LENZ_R_GPIO_Port GPIOH
#define LENZ_L_Pin GPIO_PIN_10
#define LENZ_L_GPIO_Port GPIOH
#define STEP_PULSE_Pin GPIO_PIN_14
#define STEP_PULSE_GPIO_Port GPIOB
#define STEP_EN_Pin GPIO_PIN_11
#define STEP_EN_GPIO_Port GPIOD
#define STEP_DIR_Pin GPIO_PIN_12
#define STEP_DIR_GPIO_Port GPIOD
#define REL_OUT_Pin GPIO_PIN_13
#define REL_OUT_GPIO_Port GPIOD
#define FASTIN1_Pin GPIO_PIN_8
#define FASTIN1_GPIO_Port GPIOC
#define FASTIN2_Pin GPIO_PIN_9
#define FASTIN2_GPIO_Port GPIOC
#define MBUSEN_Pin GPIO_PIN_8
#define MBUSEN_GPIO_Port GPIOA
#define OUTRST_Pin GPIO_PIN_15
#define OUTRST_GPIO_Port GPIOA
#define FASTIN3_Pin GPIO_PIN_10
#define FASTIN3_GPIO_Port GPIOC
#define EXIN4_Pin GPIO_PIN_11
#define EXIN4_GPIO_Port GPIOC
#define FASTIN4_Pin GPIO_PIN_12
#define FASTIN4_GPIO_Port GPIOC
#define EOUT5_Pin GPIO_PIN_2
#define EOUT5_GPIO_Port GPIOD
#define EOUT6_Pin GPIO_PIN_3
#define EOUT6_GPIO_Port GPIOD
#define EOUT7_Pin GPIO_PIN_4
#define EOUT7_GPIO_Port GPIOD
#define EOUT8_Pin GPIO_PIN_5
#define EOUT8_GPIO_Port GPIOD
#define EOUT2_Pin GPIO_PIN_6
#define EOUT2_GPIO_Port GPIOD
#define EOUT1_Pin GPIO_PIN_7
#define EOUT1_GPIO_Port GPIOD
#define EOUT4_Pin GPIO_PIN_9
#define EOUT4_GPIO_Port GPIOG
#define EOUT3_Pin GPIO_PIN_10
#define EOUT3_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
