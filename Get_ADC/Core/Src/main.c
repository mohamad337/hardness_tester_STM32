/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#ifdef DEBUG
//  #define USE_HSI_FOR_DEBUG
//#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// SPI communication buffers
uint8_t rxBuffer[RX_BUFFER_SIZE] = {0};
uint8_t txBuffer[TX_BUFFER_SIZE] = {0};

uint8_t txData;        // single byte used for ISR transfers
uint8_t rxData;
volatile uint16_t spiTxIndex = 0;
volatile uint16_t spiRxIndex = 0;


volatile uint8_t spiTransferComplete = 1;
volatile uint8_t commandReady = 0;
char commandBuffer[CMD_BUFFER_SIZE] = {0};

// ADC values storage
uint16_t analogValues[TOTAL_ANALOG_INPUTS+5] = {0};
uint16_t vinValue = 0;

// Digital input values storage
uint8_t digitalValues[TOTAL_DIGITAL_INPUTS] = {0};

// Continuous mode variables
uint8_t continuousMode = 0;
uint8_t continuousChannel = 0;
uint32_t lastContinuousTime = 0;
#define CONTINUOUS_INTERVAL_MS 1000  // 1 second interval


// RUN mode variables
uint8_t runMode = 0;
uint32_t lastRunTime = 0;
#define RUN_INTERVAL_MS 100  // Read every 100ms for fast monitoring

// Error tracking
uint8_t errorDetected = 0;
uint8_t errorChannel = 0;
uint16_t errorValue = 0;


// PIN definitions for digital inputs
GPIO_TypeDef* digitalPorts[TOTAL_DIGITAL_INPUTS] = {
    GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC,  // PC13-PC9
    GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,                // PB10-PB6
    GPIOD                                             // PD2
};

uint16_t digitalPins[TOTAL_DIGITAL_INPUTS] = {
    GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9,  // PC13-PC9
    GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6,                            // PB10-PB6
    GPIO_PIN_2                                                                               // PD2
};

// ADC channel mapping
typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t channel;
} ADC_ChannelMap;

ADC_ChannelMap adcChannelMap[TOTAL_ANALOG_INPUTS] = {
    // ADC1 channels (8)
    {&hadc1, ADC_CHANNEL_1}, {&hadc1, ADC_CHANNEL_2}, {&hadc1, ADC_CHANNEL_3}, {&hadc1, ADC_CHANNEL_4},
    {&hadc1, ADC_CHANNEL_5}, {&hadc1, ADC_CHANNEL_6}, {&hadc1, ADC_CHANNEL_7}, {&hadc1, ADC_CHANNEL_8},
    
    // ADC2 channels (7)
    {&hadc2, ADC_CHANNEL_1}, {&hadc2, ADC_CHANNEL_2}, {&hadc2, ADC_CHANNEL_3}, {&hadc2, ADC_CHANNEL_5},
    {&hadc2, ADC_CHANNEL_9}, {&hadc2, ADC_CHANNEL_11}, {&hadc2, ADC_CHANNEL_12},
    
    // ADC3 channels (3)
    {&hadc3, ADC_CHANNEL_1}, {&hadc3, ADC_CHANNEL_5}, {&hadc3, ADC_CHANNEL_12},
    
    // ADC4 channels (2)
    {&hadc4, ADC_CHANNEL_3}, {&hadc4, ADC_CHANNEL_4}
    
    // Additional channels to reach 22 (assuming they're configured similarly)
    //{&hadc1, ADC_CHANNEL_9}, {&hadc1, ADC_CHANNEL_10}, 
		//{&hadc1, ADC_CHANNEL_11}, {&hadc1, ADC_CHANNEL_12}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// Custom function prototypes
void ReadAllAnalogInputs(void);
void ReadAllDigitalInputs(void);
void ProcessCommand(char* cmd);
void SendResponse(const char* response);
uint16_t ReadADCChannel(ADC_HandleTypeDef* hadc, uint32_t channel);
void CheckVinAndControlPen(void);
void HandleContinuousMode(void);
void PrepareContinuousResponse(void);
void HandleRunMode(void);
void CheckAnalogErrors(void);
void SendErrorViaUART(uint8_t channel, uint16_t value);
void SendErrorViaSPI(uint8_t channel, uint16_t value);
void Debug_ADCs(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //#ifdef DEBUG
    // Ensure vector table is relocated for debug
    //SCB->VTOR = FLASH_BASE | 0x0000;
  //#endif
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	//#ifdef DEBUG
    // Increase the SysTick priority for better debug responsiveness
    //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  //#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	// Start ADC conversions
	///HAL_ADC_Start(&hadc1);
	///HAL_ADC_Start(&hadc2);
	///HAL_ADC_Start(&hadc3);
	///HAL_ADC_Start(&hadc4);
	
	// Start SPI in interrupt mode
	txData = 0xFF;
	HAL_SPI_TransmitReceive_IT(&hspi3, &txData, &rxData, 1);

	// Initial reading of inputs
	ReadAllAnalogInputs();
	ReadAllDigitalInputs();
	CheckVinAndControlPen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		  // Check if a command is ready to be processed
  if (commandReady) {
    ProcessCommand(commandBuffer);
    commandReady = 0;
    memset(commandBuffer, 0, CMD_BUFFER_SIZE);
  }
  
  // Handle continuous mode if active
  if (continuousMode) {
    HandleContinuousMode();
  }
  
	// Handle RUN mode if active
  if (runMode) {
    HandleRunMode();
  }
	
  // Periodically read all inputs
  static uint32_t lastReadTime = 0;
  if (HAL_GetTick() - lastReadTime > 100) {  // Read every 100ms
    ReadAllAnalogInputs();
    ReadAllDigitalInputs();
    CheckVinAndControlPen();
    lastReadTime = HAL_GetTick();
  }
  

		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // Small delay to prevent CPU hogging
  HAL_Delay(10); 

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 7;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 2;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PEN_GPIO_Port, PEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC6
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLINK_Pin */
  GPIO_InitStruct.Pin = LED_BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLINK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PEN_Pin */
  GPIO_InitStruct.Pin = PEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PEN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Read all analog inputs and store their values
  * @param  None
  * @retval None
  */
void ReadAllAnalogInputs(void) {
  for (int i = 0; i < TOTAL_ANALOG_INPUTS; i++) {
    analogValues[i] = ReadADCChannel(adcChannelMap[i].hadc, adcChannelMap[i].channel);
  }
  
  // Store VIN value separately (ADC2_IN12 is at index 13 in our mapping)
  vinValue = analogValues[13];
}


/**
  * @brief  Read a specific ADC channel
  * @param  hadc: ADC handle
  * @param  channel: ADC channel to read
  * @retval ADC conversion result
  */

uint16_t ReadADCChannel(ADC_HandleTypeDef* hadc, uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t value;
  // Configure the channel
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  
  // Start conversion
  if (HAL_ADC_Start(hadc) != HAL_OK) {
    //Error_Handler();
		return 0;
  }
  
  // Wait for conversion complete
  if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        value = HAL_ADC_GetValue(hadc);
    }
	
  
  // Stop conversion
  HAL_ADC_Stop(hadc);
  
  return value;
}


/**
  * @brief  Read all digital inputs and store their values
  * @param  None
  * @retval None
  */
void ReadAllDigitalInputs(void) {
  for (int i = 0; i < TOTAL_DIGITAL_INPUTS; i++) {
    digitalValues[i] = HAL_GPIO_ReadPin(digitalPorts[i], digitalPins[i]);
  }
	  HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);

}

/**
  * @brief  Check VIN value and control PEN pin accordingly
  * @param  None
  * @retval None
  */
void CheckVinAndControlPen(void) {
  if (vinValue < ADC_THRESHOLD_HIGH) {
    // VIN is below 80% of max, turn on PEN to enable master power
    HAL_GPIO_WritePin(PEN_GPIO_Port, PEN_Pin, GPIO_PIN_SET);
  } else {
    // VIN is above 80% of max, turn off PEN
    HAL_GPIO_WritePin(PEN_GPIO_Port, PEN_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief  Process received command
  * @param  cmd: Command string to process
  * @retval None
  */
void ProcessCommand(char* cmd) {
  // Convert to uppercase for case-insensitive comparison
  for (int i = 0; cmd[i]; i++) {
    if (cmd[i] >= 'a' && cmd[i] <= 'z') {
      cmd[i] -= ('a' - 'A');
    }
  }
  
  if (strncmp(cmd, CMD_GET_ALL_ANALOG, strlen(CMD_GET_ALL_ANALOG)) == 0) {
    // Prepare response with all analog values
    char response[TX_BUFFER_SIZE] = "AVALUES:";
    char temp[16];
    
    for (int i = 0; i < TOTAL_ANALOG_INPUTS; i++) {
      sprintf(temp, "%d", analogValues[i]);
      strcat(response, temp);
      if (i < TOTAL_ANALOG_INPUTS - 1) {
        strcat(response, ",");
      }
    }
    
    SendResponse(response);
  }
	else if (strncmp(cmd, CMD_GET_ALL_DIGITAL, strlen(CMD_GET_ALL_DIGITAL)) == 0) {
    // Prepare response with all digital values
    char response[TX_BUFFER_SIZE] = "DVALUES:";
    char temp[4];
    
    for (int i = 0; i < TOTAL_DIGITAL_INPUTS; i++) {
      sprintf(temp, "%d", digitalValues[i]);
      strcat(response, temp);
      if (i < TOTAL_DIGITAL_INPUTS - 1) {
        strcat(response, ",");
      }
    }
    
    SendResponse(response);
  }
	else if (strncmp(cmd, CMD_CONTINUOUS, strlen(CMD_CONTINUOUS)) == 0) {
    // Parse channel number
    char* channelStr = cmd + strlen(CMD_CONTINUOUS);
    
    // Skip whitespace
    while (*channelStr == ' ') channelStr++;
    
    // Check if it's a valid channel number
    if (*channelStr == 'A' || *channelStr == 'a') {
      channelStr++;  // Skip 'A'
      int channel = atoi(channelStr);
      
      if (channel >= 1 && channel <= TOTAL_ANALOG_INPUTS) {
        continuousMode = 1;
        continuousChannel = channel - 1;  // Convert to 0-based index
        lastContinuousTime = HAL_GetTick();
        
        // Send immediate response
        PrepareContinuousResponse();
      } else {
        SendResponse("ERROR: Invalid channel");
      }
			 } else if (strncmp(channelStr, "STOP", 4) == 0) {
      continuousMode = 0;
      SendResponse("CONTINUOUS STOPPED");
    } else {
      SendResponse("ERROR: Invalid command format");
    }
  }else if (strncmp(cmd, CMD_RUN, strlen(CMD_RUN)) == 0) {
    // Parse RUN command parameters
    char* paramStr = cmd + strlen(CMD_RUN);
    
    // Skip whitespace
    while (*paramStr == ' ') paramStr++;
    
    if (strncmp(paramStr, "START", 5) == 0) {
      runMode = 1;
      lastRunTime = HAL_GetTick();
      errorDetected = 0;
      SendResponse("RUN MODE STARTED");
    }
    else if (strncmp(paramStr, "STOP", 4) == 0) {
      runMode = 0;
      SendResponse("RUN MODE STOPPED");
    }
		else if (strncmp(paramStr, "STATUS", 6) == 0) {
      char response[64];
      if (errorDetected) {
        sprintf(response, "ERROR DETECTED: Channel A%d = %d", errorChannel + 1, errorValue);
      } else {
        sprintf(response, "NO ERRORS DETECTED");
      }
      SendResponse(response);
    }
    else {
      SendResponse("ERROR: Invalid RUN command format. Use START, STOP, or STATUS");
    }
	
	
	}
	
	
  else {
    SendResponse("ERROR: Unknown command");
  }
}

/**
  * @brief  Send response via SPI
  * @param  response: Response string to send
  * @retval None
  */
void SendResponse(const char* response)
{
    // Prevent race condition if SPI busy
    while (!spiTransferComplete);

    spiTransferComplete = 0;

    // Clear buffer
    memset(txBuffer, 0, TX_BUFFER_SIZE);

    // Copy response safely
    strncpy((char*)txBuffer, response, TX_BUFFER_SIZE - 1);

    // Reset index
    spiTxIndex = 0;

    // Load first byte
    txData = txBuffer[spiTxIndex++];

    // Start SPI interrupt transmission (1 byte)
    HAL_SPI_TransmitReceive_IT(&hspi3, &txData, &rxData, 1);
}

/**
  * @brief  Handle continuous mode operation
  * @param  None
  * @retval None
  */

void HandleContinuousMode(void) {
  if (HAL_GetTick() - lastContinuousTime >= CONTINUOUS_INTERVAL_MS) {
    PrepareContinuousResponse();
    lastContinuousTime = HAL_GetTick();
  }
}

/**
  * @brief  Prepare and send continuous mode response
  * @param  None
  * @retval None
  */
void PrepareContinuousResponse(void) {
  char response[32];
  sprintf(response, "A%d:%d", continuousChannel + 1, analogValues[continuousChannel]);
  SendResponse(response);
}



void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        rxBuffer[spiRxIndex++] = rxData;

        HAL_SPI_Receive_IT(&hspi3, &rxData, 1);
    }
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        if (spiTxIndex < sizeof(txBuffer))
        {
            txData = txBuffer[spiTxIndex++];
            HAL_SPI_Transmit_IT(&hspi3, &txData, 1);
        }
        else
        {
            spiTxIndex = 0;   // reset
        }
    }
}




/**
  * @brief  Handle RUN mode operation
  * @param  None
  * @retval None
  */
void HandleRunMode(void) {
  if (HAL_GetTick() - lastRunTime >= RUN_INTERVAL_MS) {
    // Fast read of analog inputs
    ReadAllAnalogInputs();
    
    // Check for errors
    CheckAnalogErrors();
    
    lastRunTime = HAL_GetTick();
  }
}

/**
  * @brief  Check analog inputs for values outside the 20%-80% range
  * @param  None
  * @retval None
  */
void CheckAnalogErrors(void) {
  for (int i = 0; i < TOTAL_ANALOG_INPUTS; i++) {
    if (analogValues[i] < ADC_THRESHOLD_LOW || analogValues[i] > ADC_THRESHOLD_HIGH) {
      // Error detected
      if (!errorDetected || errorChannel != i || errorValue != analogValues[i]) {
        errorDetected = 1;
        errorChannel = i;
        errorValue = analogValues[i];
        
        // Send error via UART
        SendErrorViaUART(i, analogValues[i]);
        
        // Send error via SPI
        SendErrorViaSPI(i, analogValues[i]);
      }
      return;  // Only report one error at a time
    }
  }
  
  // No errors detected
  if (errorDetected) {
    errorDetected = 0;
    SendResponse("ALL VALUES WITHIN RANGE");
  }
}

/**
  * @brief  Send error message via UART
  * @param  channel: Channel number where error was detected
  * @param  value: The value that caused the error
  * @retval None
  */
void SendErrorViaUART(uint8_t channel, uint16_t value) {
  char errorMsg[128];
  sprintf(errorMsg, "ERROR: Channel A%d out of range! Value: %d (Range: %d-%d)\r\n", 
          channel + 1, value, (int)ADC_THRESHOLD_LOW, (int)ADC_THRESHOLD_HIGH);
  
  // Transmit via UART
  HAL_UART_Transmit(&huart1, (uint8_t*)errorMsg, strlen(errorMsg), 100);
}

/**
  * @brief  Send error message via SPI
  * @param  channel: Channel number where error was detected
  * @param  value: The value that caused the error
  * @retval None
  */
void SendErrorViaSPI(uint8_t channel, uint16_t value) {
  char errorMsg[128];
  sprintf(errorMsg, "ERROR: Channel A%d out of range! Value: %d", 
          channel + 1, value);
  
  // Send via SPI
  SendResponse(errorMsg);
}

void Debug_ADCs(void) {
    char debug[100];
    uint16_t value;
    
    // Test ADC1 Channel 1
    value = ReadADCChannel(&hadc1, ADC_CHANNEL_1);
    sprintf(debug, "ADC1 CH1: %d\r\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
    
    // Test ADC2 Channel 1
    value = ReadADCChannel(&hadc2, ADC_CHANNEL_1);
    sprintf(debug, "ADC2 CH1: %d\r\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
    
    // Test ADC3 Channel 1
    value = ReadADCChannel(&hadc3, ADC_CHANNEL_1);
    sprintf(debug, "ADC3 CH1: %d\r\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
    
    // Test ADC4 Channel 3
    value = ReadADCChannel(&hadc4, ADC_CHANNEL_3);
    sprintf(debug, "ADC4 CH3: %d\r\n", value);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 100);
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
