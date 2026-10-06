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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h> //for rand()
#include "arm_math.h"
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
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Application states
typedef enum {
  STATE_IDLE,
  STATE_FIND_ZERO,
  STATE_CHECK_FORCE_ZERO,
  STATE_MOVE_FAST_DOWN,
  STATE_MOVE_SLOW_DOWN,
  STATE_FIND_START_POSITION,
  STATE_MOVE_TO_TEST_FORCE,
  STATE_WAIT_DWELL_TIME,
  STATE_CALCULATE_RESULTS,
  STATE_RUN_RIGHT_MOTOR,
  STATE_RUN_LEFT_MOTOR,
  STATE_RETURN_TO_ZERO,
	STATE_TEST_MODE,           // New state for testing mode
  STATE_ERROR
} app_state_t;

app_state_t current_state = STATE_IDLE;


// Communication protocol types
typedef enum {
  PROTOCOL_BINARY = 0,
  PROTOCOL_ASCII = 1,
  PROTOCOL_JSON = 2
} protocol_type_t;

// Command codes (matching Python protocol_definitions.py)
typedef enum {
  CMD_NOP = 0x00,
  CMD_TEST_MODE = 0x01,
  CMD_GET_STATUS = 0x02,
  CMD_SET_DIGITAL_OUT = 0x03,
  CMD_SET_ANALOG_OUT = 0x04,
  CMD_SET_RELAY = 0x05,
  CMD_SET_STEPPER = 0x06,
  CMD_SET_DC_MOTOR = 0x07,
  CMD_SET_OSCILLATOR = 0x08,
  CMD_CALIBRATE_LOAD_CELL = 0x09,
  CMD_TARE_LOAD_CELL = 0x0A,
  CMD_EMERGENCY_STOP = 0xFF,
  CMD_REQUEST_REALTIME = 0x10   // New command for real-time data
} command_code_t;

// Response codes
typedef enum {
  RSP_ACK = 0x80,
  RSP_NACK = 0x81,
  RSP_DATA = 0x82,
  RSP_ERROR = 0x83,
  RSP_STATUS = 0x84,
  RSP_REALTIME = 0x85           // New response for real-time data
} response_code_t;


// Hardware status structure (matching Python HardwareStatus)
typedef struct __attribute__((packed)) {
  uint32_t timestamp;           // 4 bytes
  float load_cell_value;        // 4 bytes
  int32_t stepper_position;      // 4 bytes
  int32_t stepper_target;        // 4 bytes
  uint16_t stepper_speed;        // 2 bytes
  uint16_t stepper_accel;        // 2 bytes
  int16_t dc_motor_speed;        // 2 bytes
  uint8_t dc_motor_direction;    // 1 byte
  float dc_motor_current;        // 4 bytes
  uint16_t analog_inputs[20];    // 40 bytes (20 * 2)
  float ma_inputs[2];            // 8 bytes (2 * 4)
  uint32_t digital_inputs;       // 4 bytes
  uint8_t digital_outputs;       // 1 byte
  uint16_t analog_outputs[3];    // 6 bytes (3 * 2)
  uint8_t relay_states;          // 1 byte
  float oscillator_freq;         // 4 bytes
  uint16_t oscillator_phase;     // 2 bytes
  uint8_t oscillator_waveform;   // 1 byte
  uint8_t oscillator_enabled;    // 1 byte
  uint16_t extra_digital_inputs; // 2 bytes
  float temperature;             // 4 bytes
  uint16_t error_flags;          // 2 bytes
} hardware_status_t;


protocol_type_t current_protocol = PROTOCOL_BINARY;
hardware_status_t hw_status;
uint8_t test_mode_enabled = 0;


// Stepper motor control
typedef struct {
  uint32_t step_count;
  uint32_t step_count_at_start;
  uint32_t target_steps;
  uint8_t direction; // 0=up, 1=down
  uint8_t enabled;
	uint16_t speed;    // steps per second
  uint16_t acceleration;
} stepper_t;

stepper_t stepper = {0, 0, 0, 0, 0,1000,500};

// Measurement variables
float h_deep_zero = 0.0f;        // Zero position in um
float h_deep_current = 0.0f;     // Current depth in um
float h_deep_result = 0.0f;      // Final result in um
int32_t raw_force = 0;           // Raw HX711 reading
float force_kg = 0.0f;           // Force in kg
float force_setpoint = 0.0f;     // Target force from flash
uint32_t dwell_time_sec = 0;     // Dwell time from flash

// ADC measurements
uint16_t adc_deep = 0;           // Depth meter (PA5 - ADC1_IN19)
uint16_t adc_power = 0;          // Power meter (ADC1_IN16)
float voltage_deep = 0.0f;       // Depth voltage (0-3V = 0-30000um)
float voltage_power = 0.0f;      // Power voltage (0-3V = 0-30V)

// DC motor control
uint8_t dc_motor_running = 0;
uint32_t dc_motor_start_time = 0;

// System timing
uint32_t state_start_time = 0;
uint32_t dwell_start_time = 0;
uint32_t last_step_time = 0;
uint32_t system_tick = 0;


// Flags
uint8_t measurement_started = 0;
uint8_t camera_triggered = 0;

// Communication buffers
uint8_t rx_buffer[256];
uint8_t tx_buffer[256];
volatile uint8_t rx_index = 0;
volatile uint8_t command_ready = 0;
volatile uint8_t uart4_tx_busy = 0;
uint32_t last_uart_tx_time = 0;


// Timing for real-time updates
uint32_t last_realtime_tx_time = 0;
uint32_t last_adc_read_time = 0;
uint32_t last_led_toggle_time = 0;

// Digital outputs state
uint8_t digital_output_state = 0;
uint8_t relay_state = 0;

// AD9833 oscillator control
float oscillator_freq = 1000.0f;
uint16_t oscillator_phase = 0;
uint8_t oscillator_waveform = 0; // 0=sine, 1=triangle, 2=square
uint8_t oscillator_enabled = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI5_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
// Application functions
void application_init(void);
void application_state_machine(void);
void start_measurement_sequence(void);
void stop_measurement_sequence(void);

void process_serial_commands(void);
void send_realtime_data(void);
void send_status_response(void);
void send_ack_response(void);
void send_error_response(uint8_t error_code);

// Stepper motor functions
void stepper_init(void);
void stepper_step(void);
void stepper_set_direction(uint8_t dir);
void stepper_enable(uint8_t enable);
void stepper_move_fast(void);
void stepper_move_slow(void);
void stepper_move_to_position(int32_t target, uint16_t speed, uint16_t accel);


// Measurement functions
void read_adc_values(void);
void read_force_sensor(void);
uint8_t check_force_zero(void);
uint8_t check_force_setpoint(void);

// Flash memory functions (W25Q256)
void flash_init(void);
void flash_read_parameters(void);
uint32_t flash_read_uint32(uint32_t address);
void flash_write_uint32(uint32_t address, uint32_t data);

// DC motor functions
void dc_motor_right_start(void);
void dc_motor_left_start(void);
void dc_motor_stop(void);
void dc_motor_update(void);
void dc_motor_set_speed(int16_t speed);

// I/O functions
void set_digital_output(uint8_t channel, uint8_t state);
void set_analog_output(uint8_t channel, uint16_t value);
void set_relay(uint8_t state);
void read_digital_inputs(void);

// Camera control
void camera_trigger(void);

// Communication functions
void send_results_uart(void);
void send_results_modbus(void);
void uart_send_status(void);

// HX711 functions
void hx711_init(void);
int32_t hx711_read(void);


void uart4_send_realtime(void);
void led1_blink(void);

// Protocol functions
void parse_binary_command(uint8_t *data, uint8_t len);
void parse_ascii_command(char *cmd_str);
void parse_json_command(char *json_str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uart4_tx_busy = 0;
    }
}

/**
  * @brief UART Rx Complete Callback
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) {
    // Check for end of command (newline for ASCII/JSON, or buffer full for binary)
    if (rx_buffer[rx_index] == '\n' || rx_index >= 255) {
      rx_buffer[rx_index] = '\0';
      command_ready = 1;
      rx_index = 0;
    } else {
      rx_index++;
    }
    HAL_UART_Receive_IT(&huart4, &rx_buffer[rx_index], 1);
  }
}


/**
  * @brief Process received serial commands
  */
void process_serial_commands(void)
{
  // Check protocol based on first byte
  if (rx_buffer[0] == '{') {
    // JSON protocol
    parse_json_command((char*)rx_buffer);
  } else if (rx_buffer[0] < 0x20) {
    // Binary protocol (command code < 0x20)
    parse_binary_command(rx_buffer, rx_index);
  } else {
    // ASCII protocol
    parse_ascii_command((char*)rx_buffer);
  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI4_Init();
  MX_TIM12_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_SPI5_Init();
  MX_ADC3_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
// System initialization
application_init();
stepper_init();
hx711_init();
flash_init();

// Read parameters from flash
flash_read_parameters();

	// Initialize hardware status
  memset(&hw_status, 0, sizeof(hardware_status_t));
  hw_status.stepper_speed = 1000;
  hw_status.stepper_accel = 500;
  hw_status.oscillator_freq = 1000.0f;
  hw_status.temperature = 25.0f;
	
// Start ADC conversions
HAL_ADC_Start(&hadc1);
HAL_ADC_Start(&hadc3);

// Enable UART reception for commands
  HAL_UART_Receive_IT(&huart4, &rx_buffer[rx_index], 1);

//printf("Brinell Hardness Tester Ready\r\n");
//printf("Force Setpoint: %.1f kg, Dwell Time: %u sec\r\n", force_setpoint, dwell_time_sec);
  


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//application_state_machine();
		//dc_motor_update();
		
		//read_adc_values();
    //read_force_sensor();

    //uart4_send_realtime();
		
    // Process any received commands
    if (command_ready) {
      process_serial_commands();
      command_ready = 0;
    }

    // Run state machine (only if not in test mode)
    if (!test_mode_enabled) {
      application_state_machine();
    }

    // Update DC motor
    dc_motor_update();

    // Read sensors every 50ms
    if (HAL_GetTick() - last_adc_read_time >= 50) {
      read_adc_values();
      read_force_sensor();
      read_digital_inputs();
      last_adc_read_time = HAL_GetTick();

      // Update hardware status
      hw_status.timestamp = HAL_GetTick();
      hw_status.load_cell_value = force_kg;
      hw_status.stepper_position = stepper.step_count;
      hw_status.dc_motor_current = 0.5f; // Placeholder - implement actual current sensing
      
      // Update analog inputs (example - you'll need to map your actual inputs)
      for (int i = 0; i < 20; i++) {
        hw_status.analog_inputs[i] = adc_deep; // Placeholder
      }
      
      hw_status.ma_inputs[0] = 4.0f + (force_kg / 10.0f) * 16.0f; // Simulated 4-20mA
      hw_status.ma_inputs[1] = 4.0f;
      
      hw_status.temperature = 25.0f + (rand() % 100) / 100.0f; // Simulated
    }

    // Send real-time data if in test mode (every 200ms as requested)
    if (test_mode_enabled && (HAL_GetTick() - last_realtime_tx_time >= 200)) {
      send_realtime_data();
      last_realtime_tx_time = HAL_GetTick();
    }
		
		
		
    led1_blink();

    //HAL_IWDG_Refresh(&hiwdg1);   // refresh watchdog
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 20;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 4;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
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
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00303D5B;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00303D5B;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|ETH_RST_Pin|STEP_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, STEP2_PULS_Pin|STEP2_EN_Pin|STEP2_DIR_Pin|LENZ_R_Pin
                          |LENZ_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_DIR_Pin|REL_OUT_Pin|EOUT5_Pin|EOUT6_Pin
                          |EOUT7_Pin|EOUT8_Pin|EOUT2_Pin|EOUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MBUSEN_Pin|OUTRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EOUT4_Pin|EOUT3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : THE_CS_Pin */
  GPIO_InitStruct.Pin = THE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(THE_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PI8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : EXIN3_Pin FASTIN1_Pin FASTIN2_Pin FASTIN3_Pin
                           EXIN4_Pin FASTIN4_Pin */
  GPIO_InitStruct.Pin = EXIN3_Pin|FASTIN1_Pin|FASTIN2_Pin|FASTIN3_Pin
                          |EXIN4_Pin|FASTIN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin ETH_RST_Pin STEP_PULSE_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|ETH_RST_Pin|STEP_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP2_PULS_Pin STEP2_EN_Pin STEP2_DIR_Pin LENZ_R_Pin
                           LENZ_L_Pin */
  GPIO_InitStruct.Pin = STEP2_PULS_Pin|STEP2_EN_Pin|STEP2_DIR_Pin|LENZ_R_Pin
                          |LENZ_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_EN_Pin */
  GPIO_InitStruct.Pin = STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STEP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_DIR_Pin REL_OUT_Pin EOUT5_Pin EOUT6_Pin
                           EOUT7_Pin EOUT8_Pin EOUT2_Pin EOUT1_Pin */
  GPIO_InitStruct.Pin = STEP_DIR_Pin|REL_OUT_Pin|EOUT5_Pin|EOUT6_Pin
                          |EOUT7_Pin|EOUT8_Pin|EOUT2_Pin|EOUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MBUSEN_Pin OUTRST_Pin */
  GPIO_InitStruct.Pin = MBUSEN_Pin|OUTRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EOUT4_Pin EOUT3_Pin */
  GPIO_InitStruct.Pin = EOUT4_Pin|EOUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Application initialization
void application_init(void)
{
  current_state = STATE_IDLE;
  stepper_init();
  dc_motor_stop();
  
  // Initialize outputs
  HAL_GPIO_WritePin(STEP_EN_GPIO_Port, STEP_EN_Pin, GPIO_PIN_SET); // Stepper disable
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_RESET); // DC motor disable
  
  // Set protocol (can be changed via command)
  current_protocol = PROTOCOL_BINARY; // Default to binary for efficiency
}

/**
  * @brief Stepper motor initialization
  */
void stepper_init(void)
{
  stepper.step_count = 0;
  stepper.direction = 0;
  stepper.enabled = 0;
  stepper.speed = 1000;
  stepper.acceleration = 500;
  stepper_set_direction(0);
  stepper_enable(0);
}


// Generate step pulse
void stepper_step(void)
{
  if (!stepper.enabled) return;
  
  HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_SET);
  for(volatile int i = 0; i < 10; i++); // Short pulse
  HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);
  
  if (stepper.direction) {
    stepper.step_count++;
  } else {
    stepper.step_count--;
  }
}


// Set stepper direction
void stepper_set_direction(uint8_t dir)
{
  HAL_GPIO_WritePin(STEP_DIR_GPIO_Port, STEP_DIR_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
  stepper.direction = dir;
}


// Enable/disable stepper
void stepper_enable(uint8_t enable)
{
  HAL_GPIO_WritePin(STEP_EN_GPIO_Port, STEP_EN_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
  stepper.enabled = enable;
}



/**
  * @brief Move stepper to target position
  */
void stepper_move_to_position(int32_t target, uint16_t speed, uint16_t accel)
{
  stepper.target_steps = target;
  stepper.speed = speed;
  stepper.acceleration = accel;
  
  if (target > stepper.step_count) {
    stepper_set_direction(1); // Down
  } else {
    stepper_set_direction(0); // Up
  }
  
  stepper_enable(1);
}


// Fast movement (2000 Hz)
void stepper_move_fast(void)
{
  if (HAL_GetTick() - last_step_time >= (1000 / stepper.speed)) {
    stepper_step();
    last_step_time = HAL_GetTick();
  }
}


// Slow movement (100 Hz)  
void stepper_move_slow(void)
{
  if (HAL_GetTick() - last_step_time >= (1000 / 100)) { // 100 Hz
    stepper_step();
    last_step_time = HAL_GetTick();
  }
}


// Read ADC values
void read_adc_values(void)
{
  // Read depth meter
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    adc_deep = HAL_ADC_GetValue(&hadc1);
    voltage_deep = (adc_deep * 3.0f) / 65535.0f;
    h_deep_current = voltage_deep * 10000.0f; // Convert to um
  }
  HAL_ADC_Start(&hadc1);
  
  // Read power meter
  if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
    adc_power = HAL_ADC_GetValue(&hadc3);
    voltage_power = (adc_power * 3.0f) / 65535.0f;
  }
  HAL_ADC_Start(&hadc3);
}


// HX711 initialization
void hx711_init(void)
{
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_SET);
}

// Read HX711 force sensor
void read_force_sensor(void)
{
  raw_force = hx711_read();
  // Convert raw reading to kg (calibration values needed)
  force_kg = raw_force / 1000.0f;
}

// Check if force is approximately zero
uint8_t check_force_zero(void)
{
  return (force_kg >= -0.5f && force_kg <= 0.5f);
}

// Check if force reached setpoint
uint8_t check_force_setpoint(void)
{
  return (force_kg >= force_setpoint * 0.95f && force_kg <= force_setpoint * 1.05f);
}

// Flash memory initialization
void flash_init(void)
{
  // SPI4 is already initialized for W25Q256
  HAL_GPIO_WritePin(GPIOE, THE_CS_Pin, GPIO_PIN_SET);
}

// Read parameters from flash
void flash_read_parameters(void)
{
  force_setpoint = (float)flash_read_uint32(0x000000) / 10.0f;
  dwell_time_sec = flash_read_uint32(0x000004);
  
  if (force_setpoint == 0) force_setpoint = 10.0f;
  if (dwell_time_sec == 0) dwell_time_sec = 30;
}


// Read 32-bit value from flash
uint32_t flash_read_uint32(uint32_t address)
{
  uint8_t rx_data[4] = {0};
  
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  
  uint8_t cmd[4] = {0x03, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
  HAL_SPI_Transmit(&hspi4, cmd, 4, 100);
  HAL_SPI_Receive(&hspi4, rx_data, 4, 100);
  
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_SET);
  
  return (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
}


/**
  * @brief DC motor set speed
  */
void dc_motor_set_speed(int16_t speed)
{
  if (speed > 0) {
    dc_motor_right_start();
  } else if (speed < 0) {
    dc_motor_left_start();
  } else {
    dc_motor_stop();
  }
  hw_status.dc_motor_speed = speed;
}


/**
  * @brief DC motor right start
  */
void dc_motor_right_start(void)
{
  HAL_GPIO_WritePin(LENZ_R_GPIO_Port, LENZ_R_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LENZ_L_GPIO_Port, LENZ_L_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET); // Motor enable
  dc_motor_running = 1;
  dc_motor_start_time = HAL_GetTick();
  hw_status.dc_motor_direction = 1;
}


/**
  * @brief DC motor left start
  */
void dc_motor_left_start(void)
{
  HAL_GPIO_WritePin(LENZ_R_GPIO_Port, LENZ_R_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LENZ_L_GPIO_Port, LENZ_L_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_SET); // Motor enable
  dc_motor_running = 2;
  dc_motor_start_time = HAL_GetTick();
  hw_status.dc_motor_direction = 2;
}


/**
  * @brief DC motor stop
  */
void dc_motor_stop(void)
{
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_8, GPIO_PIN_RESET);
  dc_motor_running = 0;
  hw_status.dc_motor_direction = 0;
}


/**
  * @brief DC motor update (auto-stop after 10 seconds)
  */
void dc_motor_update(void)
{
  if (dc_motor_running && (HAL_GetTick() - dc_motor_start_time >= 10000)) {
    dc_motor_stop();
  }
}


// Camera trigger
void camera_trigger(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  camera_triggered = 1;
}

// Main application state machine
void application_state_machine(void)
{
  static uint32_t last_measurement_time = 0;
  
  // Read sensors every 100ms
  if (HAL_GetTick() - last_measurement_time >= 100) {
    read_adc_values();
    read_force_sensor();
    last_measurement_time = HAL_GetTick();
  }
  
  switch (current_state) {
    
    case STATE_IDLE:
      // Wait for start signal (PD11)
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET) {
        start_measurement_sequence();
        current_state = STATE_FIND_ZERO;
        state_start_time = HAL_GetTick();
        ///printf("Starting measurement sequence...\r\n");
      }
      break;
      
    case STATE_FIND_ZERO:
      // Move up until PD11 or PD12 is 1
      stepper_set_direction(0); // Up
      stepper_enable(1);
      stepper_move_fast();
      
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET || 
          HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) {
        stepper_enable(0);
        current_state = STATE_CHECK_FORCE_ZERO;
        //printf("Zero position found. Checking force...\r\n");
      }
      break;
      
    case STATE_CHECK_FORCE_ZERO:
      if (force_kg < 5.0f) {
        force_kg = 0.0f; // Reset force to zero
        read_adc_values();
        h_deep_zero = h_deep_current;
        stepper.step_count_at_start = stepper.step_count;
        current_state = STATE_MOVE_FAST_DOWN;
        printf("Force zero confirmed. Moving fast down...\r\n");
      } else {
        current_state = STATE_ERROR;
        //printf("ERROR: Force too high at zero position: %.1f kg\r\n", force_kg);
      }
      break;
      
    case STATE_MOVE_FAST_DOWN:
      stepper_set_direction(1); // Down
      stepper_enable(1);
      stepper_move_fast();
      
      if (stepper.step_count >= stepper.step_count_at_start + 20000) {
        current_state = STATE_MOVE_SLOW_DOWN;
        //printf("Fast movement completed. Starting slow movement...\r\n");
      }
      break;
      
    case STATE_MOVE_SLOW_DOWN:
      stepper_move_slow();
      
      if (force_kg > 0.5f) { // Force started to change
        stepper_enable(0);
        stepper_set_direction(0); // Up
        stepper_enable(1);
        current_state = STATE_FIND_START_POSITION;
        //printf("Contact detected. Finding start position...\r\n");
      }
      break;
      
    case STATE_FIND_START_POSITION:
      stepper_move_slow();
      
      if (check_force_zero()) {
        stepper_enable(0);
        h_deep_zero = h_deep_current;
        current_state = STATE_MOVE_TO_TEST_FORCE;
        //printf("Start position found. Moving to test force...\r\n");
      }
      break;
      
    case STATE_MOVE_TO_TEST_FORCE:
      stepper_set_direction(1); // Down
      stepper_enable(1);
      stepper_move_slow();
      
      if (check_force_setpoint()) {
        stepper_enable(0);
        dwell_start_time = HAL_GetTick();
        current_state = STATE_WAIT_DWELL_TIME;
        //printf("Test force reached. Waiting dwell time...\r\n");
      }
      break;
      
    case STATE_WAIT_DWELL_TIME:
      if (HAL_GetTick() - dwell_start_time >= (dwell_time_sec * 1000)) {
        read_adc_values();
        current_state = STATE_CALCULATE_RESULTS;
        //printf("Dwell time completed. Calculating results...\r\n");
      }
      break;
      
    case STATE_CALCULATE_RESULTS:
      h_deep_result = h_deep_current - h_deep_zero;
      send_results_uart();
      send_results_modbus();
      current_state = STATE_RUN_RIGHT_MOTOR;
      //printf("Results calculated. Starting right motor...\r\n");
      break;
      
    case STATE_RUN_RIGHT_MOTOR:
      dc_motor_right_start();
      camera_trigger();
      state_start_time = HAL_GetTick();
      current_state = STATE_RUN_LEFT_MOTOR;
      break;
      
    case STATE_RUN_LEFT_MOTOR:
      if (!dc_motor_running) {
        dc_motor_left_start();
        state_start_time = HAL_GetTick();
        current_state = STATE_RETURN_TO_ZERO;
      }
      break;
      
    case STATE_RETURN_TO_ZERO:
      if (!dc_motor_running) {
        stepper_set_direction(0); // Up
        stepper_enable(1);
        stepper_move_fast();
        
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET || 
            HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) {
          stepper_enable(0);
          current_state = STATE_IDLE;
          stop_measurement_sequence();
          //printf("Measurement sequence completed.\r\n");
        }
      }
      break;
      
    case STATE_ERROR:
      stepper_enable(0);
      dc_motor_stop();
      break;
      
    default:
      break;
  }
}

// Start measurement sequence
void start_measurement_sequence(void)
{
  measurement_started = 1;
  stepper.step_count = 0;
  stepper.step_count_at_start = 0;
  camera_triggered = 0;
}

// Stop measurement sequence  
void stop_measurement_sequence(void)
{
  measurement_started = 0;
  stepper_enable(0);
  dc_motor_stop();
}

// Send results via UART4 (RS232)
void send_results_uart(void)
{
  int len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                    "BRINELL_RESULT: Depth=%.1fum, Steps=%u, Force=%.1fkg, Power=%.1fV\r\n",
                    h_deep_result, stepper.step_count, force_kg, voltage_power * 10.0f);
  HAL_UART_Transmit(&huart4, tx_buffer, len, 100);
}

// Send results via USART1 (Modbus RTU)
void send_results_modbus(void)
{
  // Simplified Modbus frame (you'll need proper Modbus implementation)
  uint8_t modbus_frame[32];
  modbus_frame[0] = 0x01; // Slave address
  modbus_frame[1] = 0x10; // Function code 16 (Write Multiple Registers)
  modbus_frame[2] = 0x00; // Starting address high
  modbus_frame[3] = 0x00; // Starting address low
  
  // Convert results to Modbus registers
  uint16_t depth_reg = (uint16_t)(h_deep_result * 10); // Depth in 0.1um
  uint16_t steps_reg = (uint16_t)stepper.step_count;
  uint16_t force_reg = (uint16_t)(force_kg * 10); // Force in 0.1kg
  
  modbus_frame[4] = 0x00; // Quantity high
  modbus_frame[5] = 0x03; // Quantity low (3 registers)
  modbus_frame[6] = 0x06; // Byte count
  
  // Data
  modbus_frame[7] = (depth_reg >> 8) & 0xFF;
  modbus_frame[8] = depth_reg & 0xFF;
  modbus_frame[9] = (steps_reg >> 8) & 0xFF;
  modbus_frame[10] = steps_reg & 0xFF;
  modbus_frame[11] = (force_reg >> 8) & 0xFF;
  modbus_frame[12] = force_reg & 0xFF;
  
  // Calculate CRC (simplified - you need proper CRC calculation)
  HAL_UART_Transmit(&huart1, modbus_frame, 13, 100);
}

// HX711 read implementation
int32_t hx711_read(void)
{
  uint8_t rx_data[3] = {0};
  int32_t value = 0;
  
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  
  if (HAL_SPI_Receive(&hspi2, rx_data, 3, 100) == HAL_OK) {
    value = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];
    if (value & 0x800000) {
      value |= 0xFF000000;
    }
  }
  
  HAL_GPIO_WritePin(THE_CS_GPIO_Port, THE_CS_Pin, GPIO_PIN_SET);
  return value;
}

// External interrupt for start signal
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_11 && current_state == STATE_IDLE) {
    start_measurement_sequence();
    current_state = STATE_FIND_ZERO;
  }
}

// Redirect printf to UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}



void uart4_send_realtime(void)
{
    if (HAL_GetTick() - last_uart_tx_time < 100) return; // 100ms rate
    if (uart4_tx_busy) return;

    last_uart_tx_time = HAL_GetTick();

    int len = snprintf((char*)tx_buffer,
                       sizeof(tx_buffer),
                       "$F=%.3f,STEP=%u,DEPTH=%.2f\r\n",
                       force_kg,
                       stepper.step_count,
                       h_deep_current);

    uart4_tx_busy = 1;
    HAL_UART_Transmit_IT(&huart4, tx_buffer, len);
}


void led1_blink(void)
{
    if (HAL_GetTick() - last_led_toggle_time >= 500)
    {
        last_led_toggle_time = HAL_GetTick();
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
}


/**
  * @brief Parse binary protocol command
  */
void parse_binary_command(uint8_t *data, uint8_t len)
{
  if (len < 2) return;
  
  uint8_t cmd = data[0];
  uint8_t param_len = data[1];
  
  switch (cmd) {
    case CMD_TEST_MODE:
      if (param_len >= 1) {
        test_mode_enabled = data[2];
        send_ack_response();
      }
      break;
      
    case CMD_SET_DIGITAL_OUT:
      if (param_len >= 2) {
        uint8_t channel = data[2];
        uint8_t state = data[3];
        set_digital_output(channel, state);
        send_ack_response();
      }
      break;
      
    case CMD_SET_ANALOG_OUT:
      if (param_len >= 3) {
        uint8_t channel = data[2];
        uint16_t value = (data[3] << 8) | data[4];
        set_analog_output(channel, value);
        send_ack_response();
      }
      break;
      
    case CMD_SET_RELAY:
      if (param_len >= 2) {
        uint8_t relay = data[2];
        uint8_t state = data[3];
        set_relay(state);
        send_ack_response();
      }
      break;
      
    case CMD_SET_STEPPER:
      if (param_len >= 8) {
        int32_t position = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
        uint16_t speed = (data[6] << 8) | data[7];
        uint16_t accel = (data[8] << 8) | data[9];
        stepper_move_to_position(position, speed, accel);
        send_ack_response();
      }
      break;
      
    case CMD_SET_DC_MOTOR:
      if (param_len >= 3) {
        int16_t speed = (data[2] << 8) | data[3];
        uint8_t direction = data[4];
        dc_motor_set_speed(speed);
        send_ack_response();
      }
      break;
      
    case CMD_GET_STATUS:
      send_status_response();
      break;
      
    case CMD_REQUEST_REALTIME:
      send_realtime_data();
      break;
      
    case CMD_EMERGENCY_STOP:
      stepper_enable(0);
      dc_motor_stop();
      test_mode_enabled = 0;
      current_state = STATE_IDLE;
      send_ack_response();
      break;
      
    default:
      send_error_response(0x01); // Unknown command
      break;
  }
}

/**
  * @brief Parse ASCII protocol command
  */
void parse_ascii_command(char *cmd_str)
{
  char response[128];
  
  // Test mode command
  if (strncmp(cmd_str, "TEST:", 5) == 0) {
    test_mode_enabled = (cmd_str[5] == '1') ? 1 : 0;
    sprintf(response, "ACK:TEST=%d\n", test_mode_enabled);
    HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
  }
  
  // Digital output command
  else if (strncmp(cmd_str, "DOUT:", 5) == 0) {
    int channel, state;
    if (sscanf(cmd_str, "DOUT:%d,%d", &channel, &state) == 2) {
      set_digital_output(channel, state);
      sprintf(response, "ACK:DOUT=%d,%d\n", channel, state);
      HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
    }
  }
  
  // Analog output command
  else if (strncmp(cmd_str, "AOUT:", 5) == 0) {
    int channel, value;
    if (sscanf(cmd_str, "AOUT:%d,%d", &channel, &value) == 2) {
      set_analog_output(channel, value);
      sprintf(response, "ACK:AOUT=%d,%d\n", channel, value);
      HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
    }
  }
  
  // Relay command
  else if (strncmp(cmd_str, "RELAY:", 6) == 0) {
    int relay, state;
    if (sscanf(cmd_str, "RELAY:%d,%d", &relay, &state) == 2) {
      set_relay(state);
      sprintf(response, "ACK:RELAY=%d,%d\n", relay, state);
      HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
    }
  }
  
  // Stepper command
  else if (strncmp(cmd_str, "STEP:", 5) == 0) {
    int position, speed, accel;
    if (sscanf(cmd_str, "STEP:%d,%d,%d", &position, &speed, &accel) == 3) {
      stepper_move_to_position(position, speed, accel);
      sprintf(response, "ACK:STEP=%d,%d,%d\n", position, speed, accel);
      HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
    }
  }
  
  // DC motor command
  else if (strncmp(cmd_str, "DC:", 3) == 0) {
    int speed, direction;
    if (sscanf(cmd_str, "DC:%d,%d", &speed, &direction) == 2) {
      dc_motor_set_speed(speed);
      sprintf(response, "ACK:DC=%d,%d\n", speed, direction);
      HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
    }
  }
  
  // Status request
  else if (strcmp(cmd_str, "STATUS?\n") == 0) {
    send_status_response();
  }
  
  // Realtime request
  else if (strcmp(cmd_str, "RT?\n") == 0) {
    send_realtime_data();
  }
  
  // Tare command
  else if (strcmp(cmd_str, "TARE\n") == 0) {
    raw_force = 0;
    sprintf(response, "ACK:TARE\n");
    HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
  }
  
  // Stop command
  else if (strcmp(cmd_str, "STOP\n") == 0) {
    stepper_enable(0);
    dc_motor_stop();
    test_mode_enabled = 0;
    current_state = STATE_IDLE;
    sprintf(response, "ACK:STOP\n");
    HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
  }
  
  else {
    sprintf(response, "ERR:Unknown command\n");
    HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
  }
}

/**
  * @brief Parse JSON protocol command
  */
void parse_json_command(char *json_str)
{
  // Simplified JSON parsing - in practice you'd use a proper JSON parser
  // For now, we'll just send an ACK
  char response[] = "{\"response\":\"ACK\"}\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
}







/**
  * @brief Send real-time data (200ms interval)
  */
void send_realtime_data(void)
{
  if (uart4_tx_busy) return;
  
  int len;
  
  switch (current_protocol) {
    case PROTOCOL_BINARY:
      // Binary protocol - send hardware status structure
      hw_status.timestamp = HAL_GetTick();
      hw_status.load_cell_value = force_kg;
      hw_status.stepper_position = stepper.step_count;
      hw_status.stepper_target = stepper.target_steps;
      
      // Add a header byte for real-time data type
      tx_buffer[0] = RSP_REALTIME;
      memcpy(&tx_buffer[1], &hw_status, sizeof(hardware_status_t));
      
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, sizeof(hardware_status_t) + 1);
      break;
      
    case PROTOCOL_ASCII:
      // ASCII protocol - human readable format
      len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                    "RT:D=%.3f,F=%.3f,P=%d,S=%d,T=%.1f,A0=%d,A1=%d,A2=%d\n",
                    h_deep_current / 1000.0f,  // Depth in mm
                    force_kg,                    // Force in kg
                    stepper.step_count,          // Stepper position
                    dc_motor_running,             // DC motor status
                    hw_status.temperature,        // Temperature
                    hw_status.analog_inputs[0],   // First analog input
                    hw_status.analog_inputs[1],   // Second analog input
                    hw_status.analog_inputs[2]);  // Third analog input
      
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, len);
      break;
      
    case PROTOCOL_JSON:
      // JSON protocol - most flexible
      len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                    "{\"type\":\"realtime\",\"data\":{"
                    "\"depth\":%.3f,"
                    "\"force\":%.3f,"
                    "\"stepper\":%d,"
                    "\"dc_motor\":%d,"
                    "\"temp\":%.1f,"
                    "\"analog\":[%d,%d,%d]"
                    "}}\n",
                    h_deep_current / 1000.0f,
                    force_kg,
                    stepper.step_count,
                    dc_motor_running,
                    hw_status.temperature,
                    hw_status.analog_inputs[0],
                    hw_status.analog_inputs[1],
                    hw_status.analog_inputs[2]);
      
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, len);
      break;
  }
}






/**
  * @brief Send status response
  */
void send_status_response(void)
{
  if (uart4_tx_busy) return;
  
  int len;
  
  switch (current_protocol) {
    case PROTOCOL_BINARY:
      tx_buffer[0] = RSP_STATUS;
      memcpy(&tx_buffer[1], &hw_status, sizeof(hardware_status_t));
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, sizeof(hardware_status_t) + 1);
      break;
      
    case PROTOCOL_ASCII:
      len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                    "STATUS|T=%u|LC=%.3f|SP=%d|ST=%d|SS=%d|DC=%d|TEMP=%.1f\n",
                    HAL_GetTick(),
                    force_kg,
                    stepper.step_count,
                    stepper.target_steps,
                    stepper.speed,
                    dc_motor_running,
                    hw_status.temperature);
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, len);
      break;
      
    case PROTOCOL_JSON:
      len = snprintf((char*)tx_buffer, sizeof(tx_buffer),
                    "{\"type\":\"status\",\"timestamp\":%u,\"load_cell\":%.3f,"
                    "\"stepper_pos\":%d,\"stepper_target\":%d,\"temp\":%.1f}\n",
                    HAL_GetTick(), force_kg, stepper.step_count, 
                    stepper.target_steps, hw_status.temperature);
      uart4_tx_busy = 1;
      HAL_UART_Transmit_IT(&huart4, tx_buffer, len);
      break;
  }
}

/**
  * @brief Send acknowledgment response
  */
void send_ack_response(void)
{
  if (current_protocol == PROTOCOL_BINARY) {
    tx_buffer[0] = RSP_ACK;
    HAL_UART_Transmit(&huart4, tx_buffer, 1, 100);
  } else {
    char ack[] = "ACK\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)ack, strlen(ack), 100);
  }
}

/**
  * @brief Send error response
  */
void send_error_response(uint8_t error_code)
{
  if (current_protocol == PROTOCOL_BINARY) {
    tx_buffer[0] = RSP_ERROR;
    tx_buffer[1] = error_code;
    HAL_UART_Transmit(&huart4, tx_buffer, 2, 100);
  } else {
    char err[32];
    sprintf(err, "ERR:%d\n", error_code);
    HAL_UART_Transmit(&huart4, (uint8_t*)err, strlen(err), 100);
  }
}

/**
  * @brief Set digital output
  */
void set_digital_output(uint8_t channel, uint8_t state)
{
  if (channel > 7) return;
  
  if (state) {
    digital_output_state |= (1 << channel);
  } else {
    digital_output_state &= ~(1 << channel);
  }
  
  // Update GPIOs based on channel
  switch (channel) {
    case 0: HAL_GPIO_WritePin(EOUT1_GPIO_Port, EOUT1_Pin, state); break;
    case 1: HAL_GPIO_WritePin(EOUT2_GPIO_Port, EOUT2_Pin, state); break;
    case 2: HAL_GPIO_WritePin(EOUT3_GPIO_Port, EOUT3_Pin, state); break;
    case 3: HAL_GPIO_WritePin(EOUT4_GPIO_Port, EOUT4_Pin, state); break;
    case 4: HAL_GPIO_WritePin(EOUT5_GPIO_Port, EOUT5_Pin, state); break;
    case 5: HAL_GPIO_WritePin(EOUT6_GPIO_Port, EOUT6_Pin, state); break;
    case 6: HAL_GPIO_WritePin(EOUT7_GPIO_Port, EOUT7_Pin, state); break;
    case 7: HAL_GPIO_WritePin(EOUT8_GPIO_Port, EOUT8_Pin, state); break;
  }
  
  hw_status.digital_outputs = digital_output_state;
}

/**
  * @brief Set analog output (DAC)
  */
void set_analog_output(uint8_t channel, uint16_t value)
{
  if (channel > 2) return;
  
  // Limit to 12-bit
  value &= 0xFFF;
  
  hw_status.analog_outputs[channel] = value;
  
  // Set DAC value (using DAC1 channel 1 for simplicity)
  if (channel == 0) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  }
  // Add other DAC channels as needed
}

/**
  * @brief Set relay
  */
void set_relay(uint8_t state)
{
  relay_state = state;
  HAL_GPIO_WritePin(REL_OUT_GPIO_Port, REL_OUT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  hw_status.relay_states = state;
}

/**
  * @brief Read digital inputs
  */
void read_digital_inputs(void)
{
  uint32_t inputs = 0;
  
  // Read main digital inputs (29 total)
  inputs |= (HAL_GPIO_ReadPin(EXIN3_GPIO_Port, EXIN3_Pin) ? 1 : 0) << 0;
  inputs |= (HAL_GPIO_ReadPin(FASTIN1_GPIO_Port, FASTIN1_Pin) ? 1 : 0) << 1;
  inputs |= (HAL_GPIO_ReadPin(FASTIN2_GPIO_Port, FASTIN2_Pin) ? 1 : 0) << 2;
  inputs |= (HAL_GPIO_ReadPin(FASTIN3_GPIO_Port, FASTIN3_Pin) ? 1 : 0) << 3;
  inputs |= (HAL_GPIO_ReadPin(EXIN4_GPIO_Port, EXIN4_Pin) ? 1 : 0) << 4;
  inputs |= (HAL_GPIO_ReadPin(FASTIN4_GPIO_Port, FASTIN4_Pin) ? 1 : 0) << 5;
  
  hw_status.digital_inputs = inputs;
}




/* USER CODE END 4 */

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
