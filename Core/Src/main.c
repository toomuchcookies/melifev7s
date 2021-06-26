/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ringbuffer.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct Config {
	float Max_RSpeed;
	float Max_LSpeed;
	float Acc;
	float Tar_RSpeed;
	float Tar_LSpeed;
} myconfig;

int sgn(double x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motionWD */
osThreadId_t motionWDHandle;
const osThreadAttr_t motionWD_attributes = {
  .name = "motionWD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* USER CODE BEGIN PV */
ringbuf_uint8t uart_rx_rb;
#define uart_tmp_rx_buffer_size 128
uint8_t uart_tmp_rx_buffer[uart_tmp_rx_buffer_size];
#define uart_rx_buffer_size 128
uint8_t uart_rx_buffer[uart_rx_buffer_size];
#define command_buffer_size 128
uint8_t command_buffer[command_buffer_size];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void motion_wd(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		for (int i=0;i<Size;i++)
		{
			if (uart_tmp_rx_buffer[i] == '\x0D')
			{
				int c=0;
				while( !rb_isempty(&uart_rx_rb) )
				{
					uint8_t *tx_byte_ptr = rb_get(&uart_rx_rb);
					command_buffer[c] = *tx_byte_ptr;
					c++;
				}
				uint8_t test[4] = "Test";
				if (strncmp(command_buffer, test, c) == 0)
				{
					HAL_GPIO_TogglePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin);
				}
				command_buffer[c] = '\r';
				c++;
				command_buffer[c] = '\n';
				c++;
				HAL_UART_Transmit(&huart1, command_buffer, c, 1000);
			}
			else
			{
				rb_put(&uart_rx_rb, uart_tmp_rx_buffer[i]);
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_tmp_rx_buffer, uart_tmp_rx_buffer_size);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
  myconfig.Acc = 5;
  myconfig.Max_LSpeed = 100;
  myconfig.Max_RSpeed = 100;
  myconfig.Tar_LSpeed = 0;
  myconfig.Tar_RSpeed = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  rb_init(&uart_rx_rb, uart_rx_buffer, uart_rx_buffer_size);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_tmp_rx_buffer, uart_tmp_rx_buffer_size);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  HAL_UART_Transmit(&huart1, "Started\x0A\x0D", 9, 1000);

  HAL_ADC_Start(&hadc1);
  HAL_GPIO_WritePin(GPIOE_PE05_SLEEP_GPIO_Port, GPIOE_PE05_SLEEP_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motionWD */
  motionWDHandle = osThreadNew(motion_wd, NULL, &motionWD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIOE_PE05_SLEEP_Pin|GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin|GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIOA_PA08_TURBINE_ENABLE_Pin|GPIOA_PA11_BEEPER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIOE_PE03_MOT_L_GROUND_Pin GPIOE_PE04_BUG_RIGHT_MAGNET_Pin GPIOE_PE06_IR_RIGHT_Pin GPIOE_PE08_ENC_R_Pin
                           GPIOE_PE10_IR_FRONTLEFT_Pin GPIOE_PE12_CONTACT_BUMPER_R_Pin GPIOE_PE00_BUG_LEFT_MAGNET_Pin */
  GPIO_InitStruct.Pin = GPIOE_PE03_MOT_L_GROUND_Pin|GPIOE_PE04_BUG_RIGHT_MAGNET_Pin|GPIOE_PE06_IR_RIGHT_Pin|GPIOE_PE08_ENC_R_Pin
                          |GPIOE_PE10_IR_FRONTLEFT_Pin|GPIOE_PE12_CONTACT_BUMPER_R_Pin|GPIOE_PE00_BUG_LEFT_MAGNET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOE_PE05_SLEEP_Pin GPIOE_PE13_MOT_R_PHASE_Pin */
  GPIO_InitStruct.Pin = GPIOE_PE05_SLEEP_Pin|GPIOE_PE13_MOT_R_PHASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOB_PB10_GROUND_SENSORS_TX_Pin GPIOB_PB13_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = GPIOB_PB10_GROUND_SENSORS_TX_Pin|GPIOB_PB13_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOB_PB11_IR_FRONTRIGHT_Pin GPIOB_PB05_CONTACT_BUMPER_L_Pin */
  GPIO_InitStruct.Pin = GPIOB_PB11_IR_FRONTRIGHT_Pin|GPIOB_PB05_CONTACT_BUMPER_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin GPIOB_PB07_MOT_L_PHASE_Pin */
  GPIO_InitStruct.Pin = GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin|GPIOB_PB07_MOT_L_PHASE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOD_PD10_MOT_R_GROUND_Pin GPIOD_PD13_IR_LEFT_Pin GPIOD_PD02_ENC_CASTER_WHEEL_Pin GPIOD_PD03_ENC_L_Pin
                           GPIOD_PD04_IR_REAR_Pin */
  GPIO_InitStruct.Pin = GPIOD_PD10_MOT_R_GROUND_Pin|GPIOD_PD13_IR_LEFT_Pin|GPIOD_PD02_ENC_CASTER_WHEEL_Pin|GPIOD_PD03_ENC_L_Pin
                          |GPIOD_PD04_IR_REAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin */
  GPIO_InitStruct.Pin = GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin */
  GPIO_InitStruct.Pin = GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOA_PA08_TURBINE_ENABLE_Pin GPIOA_PA11_BEEPER_Pin */
  GPIO_InitStruct.Pin = GPIOA_PA08_TURBINE_ENABLE_Pin|GPIOA_PA11_BEEPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void setPW(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
	 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	 TIM_OC_InitTypeDef sConfigOC;
	 timer.Init.Period = period; // set the period duration
	 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
	 sConfigOC.OCMode = TIM_OCMODE_PWM1;
	 sConfigOC.Pulse = pulse; // set the pulse duration
	 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  int myadc;
	  int increment;
	  _Bool vacuum = 0;
	  _Bool butpressed = 0;
	  int green = 0;

  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB_PB14_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA_PA08_TURBINE_ENABLE_GPIO_Port, GPIOA_PA08_TURBINE_ENABLE_Pin, GPIO_PIN_RESET);

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // start the pwm
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // start the pwm
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // start the pwm

  setPW(htim1, TIM_CHANNEL_2, 100, 0);
  //htim1.Instance->CCR2 = 0; // 50% duty cycle
  //htim3.Instance->CCR1 = 0;
  setPW(htim3, TIM_CHANNEL_1, 100, 0);
  //htim3.Instance->CCR3 = 0;
  setPW(htim3, TIM_CHANNEL_3, 100, 0);
  /* Infinite loop */
  for(;;)
  {
	  if (green > 95) increment = -1;
	  if (green < 5) increment = 1;

	  green += increment;
	  setPW(htim1, TIM_CHANNEL_2, 100, green);

	  //htim1.Instance->CCR2 += increment;

	  myadc = HAL_ADC_GetValue(&hadc1);

	  if (myadc < 100){
		  if (!butpressed){
			  vacuum = !vacuum;
			  butpressed = 1;
			  if (vacuum){
				  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_SET);
				  // HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOB_PB14_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA_PA08_TURBINE_ENABLE_GPIO_Port, GPIOA_PA08_TURBINE_ENABLE_Pin, GPIO_PIN_SET);
			  }else{
				  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);
				  // HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_SET);

				  HAL_GPIO_WritePin(GPIOB_PB14_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOB_PB14_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA_PA08_TURBINE_ENABLE_GPIO_Port, GPIOA_PA08_TURBINE_ENABLE_Pin, GPIO_PIN_RESET);
			  }}
	  }else{
		  butpressed = 0;
	  }

	  if (HAL_GPIO_ReadPin(GPIOE_PE12_CONTACT_BUMPER_R_GPIO_Port, GPIOE_PE12_CONTACT_BUMPER_R_Pin)){
		  HAL_GPIO_WritePin(GPIOB_PB07_MOT_L_PHASE_GPIO_Port, GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_SET);
		  myconfig.Tar_LSpeed = 80;
	  } else {
		  // HAL_GPIO_WritePin(GPIOB_PB07_MOT_L_PHASE_GPIO_Port, GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_RESET);
		  //htim3.Instance->CCR3 = 0;
		  myconfig.Tar_LSpeed = 0;
	  }

	  if (HAL_GPIO_ReadPin(GPIOB_PB05_CONTACT_BUMPER_L_GPIO_Port, GPIOB_PB05_CONTACT_BUMPER_L_Pin)){
		  HAL_GPIO_WritePin(GPIOE_PE13_MOT_R_PHASE_GPIO_Port, GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_RESET);
		  myconfig.Tar_RSpeed = 80;
	  } else {
		  // HAL_GPIO_WritePin(GPIOE_PE13_MOT_R_PHASE_GPIO_Port, GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_SET);
		  myconfig.Tar_RSpeed = 0;
	  }

	  //HAL_Delay(10);

	  osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motion_wd */
/**
* @brief Function implementing the motionWD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motion_wd */
void motion_wd(void *argument)
{
  /* USER CODE BEGIN motion_wd */
	float Ldiff;
	float Rdiff;
  /* Infinite loop */
  for(;;)
  {
	  Ldiff = myconfig.Tar_LSpeed - htim3.Instance->CCR3;
	  Rdiff = myconfig.Tar_RSpeed - htim3.Instance->CCR1;
	  setPW(htim3, TIM_CHANNEL_3, 100, htim3.Instance->CCR3 + (int)(sgn(Ldiff)*myconfig.Acc));
	  setPW(htim3, TIM_CHANNEL_1, 100, htim3.Instance->CCR1 + (int)(sgn(Rdiff)*myconfig.Acc));
    osDelay(50);
  }
  /* USER CODE END motion_wd */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
