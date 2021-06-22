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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int myadc;
  _Bool vacuum = 0;
  _Bool butpressed = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_GPIO_WritePin(GPIOE_PE05_SLEEP_GPIO_Port, GPIOE_PE05_SLEEP_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB_PB14_TOURBINE_ENABLE_GPIO_Port, GPIOB_PB14_TOURBINE_ENABLE_Pin, GPIO_PIN_RESET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  myadc = HAL_ADC_GetValue(&hadc1);

	  /*
	  if (HAL_GPIO_ReadPin(GPIOE_PE03_MOT_L_GROUND_GPIO_Port, GPIOE_PE03_MOT_L_GROUND_Pin) & HAL_GPIO_ReadPin(GPIOD_PD10_MOT_R_GROUND_GPIO_Port, GPIOD_PD10_MOT_R_GROUND_Pin)){
		  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_SET);
	  }
	  else {
		  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_RESET);
	  }*/

	  /* if (HAL_GPIO_ReadPin(GPIOE_PE12_CONTACT_BUMPER_R_GPIO_Port, GPIOE_PE12_CONTACT_BUMPER_R_Pin)){
		  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_RESET);
	  }
	  else {
		  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_SET);
	  } */

	  /*if (HAL_GPIO_ReadPin(GPIOE_PE03_MOT_L_GROUND_GPIO_Port, GPIOE_PE03_MOT_L_GROUND_Pin)){
	  }else{
		  HAL_GPIO_WritePin(GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
	  }
	  if (HAL_GPIO_ReadPin(GPIOD_PD10_MOT_R_GROUND_GPIO_Port, GPIOD_PD10_MOT_R_GROUND_Pin)){
	  }else{
		  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
	  }*/

	  //HAL_GPIO_WritePin(GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(GPIOB_PB14_TOURBINE_ENABLE_GPIO_Port, GPIOB_PB14_TOURBINE_ENABLE_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
	  if (myadc < 100){
		  if (!butpressed){
			  vacuum = !vacuum;
			  butpressed = 1;
			  if (vacuum){
				  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOB_PB14_TOURBINE_ENABLE_GPIO_Port, GPIOB_PB14_TOURBINE_ENABLE_Pin, GPIO_PIN_SET);
			  }else{
				  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port, GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin, GPIO_PIN_SET);

				  HAL_GPIO_WritePin(GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port, GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOB_PB14_TOURBINE_ENABLE_GPIO_Port, GPIOB_PB14_TOURBINE_ENABLE_Pin, GPIO_PIN_RESET);
			  }}
	  }else{
		  butpressed = 0;
	  }

	  if (HAL_GPIO_ReadPin(GPIOE_PE12_CONTACT_BUMPER_R_GPIO_Port, GPIOE_PE12_CONTACT_BUMPER_R_Pin)){
		  HAL_GPIO_WritePin(GPIOB_PB07_MOT_L_PHASE_GPIO_Port, GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC_PC08_MOT_L_ENABLE_GPIO_Port, GPIOC_PC08_MOT_L_ENABLE_Pin, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(GPIOB_PB07_MOT_L_PHASE_GPIO_Port, GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC_PC08_MOT_L_ENABLE_GPIO_Port, GPIOC_PC08_MOT_L_ENABLE_Pin, GPIO_PIN_RESET);
	  }
	  //HAL_GPIO_WritePin(GPIOE_PE13_MOT_R_PHASE_GPIO_Port, GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_SET);
	  //TIM3->CCR1 = (myadc);

	  if (HAL_GPIO_ReadPin(GPIOB_PB05_CONTACT_BUMPER_L_GPIO_Port, GPIOB_PB05_CONTACT_BUMPER_L_Pin)){
		  HAL_GPIO_WritePin(GPIOE_PE13_MOT_R_PHASE_GPIO_Port, GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC_PC06_MOT_R_ENABLE_GPIO_Port, GPIOC_PC06_MOT_R_ENABLE_Pin, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(GPIOE_PE13_MOT_R_PHASE_GPIO_Port, GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC_PC06_MOT_R_ENABLE_GPIO_Port, GPIOC_PC06_MOT_R_ENABLE_Pin, GPIO_PIN_RESET);
	  }

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOE, GPIOE_PE05_SLEEP_Pin|GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin|GPIOE_PE13_MOT_R_PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIOB_PB14_TOURBINE_ENABLE_Pin|GPIOB_PB07_MOT_L_PHASE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port, GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIOC_PC06_MOT_R_ENABLE_Pin|GPIOC_PC08_MOT_L_ENABLE_Pin|GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin|GPIOA_PA11_BEEPER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIOE_PE03_MOT_L_GROUND_Pin GPIOE_PE04_BUG_RIGHT_MAGNET_Pin GPIOE_PE06_IR_RIGHT_Pin GPIOE_PE08_ENC_R_Pin
                           GPIOE_PE10_IR_FRONTLEFT_Pin GPIOE_PE12_CONTACT_BUMPER_R_Pin GPIOE_PE00_BUG_LEFT_MAGNET_Pin */
  GPIO_InitStruct.Pin = GPIOE_PE03_MOT_L_GROUND_Pin|GPIOE_PE04_BUG_RIGHT_MAGNET_Pin|GPIOE_PE06_IR_RIGHT_Pin|GPIOE_PE08_ENC_R_Pin
                          |GPIOE_PE10_IR_FRONTLEFT_Pin|GPIOE_PE12_CONTACT_BUMPER_R_Pin|GPIOE_PE00_BUG_LEFT_MAGNET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOE_PE05_SLEEP_Pin GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin GPIOE_PE13_MOT_R_PHASE_Pin */
  GPIO_InitStruct.Pin = GPIOE_PE05_SLEEP_Pin|GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin|GPIOE_PE13_MOT_R_PHASE_Pin;
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

  /*Configure GPIO pins : GPIOB_PB14_TOURBINE_ENABLE_Pin GPIOB_PB07_MOT_L_PHASE_Pin */
  GPIO_InitStruct.Pin = GPIOB_PB14_TOURBINE_ENABLE_Pin|GPIOB_PB07_MOT_L_PHASE_Pin;
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

  /*Configure GPIO pins : GPIOC_PC06_MOT_R_ENABLE_Pin GPIOC_PC08_MOT_L_ENABLE_Pin GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin */
  GPIO_InitStruct.Pin = GPIOC_PC06_MOT_R_ENABLE_Pin|GPIOC_PC08_MOT_L_ENABLE_Pin|GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin GPIOA_PA11_BEEPER_Pin */
  GPIO_InitStruct.Pin = GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin|GPIOA_PA11_BEEPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIOA_PA09_USART1_TX_Pin */
  GPIO_InitStruct.Pin = GPIOA_PA09_USART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA_PA09_USART1_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIOA_PA10_USART1_RX_Pin */
  GPIO_InitStruct.Pin = GPIOA_PA10_USART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA_PA10_USART1_RX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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