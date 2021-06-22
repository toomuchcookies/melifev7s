/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define GPIOE_PE03_MOT_L_GROUND_Pin GPIO_PIN_3
#define GPIOE_PE03_MOT_L_GROUND_GPIO_Port GPIOE
#define GPIOE_PE04_BUG_RIGHT_MAGNET_Pin GPIO_PIN_4
#define GPIOE_PE04_BUG_RIGHT_MAGNET_GPIO_Port GPIOE
#define GPIOE_PE05_SLEEP_Pin GPIO_PIN_5
#define GPIOE_PE05_SLEEP_GPIO_Port GPIOE
#define GPIOE_PE06_IR_RIGHT_Pin GPIO_PIN_6
#define GPIOE_PE06_IR_RIGHT_GPIO_Port GPIOE
#define GPIOC_PC00_GROUND_SENSOR_LEFT_Pin GPIO_PIN_0
#define GPIOC_PC00_GROUND_SENSOR_LEFT_GPIO_Port GPIOC
#define GPIOC_PC01_IRBUMPER_LEFT_WALL_Pin GPIO_PIN_1
#define GPIOC_PC01_IRBUMPER_LEFT_WALL_GPIO_Port GPIOC
#define GPIOC_PC02_GROUND_SENSOR_FRONTLEFT_Pin GPIO_PIN_2
#define GPIOC_PC02_GROUND_SENSOR_FRONTLEFT_GPIO_Port GPIOC
#define GPIOC_PC03_IRBUMPER_RIGHT_Pin GPIO_PIN_3
#define GPIOC_PC03_IRBUMPER_RIGHT_GPIO_Port GPIOC
#define GPIOA_PA00_TOUCH_BUTTON_Pin GPIO_PIN_0
#define GPIOA_PA00_TOUCH_BUTTON_GPIO_Port GPIOA
#define GPIOA_PA01_CHARGER_VOLTAGE_Pin GPIO_PIN_1
#define GPIOA_PA01_CHARGER_VOLTAGE_GPIO_Port GPIOA
#define GPIOA_PA02_BATT_VOLTAGE_Pin GPIO_PIN_2
#define GPIOA_PA02_BATT_VOLTAGE_GPIO_Port GPIOA
#define GPIOA_PA03_GROUND_SENSOR_RIGHT_Pin GPIO_PIN_3
#define GPIOA_PA03_GROUND_SENSOR_RIGHT_GPIO_Port GPIOA
#define GPIOA_PA04_TOURBINE_CURRENT_Pin GPIO_PIN_4
#define GPIOA_PA04_TOURBINE_CURRENT_GPIO_Port GPIOA
#define GPIOA_PA05_IRBUMPER_CENTER_Pin GPIO_PIN_5
#define GPIOA_PA05_IRBUMPER_CENTER_GPIO_Port GPIOA
#define GPIOA_PA06_BRUSH_CURRENT_Pin GPIO_PIN_6
#define GPIOA_PA06_BRUSH_CURRENT_GPIO_Port GPIOA
#define GPIOA_PA07_BATT_CURRENT_Pin GPIO_PIN_7
#define GPIOA_PA07_BATT_CURRENT_GPIO_Port GPIOA
#define GPIOC_PC04_GROUND_SENSOR_FRONTRIGHT_Pin GPIO_PIN_4
#define GPIOC_PC04_GROUND_SENSOR_FRONTRIGHT_GPIO_Port GPIOC
#define GPIOB_PB00_IRBUMPER_LEFT_Pin GPIO_PIN_0
#define GPIOB_PB00_IRBUMPER_LEFT_GPIO_Port GPIOB
#define GPIOE_PE08_ENC_R_Pin GPIO_PIN_8
#define GPIOE_PE08_ENC_R_GPIO_Port GPIOE
#define GPIOE_PE10_IR_FRONTLEFT_Pin GPIO_PIN_10
#define GPIOE_PE10_IR_FRONTLEFT_GPIO_Port GPIOE
#define GPIOE_PE11_TOUCH_BUTTON_COLOR_2_Pin GPIO_PIN_11
#define GPIOE_PE11_TOUCH_BUTTON_COLOR_2_GPIO_Port GPIOE
#define GPIOE_PE12_CONTACT_BUMPER_R_Pin GPIO_PIN_12
#define GPIOE_PE12_CONTACT_BUMPER_R_GPIO_Port GPIOE
#define GPIOE_PE13_MOT_R_PHASE_Pin GPIO_PIN_13
#define GPIOE_PE13_MOT_R_PHASE_GPIO_Port GPIOE
#define GPIOB_PB10_GROUND_SENSORS_TX_Pin GPIO_PIN_10
#define GPIOB_PB10_GROUND_SENSORS_TX_GPIO_Port GPIOB
#define GPIOB_PB11_IR_FRONTRIGHT_Pin GPIO_PIN_11
#define GPIOB_PB11_IR_FRONTRIGHT_GPIO_Port GPIOB
#define GPIOB_PB13_SPI2_SCK_Pin GPIO_PIN_13
#define GPIOB_PB13_SPI2_SCK_GPIO_Port GPIOB
#define GPIOB_PB14_TOURBINE_ENABLE_Pin GPIO_PIN_14
#define GPIOB_PB14_TOURBINE_ENABLE_GPIO_Port GPIOB
#define GPIOD_PD10_MOT_R_GROUND_Pin GPIO_PIN_10
#define GPIOD_PD10_MOT_R_GROUND_GPIO_Port GPIOD
#define GPIOD_PD11_TOUCH_BUTTON_COLOR_1_Pin GPIO_PIN_11
#define GPIOD_PD11_TOUCH_BUTTON_COLOR_1_GPIO_Port GPIOD
#define GPIOD_PD13_IR_LEFT_Pin GPIO_PIN_13
#define GPIOD_PD13_IR_LEFT_GPIO_Port GPIOD
#define GPIOC_PC06_MOT_R_ENABLE_Pin GPIO_PIN_6
#define GPIOC_PC06_MOT_R_ENABLE_GPIO_Port GPIOC
#define GPIOC_PC08_MOT_L_ENABLE_Pin GPIO_PIN_8
#define GPIOC_PC08_MOT_L_ENABLE_GPIO_Port GPIOC
#define GPIOC_PC09_MAIN_BRUSH_ENABLE_Pin GPIO_PIN_9
#define GPIOC_PC09_MAIN_BRUSH_ENABLE_GPIO_Port GPIOC
#define GPIOA_PA08_SIDE_BRUSH_ENABLE_Pin GPIO_PIN_8
#define GPIOA_PA08_SIDE_BRUSH_ENABLE_GPIO_Port GPIOA
#define GPIOA_PA09_USART1_TX_Pin GPIO_PIN_9
#define GPIOA_PA09_USART1_TX_GPIO_Port GPIOA
#define GPIOA_PA10_USART1_RX_Pin GPIO_PIN_10
#define GPIOA_PA10_USART1_RX_GPIO_Port GPIOA
#define GPIOA_PA11_BEEPER_Pin GPIO_PIN_11
#define GPIOA_PA11_BEEPER_GPIO_Port GPIOA
#define GPIOD_PD02_ENC_CASTER_WHEEL_Pin GPIO_PIN_2
#define GPIOD_PD02_ENC_CASTER_WHEEL_GPIO_Port GPIOD
#define GPIOD_PD03_ENC_L_Pin GPIO_PIN_3
#define GPIOD_PD03_ENC_L_GPIO_Port GPIOD
#define GPIOD_PD04_IR_REAR_Pin GPIO_PIN_4
#define GPIOD_PD04_IR_REAR_GPIO_Port GPIOD
#define GPIOB_PB05_CONTACT_BUMPER_L_Pin GPIO_PIN_5
#define GPIOB_PB05_CONTACT_BUMPER_L_GPIO_Port GPIOB
#define GPIOB_PB07_MOT_L_PHASE_Pin GPIO_PIN_7
#define GPIOB_PB07_MOT_L_PHASE_GPIO_Port GPIOB
#define GPIOE_PE00_BUG_LEFT_MAGNET_Pin GPIO_PIN_0
#define GPIOE_PE00_BUG_LEFT_MAGNET_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
