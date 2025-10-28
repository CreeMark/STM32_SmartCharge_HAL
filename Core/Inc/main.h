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
#define FAN_PWM_Pin GPIO_PIN_0
#define FAN_PWM_GPIO_Port GPIOC
#define RELAY_IN1_Pin GPIO_PIN_1
#define RELAY_IN1_GPIO_Port GPIOC
#define RELAY_IN2_Pin GPIO_PIN_2
#define RELAY_IN2_GPIO_Port GPIOC
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_1_EXTI_IRQn EXTI0_IRQn
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOA
#define KEY_2_EXTI_IRQn EXTI1_IRQn
#define HLW8032_RE_Pin GPIO_PIN_4
#define HLW8032_RE_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define LCD_RES_Pin GPIO_PIN_0
#define LCD_RES_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_1
#define LCD_DC_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_10
#define LCD_CS_GPIO_Port GPIOB
#define LCD_BLK_Pin GPIO_PIN_11
#define LCD_BLK_GPIO_Port GPIOB
#define LIGHT_SENSOR_DO_Pin GPIO_PIN_15
#define LIGHT_SENSOR_DO_GPIO_Port GPIOB
#define HLW8032_PF_Pin GPIO_PIN_6
#define HLW8032_PF_GPIO_Port GPIOA
#define SR501_IN_Pin GPIO_PIN_12
#define SR501_IN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

// DHT11温湿度传感器引脚配置说明：
// 当前使用: PB4 (已禁用JTAG，PB3/PB4可用作GPIO)
// 注意: PB4原为JTAG-TRST，需确保通过__HAL_AFIO_REMAP_SWJ_NOJTAG()禁用JTAG
// 历史变更: PD2→PB3→PB4 (PD2与UART5冲突，PB3改为PB4)
// #define DHT11_DATA_Pin       GPIO_PIN_2
// #define DHT11_DATA_GPIO_Port GPIOD

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
