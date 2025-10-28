/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hlw8032.h"  // HLW8032电能计量模块驱动
#include "cmsis_os.h" // 添加FreeRTOS头文件
#include <stdio.h>
/* USER CODE END Includes */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Private includes */
extern osEventFlagsId_t pageSwitchFlagsHandle; // 声明外部事件标志句柄
/* USER CODE END Private includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
  // 直接处理按键1中断，避免调用HAL回调函数
  if (__HAL_GPIO_EXTI_GET_IT(KEY_1_Pin) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_1_Pin);
    
    // 按键1处理：切换LED状态
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
  }
  /* USER CODE END EXTI0_IRQn 0 */
  // 注释掉HAL回调函数调用
  // HAL_GPIO_EXTI_IRQHandler(KEY_1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
  // 直接处理按键2中断，避免调用HAL回调函数
  if (__HAL_GPIO_EXTI_GET_IT(KEY_2_Pin) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_2_Pin);
    
    uint32_t tick = HAL_GetTick();
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
    printf("[EXTI1] IRQ fired. tick=%lu, pin_state=%d\r\n", tick, pin_state);

    // 简单硬件层消抖：150ms窗口内忽略重复触发（提高响应）
    static uint32_t last_tick = 0;
    if ((tick - last_tick) >= 150)
    {
      // 可选视觉反馈
      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
      
      // 设置事件标志，通知FreeRTOS任务进行页面切换
      extern osEventFlagsId_t pageSwitchFlagsHandle;
      printf("[EXTI1] Using pageSwitchFlagsHandle=%p\r\n", pageSwitchFlagsHandle);
      if (pageSwitchFlagsHandle != NULL) {
        osEventFlagsSet(pageSwitchFlagsHandle, 0x01);
        printf("[EXTI1] Set flag 0x01 for KeyTask.\r\n");
      } else {
        printf("[EXTI1] pageSwitchFlagsHandle is NULL, skip set.\r\n");
      }
      
      last_tick = tick;
    }
    else
    {
      printf("[EXTI1] Debounce active, ignored. dt=%lu ms\r\n", (tick - last_tick));
    }
  }
  /* USER CODE END EXTI1_IRQn 0 */
  // 注释掉HAL回调函数调用
  // HAL_GPIO_EXTI_IRQHandler(KEY_2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  // HLW8032 已迁移至 USART2，此处不做 HLW8032 处理。
  // 若启用 USART3 接收中断，以下代码仅清除 RXNE/ORE 标志以避免堆积。
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
  {
      volatile uint8_t data = (uint8_t)huart3.Instance->DR;  // 读取数据清除RXNE
      (void)data;
      // 不再调用 HLW8032_RxCallback
  }

  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET)
  {
      __HAL_UART_CLEAR_OREFLAG(&huart3);  // 清除过载错误标志
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  // HLW8032数据接收处理（基于寄存器直接操作，喂入解析器）
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
  {
      uint8_t data = (uint8_t)huart2.Instance->DR;  // 读取数据（自动清除RXNE标志）
      hlw8032_feed_byte(data);  // 使用HLW8032解析器（BSP版本）
      return;  // 处理完毕，直接返回
  }

  // 处理可能的错误中断
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET)
  {
      __HAL_UART_CLEAR_OREFLAG(&huart2);  // 清除过载错误标志
  }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */