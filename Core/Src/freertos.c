/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "bsp_SR501.h"
#include "bsp_LightSensor.h"
#include "bsp_fan.h"
#include "bsp_relay.h"
#include "hlw8032.h"
#include "lcd.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
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
/* USER CODE BEGIN PV */
// 全局传感器数据存储变量，用于实现数据收集与页面显示分离
static uint8_t g_temperature = 0;     // 温度值
static uint8_t g_humidity = 0;        // 湿度值
static uint8_t g_light_percent = 0;   // 光照强度百分比
static SR501_Status_t g_motion_status = SR501_NO_MOTION;  // 人体感应状态
static uint8_t g_fan_status = 0;      // 风扇状态标志
static HLW8032_Data g_power_metrics = {0};  // 电能计量数据

// 添加页面管理变量
uint8_t current_page = 0;  // 0=电能计量页面, 1=环境监测页面

// 添加互斥锁和事件标志定义
osMutexId sensorDataMutexHandle;
osEventFlagsId_t pageSwitchFlagsHandle;  // 页面切换事件标志
/* USER CODE END PV */
/* Definitions for PowerMeterTask */
osThreadId PowerMeterTaskHandle;
const osThreadAttr_t PowerMeterTask_attributes = {
  .name = "PowerMeterTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for SensorTask */
osThreadId SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for DisplayTask */
osThreadId DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for ControlTask */
osThreadId ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for KeyTask */
osThreadId KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// 页面绘制函数声明
void Draw_PowerMeter_Page(void);
void Draw_EnvMonitor_Page(void);
void Update_EnvMonitor_Display(void);
void Update_PowerMeter_Display(void);
/* USER CODE END FunctionPrototypes */

void StartTask03(void *argument);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   printf("[FreeRTOS] Stack overflow in task: %s\r\n", pcTaskName);
   // 进入错误指示：红灯快闪，停机等待复位
   taskDISABLE_INTERRUPTS();
   for(;;)
   {
     HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
     // 简单忙等待延时，避免依赖SysTick
     for(volatile uint32_t d=0; d<200000; ++d) { __NOP(); }
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  // 创建传感器数据互斥锁
  sensorDataMutexHandle = osMutexNew(NULL);
  if (sensorDataMutexHandle == NULL) {
    printf("[FreeRTOS] Failed to create sensorDataMutexHandle\r\n");
  }
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

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  // 创建页面切换事件标志
  pageSwitchFlagsHandle = osEventFlagsNew(NULL);
  if (pageSwitchFlagsHandle == NULL) {
    printf("[FreeRTOS] Failed to create pageSwitchFlagsHandle\r\n");
  }
  /* USER CODE END RTOS_EVENTS */

  /* Create the thread(s) */
  /* creation of PowerMeterTask */
  PowerMeterTaskHandle = osThreadNew(StartTask03, NULL, &PowerMeterTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask01, NULL, &SensorTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartTask02, NULL, &DisplayTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartTask04, NULL, &ControlTask_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(StartTask05, NULL, &KeyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask03 */
/**
  * @brief  Function implementing the PowerMeterTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  // 初始化时就打开HLW8032的RE继电器
  HAL_GPIO_WritePin(HLW8032_RE_GPIO_Port, HLW8032_RE_Pin, GPIO_PIN_SET);  // 输出高电平，继电器吸合
  printf("[HLW8032] RE Relay ON - Output socket powered\r\n");
  
  for(;;)
  {
    // 等待定时器 - 提高更新频率
    osDelay(100);  // 100ms执行一次
    
    // 使用BSP解析器：检测新帧并读取
    if (hlw8032_frame_ready()) {
      HLW8032_Data d;
      hlw8032_get_data(&d);
      // 更新全局数据（保持原有显示结构）
      g_power_metrics.voltage = d.voltage;
      g_power_metrics.current = d.current;
      g_power_metrics.active_power = d.active_power;
      g_power_metrics.power_factor = d.power_factor;
      g_power_metrics.energy = d.energy;
      
      // 转换为显示单位
      uint32_t voltage_mV = (uint32_t)(d.voltage * 1000.0f);
      uint32_t current_mA = (uint32_t)(d.current * 1000.0f);
      uint32_t power_mW = (uint32_t)(d.active_power * 1000.0f);
      uint16_t pf_percent = (uint16_t)(d.power_factor * 100.0f);
      
      // 串口输出电能数据
      printf("[HLW8032] U=%d.%03dV, I=%d.%03dA, P=%d.%03dW, PF=%d%%, E=%.6fkWh\r\n",
             voltage_mV / 1000, voltage_mV % 1000,
             current_mA / 1000, current_mA % 1000,
             power_mW / 1000, power_mW % 1000,
             pf_percent,
             d.energy);
    }
    
    // 如果数据全为0，输出额外提示
    if (g_power_metrics.voltage == 0.0f && g_power_metrics.current == 0.0f && g_power_metrics.active_power == 0.0f)
    {
      static uint8_t zero_warning_count = 0;
      if (++zero_warning_count >= 5)  // 每5秒提示一次
      {
        printf("[HLW8032 Warning] All data is ZERO! Possible reasons:\r\n");
        printf("  1. Module needs warm-up time (wait 10-30s after 220V power on)\r\n");
        printf("  2. No load connected to output socket (voltage should still show ~220V)\r\n");
        printf("  3. Check if 220V AC input plug is properly inserted\r\n");
        printf("  4. Verify USART2 RX connection (PA3) and baud rate (4800bps)\r\n");
        printf("  5. Check HLW8032 RE pin control (should be high to power module)\r\n");
        zero_warning_count = 0;
      }
    }
    
    osDelay(100);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
  uint8_t dht11_first_read = 1;  // Flag for first read
  uint8_t retry_count = 0;  // 重试计数器
  SR501_Status_t last_status = SR501_NO_MOTION;  // 保存上次状态，避免重复输出
  uint8_t first_read = 1;  // 标记是否首次读取
  uint8_t device_status = 0;  // 设备状态 (0=关闭, 1=开启)
  
  for(;;)
  {
    // 等待定时器 - 减少延迟以提高实时性
    osDelay(100);  // 100ms执行一次
    
    // 每10个周期读取一次DHT11（1秒）
    static uint8_t dht11_counter = 0;
    if (++dht11_counter >= 10)
    {
      dht11_counter = 0;
      
      uint8_t temperature = 0;  // 温度值
      uint8_t humidity = 0;     // 湿度值
      
      // Add extra message for first read
      if (dht11_first_read)
      {
        printf("[DHT11] First data read attempt...\r\n");
        dht11_first_read = 0;
      }
      
      u8 result = DHT11_Read_Data(&temperature, &humidity);
      
      // 获取互斥锁
      if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
      {
        if(result == 0)  // 0表示读取成功，1表示失败
        {
          retry_count = 0;  // 成功后重置重试计数
          
          // 更新全局数据
          g_temperature = temperature;
          g_humidity = humidity;
          
          // 串口输出
          printf("[DHT11] Temperature: %dC, Humidity: %d%%\r\n", 
                 temperature, 
                 humidity);
        }
        else
        {
          retry_count++;
          printf("[DHT11] Read Failed (retry %d/3) - Retrying next cycle\r\n", retry_count);
          
          // 只在多次失败后才显示错误，避免屏幕闪烁
          if(retry_count >= 2)
          {
            // 更新全局数据为错误状态
            g_temperature = 0;
            g_humidity = 0;
          }
        }
        
        // 释放互斥锁
        osMutexRelease(sensorDataMutexHandle);
      }
    }
    
    // 每2个周期读取一次HC-SR501（0.2秒）
    static uint8_t sr501_counter = 0;
    if (++sr501_counter >= 2)
    {
      sr501_counter = 0;
      
      // 读取HC-SR501传感器状态
      SR501_Status_t current_status = SR501_Read(SR501_IN_GPIO_Port, SR501_IN_Pin);
      
      // 获取互斥锁
      if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
      {
        // 更新全局数据
        g_motion_status = current_status;
        
        // 首次读取：输出初始状态
        if (first_read)
        {
          first_read = 0;
          if (current_status == SR501_MOTION_DETECTED)
          {
            printf("[SR501] Initial State: Motion Detected!\r\n");
          }
          else
          {
            printf("[SR501] Initial State: No Motion.\r\n");
          }
          last_status = current_status;
        }
        // 后续检测：只在状态变化时输出
        else if (current_status != last_status)
        {
          if (current_status == SR501_MOTION_DETECTED)
          {
            printf("[SR501] Motion Detected! Human presence confirmed.\r\n");
          }
          else
          {
            printf("[SR501] No Motion - Area clear.\r\n");
          }
          last_status = current_status;  // 更新状态
        }
        
        // 释放互斥锁
        osMutexRelease(sensorDataMutexHandle);
      }
    }
    
    // 每5个周期读取一次光敏传感器（0.5秒）
    static uint8_t light_counter = 0;
    if (++light_counter >= 5)
    {
      light_counter = 0;
      
      LightSensor_Data_t light_data;
      
      // 读取光敏电阻传感器数据
      if (LightSensor_Read(&hadc1, LIGHT_SENSOR_DO_GPIO_Port, LIGHT_SENSOR_DO_Pin, &light_data) == 0)
      {
        // 获取互斥锁
        if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
        {
          // 更新全局数据
          g_light_percent = light_data.light_percent;
          
          // 串口输出
          printf("[Light] ADC=%d, Voltage=%.2fV, Percent=%d%%, Digital=%s\r\n", 
                 light_data.adc_value, 
                 light_data.voltage,
                 light_data.light_percent,
                 light_data.digital_status == LIGHT_BRIGHT ? "Bright" : "Dark");
          
          // 释放互斥锁
          osMutexRelease(sensorDataMutexHandle);
        }
      }
    }
    
    // 每10个周期控制风扇（1秒）
    static uint8_t fan_counter = 0;
    if (++fan_counter >= 10)
    {
      fan_counter = 0;
      
      // 获取互斥锁
      if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
      {
        // 自动控制风扇：当光照<20%且温度>25°C时开启风扇
        if (g_light_percent < 20 && g_temperature > 25) 
        {
          if (device_status == 0)  // 如果当前关闭，则开启
          {
            Fan_Start();             // 开启风扇
            Relay_ON(RELAY_CH1);  // 开启继电器1（风扇电源）
            device_status = 1;    // 更新状态
            g_fan_status = 1;     // 更新全局风扇状态
            printf("[Control] Light=%d%%, Temp=%dC - Auto ON Fan & Relay_CH1\r\n", 
                   g_light_percent, g_temperature);
          }
        }
        else 
        {
          if (device_status == 1)  // 如果当前开启，则关闭
          {
            Fan_Stop();            // 关闭风扇
            Relay_OFF(RELAY_CH1); // 关闭继电器1（风扇电源）
            device_status = 0;    // 更新状态
            g_fan_status = 0;     // 更新全局风扇状态
            printf("[Control] Light=%d%%, Temp=%dC - Auto OFF Fan & Relay_CH1\r\n", 
                   g_light_percent, g_temperature);
          }
        }
        
        // 释放互斥锁
        osMutexRelease(sensorDataMutexHandle);
      }
    }
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
  * @brief  Function implementing the DisplayTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  // 等待2秒，确保传感器和HLW8032模块稳定
  osDelay(2000);
  
  // 初始化页面
  if (current_page == 0)
  {
    Draw_PowerMeter_Page();
  }
  else
  {
    Draw_EnvMonitor_Page();
  }
  printf("[DisplayTask] Initial Page: %s\r\n", current_page == 0 ? "Power Meter" : "Environment Monitor");
  
  // 记录当前页面状态，避免重复绘制
  static uint8_t last_page = 0xFF; // 初始化为无效值，确保首次强制更新
  
  for(;;)
  {
    // 检查页面是否切换
    if (last_page != current_page)
    {
      printf("[DisplayTask] Page changed from %d to %d\r\n", last_page, current_page);
      if (current_page == 0)
      {
        Draw_PowerMeter_Page();
        printf("[DisplayTask] Draw Power Meter Page\r\n");
      }
      else
      {
        Draw_EnvMonitor_Page();
        printf("[DisplayTask] Draw Env Monitor Page\r\n");
      }
      last_page = current_page;
    }
    
    // 根据当前页面更新显示
    if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
    {
      if (current_page == 0)
      {
        Update_PowerMeter_Display();
      }
      else
      {
        Update_EnvMonitor_Display();
      }
      osMutexRelease(sensorDataMutexHandle);
    }
    
    // 短暂延时
    osDelay(100);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // 等待系统稳定
  osDelay(1000);
  
  // 添加控制状态变量
  static uint8_t relay_in1_status = 0;  // IN1继电器状态 (0=关闭, 1=开启)
  static uint8_t relay_in2_status = 0;  // IN2继电器状态 (0=关闭, 1=开启)
  
  for(;;)
  {
    // 获取传感器数据互斥锁
    if (osMutexAcquire(sensorDataMutexHandle, osWaitForever) == osOK)
    {
      // 功能1: 当温度值和湿度值综合超过30℃和30%RH阈值，且检测到有人时，控制继电器IN2引脚
      if ((g_temperature > 30 || g_humidity > 30) && g_motion_status == SR501_MOTION_DETECTED)
      {
        // 检查继电器IN2是否需要开启
        if (relay_in2_status == 0)
        {
          Relay_ON(RELAY_CH2);  // 开启继电器IN2
          relay_in2_status = 1;
          printf("[Control] Temp=%dC, Humidity=%d%%, Motion=DETECTED - ON RELAY_IN2\r\n", 
                 g_temperature, g_humidity);
        }
      }
      else
      {
        // 检查继电器IN2是否需要关闭
        if (relay_in2_status == 1)
        {
          Relay_OFF(RELAY_CH2);  // 关闭继电器IN2
          relay_in2_status = 0;
          printf("[Control] Temp=%dC, Humidity=%d%%, Motion=%s - OFF RELAY_IN2\r\n", 
                 g_temperature, g_humidity, 
                 g_motion_status == SR501_MOTION_DETECTED ? "DETECTED" : "NONE");
        }
      }
      
      // 功能2: 当检测到有人且光照值低于20%时，打开IN1
      if (g_motion_status == SR501_MOTION_DETECTED && g_light_percent < 20)
      {
        // 检查继电器IN1是否需要开启
        if (relay_in1_status == 0)
        {
          Relay_ON(RELAY_CH1);  // 开启继电器IN1
          relay_in1_status = 1;
          printf("[Control] Motion=DETECTED, Light=%d%% - ON RELAY_IN1\r\n", 
                 g_light_percent);
        }
      }
      else
      {
        // 检查继电器IN1是否需要关闭
        if (relay_in1_status == 1)
        {
          Relay_OFF(RELAY_CH1);  // 关闭继电器IN1
          relay_in1_status = 0;
          printf("[Control] Motion=%s, Light=%d%% - OFF RELAY_IN1\r\n", 
                 g_motion_status == SR501_MOTION_DETECTED ? "DETECTED" : "NONE",
                 g_light_percent);
        }
      }
      
      // 释放互斥锁
      osMutexRelease(sensorDataMutexHandle);
    }
    
    // 控制任务执行周期: 500ms
    osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  printf("[KeyTask] Start. Waiting for key press...\r\n");
  
  for(;;)
  {
    // 等待按键事件标志
    uint32_t flags = osEventFlagsWait(pageSwitchFlagsHandle, 0x01, osFlagsWaitAny, osWaitForever);
    
    if (flags == 0x01) {
      printf("[KeyTask] KEY2 Pressed. Toggling page...\r\n");
      
      // 切换页面
      current_page = !current_page;
      if (current_page == 0)
        printf("[KEY2] Switch to Page 0: Power Meter\r\n");
      else
        printf("[KEY2] Switch to Page 1: Environment Monitor\r\n");
      
      // LED指示
      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    }
    
    // 简单延时以避免重复触发
    osDelay(50);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/******************************************************************************
 * 函  数： Draw_PowerMeter_Page
 * 功  能：绘制电能计量页面UI
 * 参  数：无
 * 返回值：无
 ******************************************************************************/
void Draw_PowerMeter_Page(void)
{
    // 清屏
    LCD_Fill(0, 0, LCD_W-1, LCD_H-1, BLACK);
    
    // === Header Section ===
    LCD_DrawFillRectangle(0, 0, 127, 18, DARKBLUE);
    LCD_DrawFillRectangle(0, 18, 127, 19, CYAN);
    LCD_ShowString(6, 3, "POWER METER", WHITE, DARKBLUE, 12, 0);
    LCD_ShowString(96, 3, "v1.0", LGRAY, DARKBLUE, 12, 0);
    
    // === Card 1: Voltage & Current ===
    LCD_DrawFillRectangle(3, 23, 124, 73, 0x1082);
    LCD_DrawRectangle(3, 23, 124, 73, GRAYBLUE);
    LCD_DrawFillRectangle(3, 23, 6, 73, CYAN);
    LCD_ShowString(10, 26, "V & I", CYAN, 0x1082, 12, 0);
    LCD_ShowString(10, 42, "Volt:", LGRAY, 0x1082, 12, 0);
    LCD_ShowString(10, 56, "Curr:", LGRAY, 0x1082, 12, 0);
    
    // === Card 2: Power & PF ===
    LCD_DrawFillRectangle(3, 77, 124, 117, 0x1082);
    LCD_DrawRectangle(3, 77, 124, 117, GRAYBLUE);
    LCD_DrawFillRectangle(3, 77, 6, 117, YELLOW);
    LCD_ShowString(10, 80, "POWER", YELLOW, 0x1082, 12, 0);
    LCD_ShowString(10, 94, "Watt:", LGRAY, 0x1082, 12, 0);
    LCD_ShowString(10, 106, "PF:", LGRAY, 0x1082, 12, 0);
    
    // === Card 3: Energy ===
    LCD_DrawFillRectangle(3, 121, 124, 157, 0x1082);
    LCD_DrawRectangle(3, 121, 124, 157, GRAYBLUE);
    LCD_DrawFillRectangle(3, 121, 6, 157, MAGENTA);
    LCD_ShowString(10, 124, "ENERGY", MAGENTA, 0x1082, 12, 0);
    LCD_ShowString(10, 138, "kWh:", LGRAY, 0x1082, 12, 0);
}

/******************************************************************************
 * 函  数： Draw_EnvMonitor_Page
 * 功  能：绘制环境监测页面UI
 * 参  数：无
 * 返回值：无
 ******************************************************************************/
void Draw_EnvMonitor_Page(void)
{
    // 清屏
    LCD_Fill(0, 0, LCD_W-1, LCD_H-1, BLACK);
    
    // === Header Section ===
    LCD_DrawFillRectangle(0, 0, 127, 18, DARKBLUE);
    LCD_DrawFillRectangle(0, 18, 127, 19, CYAN);
    LCD_ShowString(6, 3, "ENV MONITOR", WHITE, DARKBLUE, 12, 0);
    LCD_ShowString(96, 3, "v1.0", LGRAY, DARKBLUE, 12, 0);
    
    // === Card 1: Temperature & Humidity ===
    LCD_DrawFillRectangle(3, 23, 124, 73, 0x1082);
    LCD_DrawRectangle(3, 23, 124, 73, GRAYBLUE);
    LCD_DrawFillRectangle(3, 23, 6, 73, CYAN);
    LCD_ShowString(10, 26, "CLIMATE", CYAN, 0x1082, 12, 0);
    LCD_ShowString(10, 42, "Temp:", LGRAY, 0x1082, 12, 0);
    LCD_ShowString(10, 56, "Humi:", LGRAY, 0x1082, 12, 0);
    
    // === Card 2: Light Sensor ===
    LCD_DrawFillRectangle(3, 77, 124, 117, 0x1082);
    LCD_DrawRectangle(3, 77, 124, 117, GRAYBLUE);
    LCD_DrawFillRectangle(3, 77, 6, 117, YELLOW);
    LCD_ShowString(10, 80, "LIGHT", YELLOW, 0x1082, 12, 0);
    LCD_ShowString(10, 94, "Level:", LGRAY, 0x1082, 12, 0);
    LCD_DrawRectangle(10, 105, 120, 113, GRAYBLUE);
    
    // === Card 3: Motion Sensor ===
    LCD_DrawFillRectangle(3, 121, 124, 157, 0x1082);
    LCD_DrawRectangle(3, 121, 124, 157, GRAYBLUE);
    LCD_DrawFillRectangle(3, 121, 6, 157, MAGENTA);
    LCD_ShowString(10, 124, "MOTION", MAGENTA, 0x1082, 12, 0);
    LCD_ShowString(10, 138, "Status:", LGRAY, 0x1082, 12, 0);
}

/******************************************************************************
 * 函  数： Update_EnvMonitor_Display
 * 功  能：更新环境监测页面显示
 * 参  数：无
 * 返回值：无
 ******************************************************************************/
void Update_EnvMonitor_Display(void)
{
    // 温度显示
    char temp_str[20];
    sprintf(temp_str, "%2dC  ", g_temperature);
    LCD_ShowString(50, 42, temp_str, WHITE, 0x1082, 16, 0);
    
    // 湿度显示
    char humi_str[20];
    sprintf(humi_str, "%2d%%  ", g_humidity);
    LCD_ShowString(50, 56, humi_str, WHITE, 0x1082, 16, 0);
    
    // 光照强度显示
    char light_str[20];
    sprintf(light_str, "%3d%%", g_light_percent);
    LCD_ShowString(52, 94, light_str, WHITE, 0x1082, 16, 0);
    
    // 光照强度进度条
    uint8_t bar_width = (uint8_t)((g_light_percent * 108) / 100);
    // 清空进度条区域
    LCD_DrawFillRectangle(11, 106, 119, 112, BLACK);
    // 绘制渐变进度条
    if (g_light_percent > 0)
    {
        uint16_t bar_color;
        if (g_light_percent > 70) 
            bar_color = YELLOW;
        else if (g_light_percent > 40)
            bar_color = GRED;
        else
            bar_color = CYAN;
        
        LCD_DrawFillRectangle(11, 106, 11+bar_width, 112, bar_color);
    }
    
    // 风扇状态指示
    if (g_fan_status == 1)
    {
        LCD_ShowString(92, 94, "[FAN]", YELLOW, 0x1082, 12, 0);
    }
    else
    {
        LCD_ShowString(92, 94, "     ", 0x1082, 0x1082, 12, 0);
    }
    
    // 人体感应状态显示
    if (g_motion_status == SR501_MOTION_DETECTED)
    {
        LCD_DrawFillRectangle(60, 136, 120, 151, RED);
        LCD_ShowString(66, 139, "ALERT!", WHITE, RED, 12, 0);
        LCD_DrawFillRectangle(108, 140, 113, 145, WHITE);
    }
    else
    {
        LCD_DrawFillRectangle(60, 136, 120, 151, 0x2945);
        LCD_ShowString(70, 139, "Clear", LIGHTGREEN, 0x2945, 12, 0);
    }
}

/******************************************************************************
 * 函  数： Update_PowerMeter_Display
 * 功  能：更新电能计量页面显示
 * 参  数：无
 * 返回值：无
 ******************************************************************************/
void Update_PowerMeter_Display(void)
{
    extern HLW8032_Data g_power_metrics;
    
    char str[20] = {0};
    
    // 显示电压值 (在"Volt:"标签后显示)
    if (g_power_metrics.voltage > 0.1f) {
        sprintf(str, "%3d.%02dV  ", (int)g_power_metrics.voltage, ((int)(g_power_metrics.voltage * 100)) % 100);
    } else {
        sprintf(str, "0.00V  ");
    }
    LCD_ShowString(50, 42, str, WHITE, 0x1082, 16, 0);
    
    // 显示电流值 (在"Curr:"标签后显示)
    if (g_power_metrics.current > 0.001f) {
        sprintf(str, "%2d.%03dA  ", (int)g_power_metrics.current, ((int)(g_power_metrics.current * 1000)) % 1000);
    } else {
        sprintf(str, "0.000A  ");
    }
    LCD_ShowString(50, 56, str, WHITE, 0x1082, 16, 0);
    
    // 显示功率值 (在"Watt:"标签后显示)
    if (g_power_metrics.active_power > 0.1f) {
        sprintf(str, "%4d.%02dW  ", (int)g_power_metrics.active_power, ((int)(g_power_metrics.active_power * 100)) % 100);
    } else {
        sprintf(str, "0.00W  ");
    }
    LCD_ShowString(50, 94, str, WHITE, 0x1082, 16, 0);
    
    // 显示功率因数 (在"PF:"标签后显示)
    if (g_power_metrics.power_factor > 0.01f) {
        sprintf(str, "%3d%%  ", (int)(g_power_metrics.power_factor * 100));
    } else {
        sprintf(str, "0%%  ");
    }
    LCD_ShowString(50, 106, str, WHITE, 0x1082, 16, 0);
    
    // 显示电能值 (在"kWh:"标签后显示)
    if (g_power_metrics.energy > 0.0001) {
        sprintf(str, "%.3fkWh  ", g_power_metrics.energy);
    } else {
        sprintf(str, "0.000kWh  ");
    }
    LCD_ShowString(50, 138, str, WHITE, 0x1082, 16, 0);
}

/* USER CODE END Application */
