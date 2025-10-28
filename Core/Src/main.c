/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsp_UART.h"            // 串口底层驱动文件; 已重写好初始化、收发，调用函数即可使用串口
#include "bsp_W25Q128.h"         // 初始化外部Flash：芗片型号W25Q128
#include "lcd_init.h"            // ST7735S 1.8寸LCD驱动
#include "lcd.h"                 // LCD绘图函数

#include "lvgl.h"                // 它为整个LVGL提供了更完整的头文件引用
#include "lv_port_disp.h"        // LVGL的显示支持
#include "lv_port_indev.h"       // LVGL的触屏支持

#include "dht11.h"               // DHT11温湿度传感器驱动
#include "bsp_SR501.h"           // HC-SR501人体红外感应模块驱动
#include "bsp_LightSensor.h"    // 光敏电阻传感器驱动
#include "bsp_fan.h"             // PWM风扇驱动
#include "bsp_relay.h"           // 继电器模块驱动
#include "hlw8032.h"             // HLW8032电能计量模块驱动

#include <stdio.h>               // sprintf函数支持
#include <string.h>              // 字符串处理函数


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
// 页面管理变量（统一由freertos.c维护）
// extern uint8_t current_page;  // 如需在main.c使用，请改为extern引用

// 全局传感器数据存储变量，用于实现数据收集与页面显示分离
static uint8_t g_temperature = 0;     // 温度值
static uint8_t g_humidity = 0;        // 湿度值
static uint8_t g_light_percent = 0;   // 光照强度百分比
static SR501_Status_t g_motion_status = SR501_NO_MOTION;  // 人体感应状态
static uint8_t g_fan_status = 0;      // 风扇状态标志
// g_power_metrics 在 freertos.c 中维护，这里不重复定义
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Prototypes for UI page functions implemented in freertos.c
extern void Draw_PowerMeter_Page(void);
extern void Draw_EnvMonitor_Page(void);
extern void Update_PowerMeter_Display(void);
extern void Update_EnvMonitor_Display(void);

/******************************************************************************
/*` Moved: Draw_EnvMonitor_Page is implemented in freertos.c */

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

  /* USER CODE BEGIN SysInit */
  // 关键修复：必须在GPIO初始化之前禁用JTAG，释放PB3/PB4引脚
  __HAL_AFIO_REMAP_SWJ_NOJTAG();  // Disable JTAG, keep SWD for debugging
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    /* 用户代码，必须写在配对的BEGIN与END之间 */

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);                // 引脚置高电平，红亮灭
                                                                                   
    UART1_Init(115200);                                                             // 初始化 串口1; 已写好底层，调用h中的函数即可使用; 引脚(PA9 +PA10)、波特率-None-8-1; 如果使用CubeMX配置，请使用前述引脚，但，不要在MX上进行中断及DMA配置，否则冲突
    
    // JTAG已在SysInit阶段禁用，PB3/PB4现已可用于GPIO
    printf("JTAG Disabled (in SysInit), PB3/PB4 freed for GPIO\r\n");

    W25Q128_Init();                                                                 // 外部Flash W25Q128 初始化; 共16M空间：0~10M用户分配、10~16M已烧录了汉字字库

    LCD_Init();                                                                     // 初始化ST7735S 1.8寸LCD                                                           

    // XPT2046_Init(LCD_W, LCD_H, USE_HORIZONTAL);                                  // 触摸屏初始化(新屏无触摸，已关闭)

    // LVGL初始化已关闭 - 使用原生LCD绘图
    // lv_init();                                                                      // LVGL 初始化
    // lv_port_disp_init();                                                            // 注册LVGL的显示任务
    // lv_port_indev_init();                                                           // 注册LVGL的触屏检测任务
  
    HAL_TIM_Base_Start_IT(&htim6);                                                  // 使能TIM6, 产生1ms的计时中断
    
    // Wait for sensors to stabilize (DHT11 needs ~1s after power-on)
    printf("[System] Waiting for sensors to stabilize...\r\n");
    HAL_Delay(1500);  // Wait 1.5 seconds for sensor stabilization
    printf("[System] Sensor stabilization complete!\r\n");
    
    // DHT11 Temperature & Humidity Sensor Initialization
    printf("[DHT11] Initializing DHT11 sensor on PB4...\r\n");
    if(DHT11_Init() == 0)  // DHT11_Init()返回0表示成功，1表示失败
    {
        printf("[DHT11] Init Success - Sensor detected on PB4\r\n");
    }
    else
    {
        printf("[DHT11] Init Failed - Check Hardware Connection\r\n");
        printf("[DHT11] Please verify: 1) Pin PB4 connection 2) Power supply (3.3V-5V) 3) Pull-up resistor (module usually has built-in)\r\n");
    }
    
    // HC-SR501 Human Infrared Sensor Initialization
    SR501_Init(SR501_IN_GPIO_Port, SR501_IN_Pin);
    printf("HC-SR501 Init Success (Pin: PC12)\r\n");
    printf("Note: HC-SR501 requires ~1min warm-up time after power-on\r\n");
    
    // Light Sensor Initialization
    LightSensor_Init(&hadc1, LIGHT_SENSOR_DO_GPIO_Port, LIGHT_SENSOR_DO_Pin);
    printf("Light Sensor Init Success (DO: PB15, AO: PA2/ADC1_IN2)\r\n");
    
    // PWM Fan Initialization
    Fan_Init();
    printf("PWM Fan Init Success (PWM: PC0)\r\n");
    
    // Relay Initialization
    Relay_Init();
    printf("Relay Init Success (IN1: PC1, IN2: PC2)\r\n");
    
    // HLW8032 Power Meter Initialization (BSP解析器 + USART2 PA3-RX)
    hlw8032_init();
    printf("HLW8032 Parser Init Success (USART2: PA3-RX, 4800bps)\r\n");
    
    // 注意：页面绘制和数据显示现在由FreeRTOS任务处理
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* 用户代码，必须写在配对的BEGIN与END之间 */

        /** 1ms延时 **/
        HAL_Delay(1 - 1);                                                  // 延时函数，参数：ms; 注意：CubeMX生成的HAL_Delay(), 函数内部为避免无效操作，会对传入参数+1，因此，如果只需要几ms延时建议传入参数时-1，如果需要大几百ms的非精准延时，那参数不-1也影响不大

        
    

        /** UART1 是否接收到数据 **/
        if (UART1_GetRxNum())                                              // 如果接收字节数>0, 即为接收到新一帧数据
        {
            /*
            if (strstr((char *)UART1_GetRxData(), "XPT2046"))              // 判断是否收到"XPT2046", 调用触摸屏重新校准函数
            {
                XPT2046_ReCalibration();                                   // 进入触摸屏校准
            }
            */
            UART1_ClearRx();                                               // 桦0本次接收
        }
                
        /** 调试：每5秒打印一次USART2寄存器状态 **/
        static uint16_t msUSART2Debug = 0;
        if (++msUSART2Debug >= 5000)
        {
            msUSART2Debug = 0;
            printf("[USART2 Debug] SR=0x%04X, CR1=0x%04X, BRR=0x%04X\r\n", 
                   USART2->SR, USART2->CR1, USART2->BRR);
        }
        
        /** LVGL已禁用 **/
        // static uint8_t msLVGL = 0;                                         // 用于LVGL周期任务计时
        // if (msLVGL++ >= 5)                                                 // 每5ms执行一次
        // {
        //     lv_timer_handler();                                            // 调用LVGL的周期性任务函数，它的作用是检查所有已注册任务的时间戳，执行那些已到期的任务
        //     msLVGL = 0;                                                    // 计数清0
        // }

        /** 每500ms闪烁一次蓝色LED **/
        static uint16_t msLED = 0;
        if (++msLED == 500)                                                // 每500ms执行一次
        {
            msLED = 0;                                                     // 计数清0
            HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);          // 规律地闪烁蓝色LED，方便外部观察系统是否正常运行
        }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 所有用户代码，必须写在配对的BEGIN与END注释行之间，否则重新生成时会被删除 */

/******************************************************************************
 * 函  数： HAL_GPIO_EXTI_Callback
 * 功  能： EXTI中断回调函数
 * 参  数： uint16_t  GPIO_Pin    引脚编号，参数范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 * 备  注： CubeMX生成的代码在思维和风格上，与使用标准库编写时有很大的区别：
 *          1：这个是回调函数，不是中断服务函数。完整的调用：EXTIx_IRQHandler() > HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_x) > HAL_GPIO_EXTI_Callback(GPIO_PIN_x)
 *          2：必须使用这个函数名称，因为它在CubeMX生成时，已被写好了各种函数调用、函数弱定义(在stm32xx_hal_gpio.c的底部); 不建议在原弱定义中增添代码，而是重写本函数
 *          3：中断条件触发后，本函数被自动调用，无需进行人工调用; 也无需进行中断标志的清理，因为HAL_GPIO_EXTI_IRQHandler()函数中已有操作;
 *          4：生成的所有EXTI中断服务函数，都统一调用这个函数，以引脚编号作参数
 *          5：判断参数传进来的引脚编号，即可知道是哪个外部中断线产生的中断信号
******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_1_Pin)
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        printf("KEY_1 Pressed\r\n");
    }

    if (GPIO_Pin == KEY_2_Pin)
    {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        // 页面切换由FreeRTOS KeyTask处理，这里不进行页面绘制
        printf("KEY_2 Interrupt - page switch handled by KeyTask\r\n");
    }
}



/******************************************************************************
 * 函  数： HAL_TIM_PeriodElapsedCallback
 * 功  能： 周期更新回调函数
 * 备  注： 本函数是TIM的CNT溢出中断回调函数。
 *          当TIM的计数器CNT，完成1周期计数时触发(向上计数：CNT==ARR、向下计数：CNT==0);
 *          上述中断触发后，硬件自动调用相关中断服务函数，继而调用本函数。
 *          所有TIM的周期更新中断，都是调用本函数，因此需要在函数内判断是哪一个TIM触发的中断;
 * 参  数： TIM_HandleTypeDef   *htim
 * 返回值： 无
******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 处理TIM6中断：LVGL心跳和LED闪烁
    if (htim->Instance == TIM6)                     
    {
        lv_tick_inc(1);                             // 给LVGL提供1ms的心跳时期

        static uint16_t ledTimes = 0;               // 用于LED闪烁计时
        if (ledTimes++ >= 500)                      // 每500ms执行一次LED闪烁
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);  // 反转LED引脚的电平，即闪烁LED
            ledTimes = 0;                           // 计时清0
        }
    }
    // 处理TIM1中断：HAL时基（FreeRTOS需要）
    else if (htim->Instance == TIM1) 
    {
        HAL_IncTick();  // 更新HAL库的系统滴答
    }
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
