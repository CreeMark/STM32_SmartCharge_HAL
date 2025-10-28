#ifndef __BSP_LIGHTSENSOR_H
#define __BSP_LIGHTSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 光敏电阻传感器数据结构体
 */
typedef struct
{
    uint16_t adc_value;          // ADC原始值 (0-4095)
    float voltage;               // 电压值 (0-3.3V)
    uint8_t light_percent;       // 光照强度百分比 (0-100%)
    uint8_t digital_status;      // 数字状态 (0=暗, 1=亮)
} LightSensor_Data_t;

/**
 * @brief 光敏电阻传感器状态枚举
 */
typedef enum
{
    LIGHT_DARK = 0,              // 环境暗（低于阈值）
    LIGHT_BRIGHT = 1             // 环境亮（高于阈值）
} LightSensor_Status_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* ADC参考电压 (单位: V) */
#define LIGHT_SENSOR_VREF           3.3f

/* ADC分辨率 (12位 = 4096) */
#define LIGHT_SENSOR_ADC_MAX        4095

/* 光照强度阈值百分比 (用于数字判断，可根据实际需求调整) */
#define LIGHT_SENSOR_THRESHOLD      50    // 50%作为亮暗分界线

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化光敏电阻传感器
 * @param  hadc: ADC句柄指针 (例如: &hadc1)
 * @param  GPIOx_DO: 数字输出引脚的GPIO端口 (例如: GPIOB)
 * @param  GPIO_Pin_DO: 数字输出引脚号 (例如: GPIO_PIN_15)
 * @retval None
 * @note   GPIO和ADC已在CubeMX生成的初始化函数中配置，本函数执行ADC校准
 */
void LightSensor_Init(ADC_HandleTypeDef *hadc, GPIO_TypeDef *GPIOx_DO, uint16_t GPIO_Pin_DO);

/**
 * @brief  读取光敏电阻传感器数据（包含模拟和数字信号）
 * @param  hadc: ADC句柄指针
 * @param  GPIOx_DO: 数字输出引脚的GPIO端口
 * @param  GPIO_Pin_DO: 数字输出引脚号
 * @param  data: 数据结构体指针，用于存储读取结果
 * @retval 0=成功, 1=失败
 */
uint8_t LightSensor_Read(ADC_HandleTypeDef *hadc, GPIO_TypeDef *GPIOx_DO, uint16_t GPIO_Pin_DO, LightSensor_Data_t *data);

/**
 * @brief  读取光敏电阻ADC原始值
 * @param  hadc: ADC句柄指针
 * @retval ADC原始值 (0-4095)
 */
uint16_t LightSensor_ReadADC(ADC_HandleTypeDef *hadc);

/**
 * @brief  读取光敏电阻数字输出状态
 * @param  GPIOx: GPIO端口
 * @param  GPIO_Pin: GPIO引脚号
 * @retval LIGHT_DARK=0(暗), LIGHT_BRIGHT=1(亮)
 */
LightSensor_Status_t LightSensor_ReadDigital(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/**
 * @brief  将ADC值转换为电压
 * @param  adc_value: ADC原始值 (0-4095)
 * @retval 电压值 (0.0-3.3V)
 */
float LightSensor_ADC_To_Voltage(uint16_t adc_value);

/**
 * @brief  将ADC值转换为光照强度百分比
 * @param  adc_value: ADC原始值 (0-4095)
 * @retval 光照强度百分比 (0-100%)
 * @note   百分比越大表示光照越强
 */
uint8_t LightSensor_ADC_To_Percent(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LIGHTSENSOR_H */
