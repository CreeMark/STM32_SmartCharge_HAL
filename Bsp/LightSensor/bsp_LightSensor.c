/***********************************************************************************************************************************
 ** 【文件名称】  bsp_LightSensor.c
 **
 ** 【文件功能】  光敏电阻传感器模块驱动
 **
 ** 【模块说明】  光敏电阻传感器模块（带比较器输出）
 **               特点：
 **               - 双路输出：数字DO口（开关量）+ 模拟AO口（连续电压）
 **               - DO口输出：通过板载电位器调节阈值，低于阈值输出高电平，高于阈值输出低电平
 **               - AO口输出：随光照强度变化的连续模拟电压（0-3.3V）
 **               - 工作电压：DC 3.3V-5V
 **               - 检测范围：可见光（400-700nm）
 **               - 响应速度：快速响应（<1ms）
 **
 ** 【硬件连接】  VCC  -> 3.3V/5V
 **               GND  -> GND
 **               DO   -> PB15 (LIGHT_SENSOR_DO) - 数字输出
 **               AO   -> PA2  (ADC1_IN2)        - 模拟输出
 **
 ** 【使用说明】  1. 确保在main.c中已调用MX_ADC1_Init()和MX_GPIO_Init()
 **               2. 调用LightSensor_Init()进行ADC校准
 **               3. 周期性调用LightSensor_Read()读取光照数据
 **
 ** 【适用平台】  STM32F103 + HAL库
 **
 ** 【作者信息】  Created on: 2025-10-20
 **
************************************************************************************************************************************/

#include "bsp_LightSensor.h"
#include "adc.h"

/******************************************************************************
 * 函  数： LightSensor_Init
 * 功  能： 初始化光敏电阻传感器
 * 参  数： ADC_HandleTypeDef *hadc  - ADC句柄指针
 *          GPIO_TypeDef *GPIOx_DO    - 数字输出引脚的GPIO端口
 *          uint16_t GPIO_Pin_DO      - 数字输出引脚号
 * 返回值： 无
 * 备  注： GPIO和ADC已在MX_GPIO_Init()和MX_ADC1_Init()中完成配置
 *          本函数执行ADC校准以提高精度
 ******************************************************************************/
void LightSensor_Init(ADC_HandleTypeDef *hadc, GPIO_TypeDef *GPIOx_DO, uint16_t GPIO_Pin_DO)
{
    // ADC校准（提高转换精度）
    HAL_ADCEx_Calibration_Start(hadc);
    
    // 启动ADC（如果配置为连续转换模式，这里启动一次即可）
    // 注意：如果ADC配置为连续模式，这里可选择性启动
    // HAL_ADC_Start(hadc);
}

/******************************************************************************
 * 函  数： LightSensor_Read
 * 功  能： 读取光敏电阻传感器数据（包含模拟和数字信号）
 * 参  数： ADC_HandleTypeDef *hadc      - ADC句柄指针
 *          GPIO_TypeDef *GPIOx_DO        - 数字输出引脚的GPIO端口
 *          uint16_t GPIO_Pin_DO          - 数字输出引脚号
 *          LightSensor_Data_t *data      - 数据结构体指针
 * 返回值： 0=成功, 1=失败
 * 备  注： 该函数会同时读取ADC值和数字信号，并进行转换计算
 ******************************************************************************/
uint8_t LightSensor_Read(ADC_HandleTypeDef *hadc, GPIO_TypeDef *GPIOx_DO, uint16_t GPIO_Pin_DO, LightSensor_Data_t *data)
{
    if (data == NULL)
    {
        return 1;  // 参数错误
    }
    
    // 读取ADC值
    data->adc_value = LightSensor_ReadADC(hadc);
    
    // 转换为电压
    data->voltage = LightSensor_ADC_To_Voltage(data->adc_value);
    
    // 转换为光照强度百分比
    data->light_percent = LightSensor_ADC_To_Percent(data->adc_value);
    
    // 读取数字输出状态
    data->digital_status = (uint8_t)LightSensor_ReadDigital(GPIOx_DO, GPIO_Pin_DO);
    
    return 0;  // 成功
}

/******************************************************************************
 * 函  数： LightSensor_ReadADC
 * 功  能： 读取光敏电阻ADC原始值
 * 参  数： ADC_HandleTypeDef *hadc  - ADC句柄指针
 * 返回值： ADC原始值 (0-4095)
 * 备  注： 采用轮询方式读取ADC，适合周期性采样
 ******************************************************************************/
uint16_t LightSensor_ReadADC(ADC_HandleTypeDef *hadc)
{
    uint16_t adc_value = 0;
    
    // 启动ADC转换
    HAL_ADC_Start(hadc);
    
    // 等待转换完成（超时时间100ms）
    if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK)
    {
        // 读取ADC转换结果
        adc_value = HAL_ADC_GetValue(hadc);
    }
    
    // 停止ADC转换（如果使用连续模式，可以不停止）
    HAL_ADC_Stop(hadc);
    
    return adc_value;
}

/******************************************************************************
 * 函  数： LightSensor_ReadDigital
 * 功  能： 读取光敏电阻数字输出状态
 * 参  数： GPIO_TypeDef *GPIOx  - GPIO端口
 *          uint16_t GPIO_Pin    - GPIO引脚号
 * 返回值： LIGHT_DARK=0(暗), LIGHT_BRIGHT=1(亮)
 * 备  注： DO口通过板载比较器输出，可通过电位器调节阈值
 *          注意：有些模块DO输出为反向逻辑（亮时输出低电平）
 *          如果您的模块是反向逻辑，请修改此函数的返回值
 ******************************************************************************/
LightSensor_Status_t LightSensor_ReadDigital(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_PinState pin_state;
    
    // 读取GPIO引脚电平
    pin_state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    
    // 根据实际模块逻辑进行判断
    // 正向逻辑：高电平=亮，低电平=暗
    if (pin_state == GPIO_PIN_SET)
    {
        return LIGHT_BRIGHT;  // 环境亮
    }
    else
    {
        return LIGHT_DARK;    // 环境暗
    }
    
    // 如果您的模块是反向逻辑（大多数光敏模块），请使用以下代码：
    // if (pin_state == GPIO_PIN_RESET)
    // {
    //     return LIGHT_BRIGHT;  // 环境亮（低电平）
    // }
    // else
    // {
    //     return LIGHT_DARK;    // 环境暗（高电平）
    // }
}

/******************************************************************************
 * 函  数： LightSensor_ADC_To_Voltage
 * 功  能： 将ADC值转换为电压
 * 参  数： uint16_t adc_value  - ADC原始值 (0-4095)
 * 返回值： 电压值 (0.0-3.3V)
 * 备  注： 转换公式: Voltage = (ADC_Value / 4095) * 3.3V
 ******************************************************************************/
float LightSensor_ADC_To_Voltage(uint16_t adc_value)
{
    return (adc_value * LIGHT_SENSOR_VREF) / LIGHT_SENSOR_ADC_MAX;
}

/******************************************************************************
 * 函  数： LightSensor_ADC_To_Percent
 * 功  能： 将ADC值转换为光照强度百分比
 * 参  数： uint16_t adc_value  - ADC原始值 (0-4095)
 * 返回值： 光照强度百分比 (0-100%)
 * 备  注： 转换公式（反向逻辑）: Percent = 100 - (ADC_Value / 4095) * 100
 *          适用于：ADC值越大 = 环境越暗的光敏模块
 *          如果您的模块是正向逻辑，请改用: (adc_value * 100) / LIGHT_SENSOR_ADC_MAX
 ******************************************************************************/
uint8_t LightSensor_ADC_To_Percent(uint16_t adc_value)
{
    // 反向逻辑：ADC值越大表示越暗，百分比越小
    return 100 - (uint8_t)((adc_value * 100) / LIGHT_SENSOR_ADC_MAX);
}
