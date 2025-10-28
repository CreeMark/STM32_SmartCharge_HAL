/***********************************************************************************************************************************
 ** 【文件名称】  bsp_relay.c
 **
 ** 【文件功能】  继电器模块驱动实现
 **
 ** 【模块说明】  单路/双路继电器控制模块
 **               特点：
 **               - 低电平触发：IN口接收低电平时继电器吸合，高电平时继电器断开
 **               - 工作电压：DC 5V（继电器线圈），控制信号3.3V/5V兼容
 **               - 负载能力：AC 250V/10A, DC 30V/10A
 **               - 响应时间：<10ms
 **
 ** 【硬件连接】  VCC  -> 5V
 **               GND  -> GND
 **               IN1  -> PC1 (RELAY_IN1) - 继电器1控制信号
 **               IN2  -> PC2 (RELAY_IN2) - 继电器2控制信号（可选）
 **
 ** 【使用说明】  1. 确保在main.c中已调用MX_GPIO_Init()
 **               2. 调用Relay_ON(RELAY_CH1)开启继电器1
 **               3. 调用Relay_OFF(RELAY_CH1)关闭继电器1
 **
 ** 【适用平台】  STM32F103 + HAL库
 **
 ** 【作者信息】  Created on: 2025-10-22
 **
************************************************************************************************************************************/

#include "bsp_relay.h"
#include "main.h"

/******************************************************************************
 * 函  数： Relay_Init
 * 功  能： 初始化继电器模块
 * 参  数： 无
 * 返回值： 无
 * 备  注： GPIO已由MX_GPIO_Init()初始化，本函数确保继电器初始为断开状态
 ******************************************************************************/
void Relay_Init(void)
{
    // 确保继电器初始为断开状态（高电平）
    HAL_GPIO_WritePin(RELAY_IN1_GPIO_Port, RELAY_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_IN2_GPIO_Port, RELAY_IN2_Pin, GPIO_PIN_SET);
}

/******************************************************************************
 * 函  数： Relay_ON
 * 功  能： 继电器吸合（闭合触点，负载通电）
 * 参  数： Relay_Channel_t channel - 继电器通道（RELAY_CH1或RELAY_CH2）
 * 返回值： 无
 * 备  注： 低电平有效：输出GPIO_PIN_RESET使继电器吸合
 ******************************************************************************/
void Relay_ON(Relay_Channel_t channel)
{
    if (channel == RELAY_CH1)
    {
        // 输出低电平，继电器吸合
        HAL_GPIO_WritePin(RELAY_IN1_GPIO_Port, RELAY_IN1_Pin, GPIO_PIN_RESET);
    }
    else if (channel == RELAY_CH2)
    {
        // 输出低电平，继电器吸合
        HAL_GPIO_WritePin(RELAY_IN2_GPIO_Port, RELAY_IN2_Pin, GPIO_PIN_RESET);
    }
}

/******************************************************************************
 * 函  数： Relay_OFF
 * 功  能： 继电器释放（断开触点，负载断电）
 * 参  数： Relay_Channel_t channel - 继电器通道（RELAY_CH1或RELAY_CH2）
 * 返回值： 无
 * 备  注： 高电平：输出GPIO_PIN_SET使继电器释放
 ******************************************************************************/
void Relay_OFF(Relay_Channel_t channel)
{
    if (channel == RELAY_CH1)
    {
        // 输出高电平，继电器释放
        HAL_GPIO_WritePin(RELAY_IN1_GPIO_Port, RELAY_IN1_Pin, GPIO_PIN_SET);
    }
    else if (channel == RELAY_CH2)
    {
        // 输出高电平，继电器释放
        HAL_GPIO_WritePin(RELAY_IN2_GPIO_Port, RELAY_IN2_Pin, GPIO_PIN_SET);
    }
}

/******************************************************************************
 * 函  数： Relay_Toggle
 * 功  能： 继电器状态翻转
 * 参  数： Relay_Channel_t channel - 继电器通道（RELAY_CH1或RELAY_CH2）
 * 返回值： 无
 * 备  注： 切换继电器当前状态（开→关 或 关→开）
 ******************************************************************************/
void Relay_Toggle(Relay_Channel_t channel)
{
    if (channel == RELAY_CH1)
    {
        HAL_GPIO_TogglePin(RELAY_IN1_GPIO_Port, RELAY_IN1_Pin);
    }
    else if (channel == RELAY_CH2)
    {
        HAL_GPIO_TogglePin(RELAY_IN2_GPIO_Port, RELAY_IN2_Pin);
    }
}

/******************************************************************************
 * 函  数： Relay_GetStatus
 * 功  能： 获取继电器当前状态
 * 参  数： Relay_Channel_t channel - 继电器通道（RELAY_CH1或RELAY_CH2）
 * 返回值： Relay_Status_t - RELAY_ON=吸合, RELAY_OFF=断开
 * 备  注： 读取GPIO电平判断继电器状态
 *          低电平=吸合(ON), 高电平=断开(OFF)
 ******************************************************************************/
Relay_Status_t Relay_GetStatus(Relay_Channel_t channel)
{
    GPIO_PinState pin_state;
    
    if (channel == RELAY_CH1)
    {
        pin_state = HAL_GPIO_ReadPin(RELAY_IN1_GPIO_Port, RELAY_IN1_Pin);
    }
    else if (channel == RELAY_CH2)
    {
        pin_state = HAL_GPIO_ReadPin(RELAY_IN2_GPIO_Port, RELAY_IN2_Pin);
    }
    else
    {
        return RELAY_OFF;  // 无效通道，默认返回断开
    }
    
    // 低电平有效：低电平=吸合，高电平=断开
    if (pin_state == GPIO_PIN_RESET)
    {
        return RELAY_ON;   // 继电器吸合
    }
    else
    {
        return RELAY_OFF;  // 继电器断开
    }
}
