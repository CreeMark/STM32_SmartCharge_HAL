#ifndef __BSP_SR501_H
#define __BSP_SR501_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    SR501_NO_MOTION = 0,    // 未检测到人体
    SR501_MOTION_DETECTED   // 检测到人体
} SR501_Status_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化HC-SR501人体红外感应模块
 * @param  GPIOx: GPIO端口 (例如: GPIOC)
 * @param  GPIO_Pin: GPIO引脚 (例如: GPIO_PIN_12)
 * @retval None
 * @note   GPIO已在MX_GPIO_Init()中初始化，本函数保留用于后续扩展
 */
void SR501_Init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/**
 * @brief  读取HC-SR501传感器状态
 * @param  GPIOx: GPIO端口 (例如: GPIOC)
 * @param  GPIO_Pin: GPIO引脚 (例如: GPIO_PIN_12)
 * @retval SR501_Status_t: SR501_NO_MOTION=无人体, SR501_MOTION_DETECTED=检测到人体
 * @note   HC-SR501输出高电平表示检测到人体运动，低电平表示无运动
 */
SR501_Status_t SR501_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SR501_H */
