#ifndef __BSP_FAN_H
#define __BSP_FAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PWM风扇状态枚举
 */
typedef enum
{
    FAN_OFF = 0,        // 风扇关闭
    FAN_ON = 1          // 风扇开启（全速）
} Fan_Status_t;

/* Exported constants --------------------------------------------------------*/

/* 风扇PWM控制引脚定义（连接至PC0） */
#define FAN_PWM_GPIO_PORT       GPIOC
#define FAN_PWM_GPIO_PIN        GPIO_PIN_0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化PWM风扇控制引脚
 * @param  None
 * @retval None
 * @note   GPIO已在MX_GPIO_Init()中配置，本函数确保初始状态为关闭
 */
void Fan_Init(void);

/**
 * @brief  启动风扇（全速运行）
 * @param  None
 * @retval None
 * @note   对于低电平有效的PWM风扇，输出低电平启动风扇
 */
void Fan_Start(void);

/**
 * @brief  停止风扇
 * @param  None
 * @retval None
 * @note   对于低电平有效的PWM风扇，输出高电平停止风扇
 */
void Fan_Stop(void);

/**
 * @brief  切换风扇状态（开/关）
 * @param  None
 * @retval None
 */
void Fan_Toggle(void);

/**
 * @brief  获取风扇当前状态
 * @param  None
 * @retval FAN_OFF=0(关闭), FAN_ON=1(运行)
 */
Fan_Status_t Fan_GetStatus(void);

/**
 * @brief  设置风扇状态
 * @param  status: FAN_OFF或FAN_ON
 * @retval None
 */
void Fan_SetStatus(Fan_Status_t status);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_FAN_H */
