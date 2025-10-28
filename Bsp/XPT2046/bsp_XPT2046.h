#ifndef  __BSP_XPT2046_H
#define  __BSP_XPT2046_H


#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f10x.h"                       // 标准库
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"                   // HAL库
#endif

#include "lcd.h"                                 // ST7735S LCD驱动


/*****************************************************************************
 ** 引脚宠定义
 ** 备注：使用的是模拟SPI, INT引脚使用状态轮询, 不使用中断;
****************************************************************************/
#define    XPT2046_IRQ_GPIO       GPIOC        // 触摸信号指示引脚(不使用中断) 
#define    XPT2046_IRQ_PIN        GPIO_PIN_1

#define    XPT2046_CS_GPIO        GPIOC        // 模拟SPI_CS
#define    XPT2046_CS_PIN         GPIO_PIN_4  

#define    XPT2046_CLK_GPIO       GPIOC        // 模拟SPI_CLK
#define    XPT2046_CLK_PIN        GPIO_PIN_0   

#define    XPT2046_MOSI_GPIO      GPIOC        // 模拟SPI_MOSI
#define    XPT2046_MOSI_PIN       GPIO_PIN_3   

#define    XPT2046_MISO_GPIO      GPIOC        // 模拟SPI_MISO
#define    XPT2046_MISO_PIN       GPIO_PIN_2   
         


/*****************************************************************************
 ** 声明全局函数声明                 
 *****************************************************************************/
// 初始化
void      XPT2046_Init (uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir ); // 初始化(引脚, 校准检测);  参数: 显示屏的显示方向, 取值范围: 1,2,3,4
// 重新校准
uint8_t   XPT2046_ReCalibration(void);                                        // 触摸坐标重新校准; 当发现坐标不对时，调用一次本函数，用笔尖依次点击十字光标线即可。
// 触摸检测、获取坐标; 
uint8_t   XPT2046_IsPressed(void);                                            // 判断触摸屏按下状态; 返回:0-释放、1-按下; 移植LVGL时，在lv_port_indev.c文件的touchpad_is_pressed()函数里调用此函数即可;
uint16_t  XPT2046_GetX(void);                                                 // 获取按下的X坐标值; 移植LVGL时，在lv_port_indev.c文件的touchpad_get_xy()调用本函数，可获得(*X)值;
uint16_t  XPT2046_GetY(void);                                                 // 获取按下的Y坐标值; 移植LVGL时，在lv_port_indev.c文件的touchpad_get_xy()调用本函数，可获得(*Y)值;  
// 裸机示范用的函数; LVGL下用不到，可以删除, 不影响底层
void      XPT2046_Cmd(uint8_t status);                                        // 触摸检测开关; 非必要的; 初始化后默认是开启的; 可以手动关闭，暂停检测以实现一些操作，然后再手动开启
void      XPT2046_TouchDown(void);                                            // 空白函数, 调用按下时的处理
void      XPT2046_TouchUp(void);                                              // 空白函数, 调用释放时的处理   




#endif 

