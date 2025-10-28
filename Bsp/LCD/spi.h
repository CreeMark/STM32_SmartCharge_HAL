#ifndef _SPI_H_
#define _SPI_H_

#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f10x.h"                       // 标准库
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"                   // HAL库
#endif

/* 定义管脚端口 - HAL库版本 */
/* 引脚分配:
 * SPI总线(PA5-SCK/PA7-MOSI)与W25Q128共用SPI1
 * 控制引脚使用PB口避免冲突: RES→PB0, DC→PB1, CS→PB10, BLK→PB11
 */
#define LCD_SCK_GPIO        GPIOA
#define LCD_SCK_PIN         GPIO_PIN_5

#define LCD_MOSI_GPIO       GPIOA
#define LCD_MOSI_PIN        GPIO_PIN_7

#define LCD_RES_GPIO        GPIOB
#define LCD_RES_PIN         GPIO_PIN_0

#define LCD_DC_GPIO         GPIOB
#define LCD_DC_PIN          GPIO_PIN_1

#define LCD_CS_GPIO         GPIOB
#define LCD_CS_PIN          GPIO_PIN_10

#define LCD_BLK_GPIO        GPIOB
#define LCD_BLK_PIN         GPIO_PIN_11

/* 定义端口电平状态 - 兼容HAL库 */
#define LCD_SCK_Clr()   HAL_GPIO_WritePin(LCD_SCK_GPIO, LCD_SCK_PIN, GPIO_PIN_RESET)
#define LCD_SCK_Set()   HAL_GPIO_WritePin(LCD_SCK_GPIO, LCD_SCK_PIN, GPIO_PIN_SET)

#define LCD_MOSI_Clr()  HAL_GPIO_WritePin(LCD_MOSI_GPIO, LCD_MOSI_PIN, GPIO_PIN_RESET)
#define LCD_MOSI_Set()  HAL_GPIO_WritePin(LCD_MOSI_GPIO, LCD_MOSI_PIN, GPIO_PIN_SET)

#define LCD_RES_Clr()   HAL_GPIO_WritePin(LCD_RES_GPIO, LCD_RES_PIN, GPIO_PIN_RESET)
#define LCD_RES_Set()   HAL_GPIO_WritePin(LCD_RES_GPIO, LCD_RES_PIN, GPIO_PIN_SET)

#define LCD_DC_Clr()    HAL_GPIO_WritePin(LCD_DC_GPIO, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_Set()    HAL_GPIO_WritePin(LCD_DC_GPIO, LCD_DC_PIN, GPIO_PIN_SET)

#define LCD_CS_Clr()    HAL_GPIO_WritePin(LCD_CS_GPIO, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_Set()    HAL_GPIO_WritePin(LCD_CS_GPIO, LCD_CS_PIN, GPIO_PIN_SET)

#define LCD_BLK_Clr()   HAL_GPIO_WritePin(LCD_BLK_GPIO, LCD_BLK_PIN, GPIO_PIN_RESET)
#define LCD_BLK_Set()   HAL_GPIO_WritePin(LCD_BLK_GPIO, LCD_BLK_PIN, GPIO_PIN_SET)

/* 函数声明 */
void SPI1_Init(void);                   // SPI1初始化
void SPI1_WriteByte(uint8_t dat);       // SPI写单字节
void LCD_GPIOInit(void);                // LCD GPIO初始化
void LCD_WR_Bus(uint8_t dat);           // LCD写总线
void LCD_WR_REG(uint8_t reg);           // LCD写寄存器
void LCD_WR_DATA8(uint8_t dat);         // LCD写8位数据
void LCD_WR_DATA(uint16_t dat);         // LCD写16位数据

#endif



