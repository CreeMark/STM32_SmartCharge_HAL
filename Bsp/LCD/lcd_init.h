#ifndef _LCD_INIT_H_
#define _LCD_INIT_H_

#include "spi.h"

/* 定义液晶分辨率 */
#define USE_HORIZONTAL 0 // 设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif


/* 函数说明 */
uint16_t LCD_ReadID(void);                                                         // 获取屏ID
void LCD_SetCursor(uint16_t x, uint16_t y);                                        // 设置光标位置
void LCD_Address_Set(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye);          // 设置坐标函数
void LCD_Fill(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color); // 填充函数
void LCD_Init(void);                                                               // 设置LCD初始化
void LCD_DispFlush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *pData); // LVGL专用刷新函数

/* 背光控制接口 - 符合产品手册要求 */
void LCD_BL_On(void);                                                              // 开启背光
void LCD_BL_Off(void);                                                             // 关闭背光
void LCD_BL_Toggle(void);                                                          // 翻转背光状态

/* 定义画笔颜色 */
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40      // 棕色
#define BRRED 0XFC07      // 棕红色
#define GRAY 0X8430       // 灰色
#define DARKBLUE 0X01CF   // 深蓝色
#define LIGHTBLUE 0X7D7C  // 浅蓝色
#define GRAYBLUE 0X5458   // 灰蓝色
#define LIGHTGREEN 0X841F // 浅绿色
#define LGRAY 0XC618      // 浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE 0XA651  // 浅灰蓝色(中间层颜色)
#define LBBLUE 0X2B12     // 浅棕蓝色(选择条目的反色)

#endif
