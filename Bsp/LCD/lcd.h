#ifndef _LCD_H_
#define _LCD_H_

#include "lcd_init.h"

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);                                                                                // 画点函数
void LCD_DrawLine(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);                                                     // 划线函数
void LCD_DrawRectangle(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);                                                // 绘制空心矩形
void LCD_DrawFillRectangle(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);                                            // 绘制实心矩形
void Draw_Circle(int xc, int yc, int x, int y, uint16_t color);                                                                            // 8分法画圆内部调用
void LCD_DrawCircle(uint16_t xc, uint16_t yc, uint16_t r, uint16_t color, uint16_t mode);                                                  // 绘制圆形
void LCD_DrawTriangel(uint16_t x, uint16_t y, uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);                         // 绘制空心三角形
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                             // 显示字符函数
void LCD_ShowString(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint16_t sizey, uint8_t mode);                           // 显示字符串
uint32_t mypow(uint8_t m, uint8_t n);                                                                                                      // 幂运算内部调用
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                // 显示数字
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t pre, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode); // 显示浮点数
void LCD_ShowChinese12x12(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                      // 显示12x12字符串
void LCD_ShowChinese16x16(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                      // 显示16x16字符串
void LCD_ShowChinese24x24(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                      // 显示24x24字符串
void LCD_ShowChinese32x32(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                      // 显示32x32字符串
void LCD_ShowChinese(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                           // 显示中文字符串
void LCD_ShowStr(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                               // 中英混显
void LCD_StrCenter(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);                             // 居中显示
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);                                        // 显示图片

/* XPT2046触摸屏校准辅助函数 */
void LCD_String(uint16_t x, uint16_t y, const char *s, uint8_t size, uint16_t fc, uint16_t bc);                                            // 简化版字符串显示
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint16_t color);                                                                      // 绘制十字准星
#endif
