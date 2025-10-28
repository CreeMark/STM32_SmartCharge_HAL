#ifndef __BSP__LCD_ILI93XX_H
#define __BSP__LCD_ILI93XX_H
/**==================================================================================================================
 **���ļ����ơ�  bsp_lcd_2.8_ili93xx.h
 **�����ܲ��ԡ�  ��ʾ��
 **==================================================================================================================
 **������ƽ̨��  STM32F103 + KEIL5.27 + 2.8����ʾ��_ILI9341
 **
 **�����¼�¼��  2024-07-03  �޸ĺ�����ʹ�ø��������ֲLVGL
 **              2024-04-19  ��ֲ��HAL��
 **              2022-05-22  �޸��������ô��룬ʹ�ñ�׼��
 **              2020-12-12  �����޸���Ӣ����ʾ����
 **              2020-09     ����
 **
 **����ע˵����  �����Ȩ��ħŮ�Ƽ����У��������ã�лл��
 **              https://demoboard.taobao.com
====================================================================================================================*/
#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f10x.h"                       // ��׼��
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"                   // HAL��
#endif



/*****************************************************************************
 ** ��ֲ����
****************************************************************************/
// BL_����
#define    LCD_BL_GPIO     GPIOA
#define    LCD_BL_PIN      GPIO_PIN_15
// RD_������
#define    LCD_RD_GPIO     GPIOC
#define    LCD_RD_PIN      GPIO_PIN_6
// WE_д����
#define    LCD_WE_GPIO     GPIOC
#define    LCD_WE_PIN      GPIO_PIN_7
// RS_�л�����\����
#define    LCD_RS_GPIO     GPIOC
#define    LCD_RS_PIN      GPIO_PIN_8
// CS_Ƭѡ
#define    LCD_CS_GPIO     GPIOC
#define    LCD_CS_PIN      GPIO_PIN_9

// ��Ļ����
#define    LCD_WIDTH       240                // ��Ļ������أ�ע�⣺0~239
#define    LCD_HIGH        320                // ��Ļ�߶����أ�ע�⣺0~319
#define    LCD_DIR         6                  // ������ʾ����0-��������3-��������5-������, 6-������



/******************************* ���峣����ɫֵ *****************************/
#define      WHITE               0xFFFF       // ��ɫ
#define      BLACK               0x0000       // ��ɫ 
#define      GREY                0xF7DE       // ��ɫ 
#define      GRAY                0X8430       // ��ɫ
#define      RED                 0xF800       // �� 
#define      MAGENTA             0xF81F       // ���ɫ 
#define      GRED                0xFFE0       // ���ɫ
#define      BROWN               0XBC40       // ��ɫ
#define      BRRED               0XFC07       // �غ�ɫ
#define      GREEN               0x07E0       // �� 
#define      CYAN                0x7FFF       // ��ɫ 
#define      YELLOW              0xFFE0       // ��ɫ 
#define      LIGHTGREEN          0X841F       // ǳ��ɫ 
#define      BLUE                0x001F       // �� 
#define      GBLUE               0x07FF       // ǳ�� 1
#define      LIGHTBLUE           0X7D7C       // ǳ�� 2
#define      BLUE2               0x051F       // ǳ�� 3
#define      GRAYBLUE            0X5458       // ���� 
#define      DARKBLUE            0X01CF       // ����
#define      LGRAY               0XC618       // ǳ��ɫ,���屳��ɫ
#define      LGRAYBLUE           0XA651       // ǳ����ɫ(�м����ɫ)
#define      LBBLUE              0X2B12       // ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)



/*****************************************************************************
 ** ����ȫ�ֺ���

****************************************************************************/
// ����
void LCD_Init(void);                                                                                   // ��ʼ��
void LCD_SetDir(uint8_t dir);                                                                          // ������ʾ����; 0-������1-����
void LCD_DisplayOn(void);                                                                              // ����ʾ
void LCD_DisplayOff(void);                                                                             // ����ʾ
// ��ȡ���ò���
uint8_t  LCD_GetDir(void);                                                                             // ��ȡ ��ǰ����ʾ����: 0-������1-����
uint16_t LCD_GetWidth(void);                                                                           // ��ȡ ��ȴ�С(����); ����ʾ����Ϊ׼
uint16_t LCD_GetHeight(void);                                                                          // ��ȡ �߶ȴ�С(����)
// ��������
void LCD_DrawPoint(uint16_t  x, uint16_t  y, uint16_t _color);                                         // ���㺯��
void LCD_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);                                  // ��Բ
void LCD_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                     // ����
void LCD_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);                // ������
void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);                     // ��䵥ɫ
void LCD_Cross(uint16_t x, uint16_t y, uint16_t len, uint32_t fColor);                                 // ��ʮ����; ��������У׼
// ��չ����
void LCD_String(uint16_t x, uint16_t y, char *pFont, uint8_t size, uint32_t fColor, uint32_t bColor);  // ��ʾ��Ӣ���ַ���
void LCD_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image) ;        // ��ʾͼ��
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData);    // ��ָ������������ݣ�������ͼƬ��LVGL��
// ʾ��
void LCD_GUI(void);                                                                                    // ���Ƽ򵥽���

#endif

