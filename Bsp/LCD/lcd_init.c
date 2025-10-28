#include "lcd_init.h"

#ifdef USE_STDPERIPH_DRIVER
    #include "dma.h"       // 标准库DMA
    #include "delay.h"     // 标准库延时
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f1xx_hal.h"  // HAL库版本
    #define delay_ms(x)  HAL_Delay(x)
#endif

#include "stdio.h"

/**
 * @brief       设置光标位置
 * @param       x:坐标列地址
 * @param       y:坐标行地址
 * @retval      无
 */
void LCD_SetCursor(uint16_t x, uint16_t y)
{
    LCD_Address_Set(x, y, x, y);
}

/**
 * @brief       设置显示窗口 - ST7735S版本 (修复边界处理)
 * @param       xs:窗口列起始地址(包含)
 * @param       ys:坐标行起始地址(包含)
 * @param       xe:窗口列结束地址(包含)
 * @param       ye:坐标行结束地址(包含)
 * @retval      无
 */
void LCD_Address_Set(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
    if(USE_HORIZONTAL == 0)
    {
        LCD_WR_REG(0x2a); // 列地址设置
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xs + 2);  // ST7735S的列偏移+2
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xe + 2);
        LCD_WR_REG(0x2b); // 行地址设置
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ys + 1);  // ST7735S的行偏移+1
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ye + 1);
        LCD_WR_REG(0x2c); // 储存器写
    }
    else if(USE_HORIZONTAL == 1)
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xs + 2);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xe + 2);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ys + 1);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ye + 1);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL == 2)
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xs + 1);  // 横屏模式列偏移+1
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xe + 1);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ys + 2);  // 横屏模式行偏移+2
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ye + 2);
        LCD_WR_REG(0x2c);
    }
    else
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xs + 1);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(xe + 1);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ys + 2);
        LCD_WR_DATA8(0x00);
        LCD_WR_DATA8(ye + 2);
        LCD_WR_REG(0x2c);
    }
}


/**
 * @brief       指定颜色填充区域
 * @param       xs:填充区域列起始地址
 * @param       ys:填充区域行起始地址
 * @param       xe:填充区域列结束地址
 * @param       ye:填充区域行结束地址
 * @param       color:填充颜色值
 * @retval      无
 */
void LCD_Fill(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color)
{
#ifdef USE_STDPERIPH_DRIVER
    // 标准库版本 - 使用DMA加速
    uint16_t color1[1], t = 1;
    uint32_t num, num1;
    color1[0] = color;
    num = (xe - xs + 1) * (ye - ys + 1);  // 修复：坐标包含端点，需要+1
    LCD_Address_Set(xs, ys, xe, ye);      // 修复：不需要-1，传入实际结束坐标
    LCD_CS_Clr();
    SPI1->CR1 |= 1 << 11;  // 16位模式
    SPI_Cmd(SPI1, ENABLE);
    while (t)
    {
        if (num > 65534)
        {
            num -= 65534;
            num1 = 65534;
        }
        else
        {
            t = 0;
            num1 = num;
        }
        MYDMA_Config1(DMA1_Channel3, (u32)&SPI1->DR, (u32)color1, num1);
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
        MYDMA_Enable(DMA1_Channel3);
        while (1)
        {
            if (DMA_GetFlagStatus(DMA1_FLAG_TC3) != RESET)
            {
                DMA_ClearFlag(DMA1_FLAG_TC3);
                break;
            }
        }
    }
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY)==SET);
    LCD_CS_Set();
    SPI1->CR1 = ~SPI1->CR1;
    SPI1->CR1 |= 1 << 11;
    SPI1->CR1 = ~SPI1->CR1;
    SPI_Cmd(SPI1, ENABLE);
#endif

#ifdef USE_HAL_DRIVER
    // HAL库版本 - 使用软件填充（后续可优化为DMA）
    uint32_t total_pixels = (xe - xs + 1) * (ye - ys + 1);  // 修复：坐标包含端点，需要+1
    
    LCD_Address_Set(xs, ys, xe, ye);  // 修复：不需要-1，传入实际结束坐标
    LCD_CS_Clr();
    LCD_DC_Set();
    
    // 逐像素发送颜色数据
    for(uint32_t i = 0; i < total_pixels; i++)
    {
        LCD_WR_Bus(color >> 8);     // 高字节
        LCD_WR_Bus(color & 0xFF);   // 低字节
    }
    
    LCD_CS_Set();
#endif
}



/**
 * @brief       初始化LCD - ST7735S驱动
 * @param       无
 * @retval      无
 * @note        严格按照产品手册复位时序: RES低100ms→RES高100ms→初始化
 */
void LCD_Init(void)
{
    LCD_GPIOInit();
    
    // 硬件复位 - 符合产品手册要求
    LCD_RES_Set();
    delay_ms(100);  // 稳定高电平
    LCD_RES_Clr();  // 拉低复位
    delay_ms(100);  // 保持100ms低电平
    LCD_RES_Set();  // 释放复位
    delay_ms(120);  // 等待芜片就绪
    
    LCD_BLK_Set();  // 开启背光

    // ST7735S 初始化序列
    LCD_WR_REG(0x11);  // Sleep out
    delay_ms(120);     // Delay 120ms

    // ST7735S Frame Rate
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);

    // Dot inversion
    LCD_WR_REG(0xB4);
    LCD_WR_DATA8(0x03);

    // ST7735S Power Sequence
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1);
    LCD_WR_DATA8(0xC0);
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0xEE);

    // VCOM
    LCD_WR_REG(0xC5);
    LCD_WR_DATA8(0x1A);

    // ST7735S Gamma Sequence
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x22);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x30);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x13);

    // 颜色模式16bit
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    // 显示方向
    LCD_WR_REG(0x36);
    if (USE_HORIZONTAL == 0)
    {
        LCD_WR_DATA8(0x08);  // 竖屏正常
    }
    else if (USE_HORIZONTAL == 1)
    {
        LCD_WR_DATA8(0xC8);  // 竖屏180度旋转
    }
    else if (USE_HORIZONTAL == 2)
    {
        LCD_WR_DATA8(0x78);  // 横屏正常
    }
    else
    {
        LCD_WR_DATA8(0xA8);  // 横屏180度旋转
    }

    // Display On
    LCD_WR_REG(0x29);
    delay_ms(50);
}

/**
 * @brief       LVGL专用刷新函数 - 将数据写入指定区域 (修复像素计算)
 * @param       x1,y1: 起始坐标
 * @param       x2,y2: 结束坐标
 * @param       pData: 颜色数据指针(16位RGB565格式)
 * @retval      无
 */
void LCD_DispFlush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *pData)
{
    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);  // 修复：坐标包含端点，需要+1
    
    LCD_Address_Set(x1, y1, x2, y2);  // 设置刷新区域
    
    LCD_CS_Clr();   // 选中LCD
    LCD_DC_Set();   // 数据模式
    
    // 逐像素发送RGB565数据
    for(uint32_t i = 0; i < size; i++)
    {
        LCD_WR_Bus(pData[i] >> 8);     // 高字节
        LCD_WR_Bus(pData[i] & 0xFF);   // 低字节
    }
    
    LCD_CS_Set();   // 释放LCD
}

/**
 * @brief       开启背光 - 符合产品手册BLK控制要求
 * @param       无
 * @retval      无
 */
void LCD_BL_On(void)
{
    LCD_BLK_Set();  // BLK引脚拉高，背光开启
}

/**
 * @brief       关闭背光 - 符合产品手册BLK控制要求
 * @param       无
 * @retval      无
 */
void LCD_BL_Off(void)
{
    LCD_BLK_Clr();  // BLK引脚拉低，背光关闭
}

/**
 * @brief       翻转背光状态
 * @param       无
 * @retval      无
 */
void LCD_BL_Toggle(void)
{
    HAL_GPIO_TogglePin(LCD_BLK_GPIO, LCD_BLK_PIN);
}

