#include "spi.h"

/**
 * @brief       SPI接口初始化 - HAL库版本
 * @param       无
 * @retval      无
 * @note        使用HAL库的SPI硬件SPI，SPI1配置已由CubeMX生成
 */
void SPI1_Init(void)
{
#ifdef USE_STDPERIPH_DRIVER
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);                                    // 开启SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);                                   // 开启管脚对应时钟
    /********************配置GPIO***********************/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;          // 配置SCK管脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    // 配置为复用推挡输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 配置IO口翻转速度为50Mhz
    GPIO_Init(GPIOA, &GPIO_InitStructure);             // 初始化SCK管脚
    GPIO_SetBits(GPIOA, GPIO_Pin_5);                   // 拉高SCK管脚

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;          // 配置MOSI管脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    // 配置为复用推挡输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // 配置IO口翻转速度为50Mhz
    GPIO_Init(GPIOA, &GPIO_InitStructure);             // 初始化MOSI管脚
    GPIO_SetBits(GPIOA, GPIO_Pin_7);                   // 拉高MOSI管脚

    /********************配置SPI外设***********************/
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI1挂载APB2(总线速度:72Mhz)4分频为18Mhz
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                       // 第二个时钟沿进行采样
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                        // 空闲时钟状态为高
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  // 数据宽度为8bit
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工模式
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 // 发送数据格式高位在前
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      // SPI1作为主机模式工作
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          // SPI片选通过软件控制
    SPI_Init(SPI1, &SPI_InitStructure);                                // 配置SPI1
    SPI_Cmd(SPI1, ENABLE);                                             // 使能SPI1
#endif
#ifdef USE_HAL_DRIVER
    // HAL库版本：SPI1已由CubeMX生成，这里不需要再次初始化
    // 如果需要修改SPI配置，请在CubeMX中进行
#endif
}

/**
 * @brief       LCD_GPIOInit端口初始化配置 - HAL库版本
 * @param       无
 * @retval      无
 */
void LCD_GPIOInit(void)
{
#ifdef USE_STDPERIPH_DRIVER
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    SPI1_Init();
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  // RES
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  // DC
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // CS
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // BLK
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef USE_HAL_DRIVER
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pins : RES DC CS BLK */
    GPIO_InitStruct.Pin = LCD_RES_PIN | LCD_DC_PIN | LCD_CS_PIN | LCD_BLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif
    
    // 初始化状态：LCD CS拉高（未选中），背光打开
    LCD_CS_Set();
    LCD_BLK_Set();
}

/**
 * @brief       硬件SPI发送一个字节数据（优化版，CS由上层控制）
 * @param       dat: 需要发送的字节数据
 * @retval      无
 */
void LCD_WR_Bus(uint8_t dat)
{
#ifdef USE_STDPERIPH_DRIVER
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); // 等待发送缓冲区空
    SPI_I2S_SendData(SPI1, dat);                                   // SPI外设发送数据
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);  // 等待发送完成
#endif

#ifdef USE_HAL_DRIVER
    // HAL库版本: 直接操作SPI寄存器
    while((SPI1->SR & SPI_SR_TXE) == 0);  // 等待发送缓冲区空
    SPI1->DR = dat;                       // 发送数据
    while((SPI1->SR & SPI_SR_BSY) != 0);  // 等待发送完成
#endif
}

/**
 * @brief       向液晶写寄存器命令
 * @param       reg: 要写的命令
 * @retval      无
 */
void LCD_WR_REG(uint8_t reg)
{
    LCD_CS_Clr();  // 选中LCD
    LCD_DC_Clr();  // 命令模式
    LCD_WR_Bus(reg);
    LCD_CS_Set();  // 释放LCD
}

/**
 * @brief       向液晶写一个字节数据
 * @param       dat: 要写的数据
 * @retval      无
 */
void LCD_WR_DATA8(uint8_t dat)
{
    LCD_CS_Clr();  // 选中LCD
    LCD_DC_Set();  // 数据模式
    LCD_WR_Bus(dat);
    LCD_CS_Set();  // 释放LCD
}

/**
 * @brief       向液晶写一个半字数据
 * @param       dat: 要写的数据
 * @retval      无
 */
void LCD_WR_DATA(uint16_t dat)
{
    LCD_CS_Clr();  // 选中LCD
    LCD_DC_Set();  // 数据模式
    LCD_WR_Bus(dat >> 8);
    LCD_WR_Bus(dat & 0xFF);
    LCD_CS_Set();  // 释放LCD
}
