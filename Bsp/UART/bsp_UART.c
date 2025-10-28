/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【代码更新】
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 **【文件名称】  bsp_UART.c
 **
 **【文件功能】  各USART的GPIO配置、通信协议配置、中断配置，及功能函数实现
 **
 **【适用平台】  STM32F103 + HAL库 + keil5
 **
************************************************************************************************************************************/
#include "bsp_UART.h"     // 头文件
#include <stdarg.h>       // 用于支持不定长参数






/*****************************************************************************
 ** 声明本地变量
****************************************************************************/
typedef struct
{
    uint16_t  usRxNum;            // 新一帧数据，接收到多少个字节数据
    uint8_t  *puRxData;           // 新一帧数据，数据缓存; 存放的是空闲中断后，从临时接收缓存复制过来的完整数据，并非接收过程中的不完整数据;

    uint8_t  *puTxFiFoData;       // 发送缓冲区，环形队列; 为了方便理解阅读，没有封装成队列函数
    uint16_t  usTxFiFoData ;      // 环形缓冲区的队头
    uint16_t  usTxFiFoTail ;      // 环形缓冲区的队尾
} xUSATR_TypeDef;



#if 0
/******************************************************************************
 * 函  数： delay_us
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟72MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  us  微秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesUS;    // 使用volatile声明，防止变量被编译器优化
static void delay_us(uint16_t us)
{
    ulTimesUS = us ;
    while (ulTimesUS)
        ulTimesUS--;                   // 操作外部变量，防止空循环被编译器优化掉
}
#endif



/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟72MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 650;              
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}



//////////////////////////////////////////////////////////////   USART-1   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART1_EN

static xUSATR_TypeDef  xUART1 = { 0 };                      // 定义 UART1 的收发结构体
static uint8_t uaUART1RxData[UART1_RX_BUF_SIZE];            // 定义 UART1 的接收缓存
static uint8_t uaUART1TxFiFoData[UART1_TX_BUF_SIZE];        // 定义 UART1 的发送缓存

/******************************************************************************
 * 函  数： UART1_Init
 * 功  能： 初始化USART1的通信引脚、协议参数、中断优先级
 *          引脚：TX-PA10、RX-PA11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t  ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART1_Init(uint32_t ulBaudrate)
{
    // 使能相关时钟
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                       // 使能外设：USART1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                         // 使能GPIO：GPIOA                                                         
    
    #ifdef USE_STDPERIPH_DRIVER  
    // TX引脚 配置                                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                                      // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;                                      // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                                 // 引脚功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                // 输出速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                           // 初始化引脚
    // RX引脚 配置                                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                                     // RX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;                                   // RX引脚工作模式：上拉输入; 如果使用浮空输入，引脚空置时可能产生误输入; 当电路上为一主多从电路时，可以使用复用开漏模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                           // 初始化引脚
    // 中断 配置                                                                     
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                                      // 优先级配置结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART1_IRQn;                               // 中断编号
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 2 ;                      // 抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;                              // 子优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                                 // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                  // 初始化中断优先级
    //USART 设置                                                                     
    USART_DeInit(USART1);                                                            // 复位串口
    USART_InitTypeDef USART_InitStructure = {0};                                     // UART配置结构体 
    USART_InitStructure.USART_BaudRate   = ulBaudrate;                               // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 位长
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;                         // 停止位
    USART_InitStructure.USART_Parity     = USART_Parity_No;                          // 奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 使能工作模式
    USART_Init(USART1, &USART_InitStructure);                                        // 初始化串口
    // 打开中断
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                                   // 发送中断 使能
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                   // 接收中断 使能
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                                   // 空闲中断 使能
    // 使能串口
    USART_Cmd(USART1, ENABLE);                                                       // 使能串口, 开始工作
#endif  
    
#ifdef USE_HAL_DRIVER  
    // 关闭串口                                                 
    USART1 -> CR1  =   0;                                       // 关闭串口，配置清零
    // 配置TX引脚
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                  // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_9 ;                        // 引脚 TX-PA9
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // 初始化引脚工作模式
    // 配置RX引脚                                               
    GPIO_InitStruct.Pin   =  GPIO_PIN_10;                       // 引脚 RX-PA10
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // 初始化引脚工作模式                                                                
    // 中断优选级配置                                           
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);                    // 配置中断线的优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);                            // 使能中断线
    // 计算波特率参数
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                    // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)SystemCoreClock  / (ulBaudrate * 16);         // 波特率公式计算; USART1挂载在APB2, 时钟为系统时钟的1分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                            // 整数部分
    fraction = (temp - mantissa) * 16;                          // 小数部分
    USART1 -> BRR  = mantissa << 4 | fraction;                  // 设置波特率
    // 串口通信参数配置
    USART1 -> CR1  =   0;                                       // 清0
    USART1 -> CR1 |=   0x01 << 2;                               // 接收使能[02]: 0=失能、1=使能
    USART1 -> CR1 |=   0x01 << 3;                               // 发送使能[03]：0=失能、1=使能
    USART1 -> CR1 |=   0x00 << 9;                               // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART1 -> CR1 |=   0x00 << 10;                              // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART1 -> CR1 |=   0x00 << 12;                              // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART1 -> CR2  =   0;                                       // 数据清0
    USART1 -> CR2 &= ~(0x03 << 12);                             // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART1 -> CR3  =   0;                                       // 数据清0
    USART1 -> CR3 &= ~(0x01 << 6);                              // DMA接收[6]: 0=禁止、1=使能
    USART1 -> CR3 &= ~(0x01 << 7);                              // DMA发送[7]: 0=禁止、1=使能
    // 串口中断设置
    USART1 -> CR1 &= ~(0x01 << 7);                              // 关闭发送中断
    USART1 -> CR1 |=   0x01 << 5;                               // 使能接收中断: 接收缓冲区非空
    USART1 -> CR1 |=   0x01 << 4;                               // 使能空闲中断：超过1字节时间没收到新数据
    USART1 -> SR   = ~(0x00F0);                                 // 清理中断
    // 打开串口
    USART1 -> CR1 |=   0x01 << 13;                              // 使能UART开始工作
#endif

    // 清理中断
    USART1->SR = ~(0x00F0);                                     // 清理中断
    // 关联缓冲区
    xUART1.puRxData = uaUART1RxData;                            // 关联接收缓冲区的地址
    xUART1.puTxFiFoData = uaUART1TxFiFoData;                    // 关联发送缓冲区的地址
    // 输出提示
    printf("\r\r\r===========  STM32F103 Peripheral Init Report ===========\r");              // 输出到串口助手
    SystemCoreClockUpdate();                                                                  // 更新一下系统运行频率变量
    printf("System Clock Freq      %d MHz\r", SystemCoreClock / 1000000);                   // 输出到串口助手
    printf("UART1 Config           %d-None-8-1; Init & TX/RX Ready\r", ulBaudrate);         // 输出到串口助手
}

/******************************************************************************
 * 函  数： USART1_IRQHandler
 * 功  能： USART1的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
******************************************************************************/
void USART1_IRQHandler(void)
{
    static uint16_t cnt = 0;                                         // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART1_RX_BUF_SIZE];                       // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUSART.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((USART1->SR & 1 << 7) && (USART1->CR1 & 1 << 7))             // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART1->DR = xUART1.puTxFiFoData[xUART1.usTxFiFoTail++];     // 把要发送的字节，放入USART的发送寄存器
        if (xUART1.usTxFiFoTail == UART1_TX_BUF_SIZE)                // 如果数据指针到了尾部，就重新标记到0
            xUART1.usTxFiFoTail = 0;
        if (xUART1.usTxFiFoTail == xUART1.usTxFiFoData)
            USART1->CR1 &= ~(0x01 << 7);                             // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (USART1->SR & (0x01 << 5))                                    // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART1_RX_BUF_SIZE))//||(xUART1.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("Warning: UART1 RX buffer overflow!\r");
            USART1->DR;                                              // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART1->DR ;                                 // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位；
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (USART1->SR & (0x01 << 4))                                    // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART1.usRxNum  = 0;                                         // 把接收到的数据字节数清0
        memcpy(xUART1.puRxData, rxTemp, UART1_RX_BUF_SIZE);          // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART1.usRxNum  = cnt;                                       // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                     // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART1_RX_BUF_SIZE);                        // 接收数据缓存数组，清零; 准备下一次的接收
        USART1 ->SR;
        USART1 ->DR;                                                 // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART1_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t*  puData   需发送数据的地址
 *          uint16_t  usNum    发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART1_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART1.puTxFiFoData[xUART1.usTxFiFoData++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART1.usTxFiFoData == UART1_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART1.usTxFiFoData = 0;
    }                                                            // 为了方便阅读理解，这里没有把此部分封装成队列函数，可以自行封装

    if ((USART1->CR1 & 1 << 7) == 0)                             // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART1->CR1 |= 1 << 7;                                   // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART1_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART1_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                     // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                                   // 新建一个可变参数列表
    va_start(ap, pcString);                                       // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                        // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                                   // 清空可变参数列表
    UART1_SendData((uint8_t *)mBuffer, strlen(mBuffer));          // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART1_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART1_ClearRx();                                              // 清0
    UART1_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART1_GetRxNum())                                     // 判断是否接收到数据
        {
            UART1_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART1_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART1_SendStringForDMA
 * 功  能： UART通过DMA发送数据，省了占用中断的时间
 *         【适合场景】字符串，字节数非常多，
 *         【不 适 合】1:只适合发送字符串，不适合发送可能含0的数值类数据; 2-时间间隔要足够
 * 参  数： char strintTemp  要发送的字符串首地址
 * 返回值： 无
 * 备  注:  本函数为保留函数，留作用户参考。为了方便移植，本文件对外不再使用本函数。
 ******************************************************************************/
#if 0
void UART1_SendStringForDMA(char *stringTemp)
{
    static uint8_t Flag_DmaTxInit = 0;                // 用于标记是否已配置DMA发送
    uint32_t   num = 0;                               // 发送的数量，注意发送的单位不是必须8位的
    char *t = stringTemp ;                            // 用于配合计算发送的数量

    while (*t++ != 0)  num++;                         // 计算要发送的数目，这步比较耗时，测试发现每多6个字节，增加1us，单位：8位

    while (DMA1_Channel4->CNDTR > 0);                 // 重要：如果DMA还在进行上次发送，就等待; 得进完成中断清标志，F4不用这么麻烦，发送完后EN自动清零
    if (Flag_DmaTxInit == 0)                          // 是否已进行过USAART_TX的DMA传输配置
    {
        Flag_DmaTxInit  = 1;                          // 设置标记，下次调用本函数就不再进行配置了
        USART1 ->CR3   |= 1 << 7;                     // 使能DMA发送
        RCC->AHBENR    |= 1 << 0;                     // 开启DMA1时钟  [0]DMA1   [1]DMA2

        DMA1_Channel4->CCR   = 0;                     // 失能， 清0整个寄存器, DMA必须失能才能配置
        DMA1_Channel4->CNDTR = num;                   // 传输数据量
        DMA1_Channel4->CMAR  = (uint32_t)stringTemp;  // 存储器地址
        DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR; // 外设地址

        DMA1_Channel4->CCR |= 1 << 4;                 // 数据传输方向   0:从外设读   1:从存储器读
        DMA1_Channel4->CCR |= 0 << 5;                 // 循环模式       0:不循环     1：循环
        DMA1_Channel4->CCR |= 0 << 6;                 // 外设地址非增量模式
        DMA1_Channel4->CCR |= 1 << 7;                 // 存储器增量模式
        DMA1_Channel4->CCR |= 0 << 8;                 // 外设数据宽度为8位
        DMA1_Channel4->CCR |= 0 << 10;                // 存储器数据宽度8位
        DMA1_Channel4->CCR |= 0 << 12;                // 中等优先级
        DMA1_Channel4->CCR |= 0 << 14;                // 非存储器到存储器模式
    }
    DMA1_Channel4->CCR  &= ~((uint32_t)(1 << 0));     // 失能，DMA必须失能才能配置
    DMA1_Channel4->CNDTR = num;                       // 传输数据量
    DMA1_Channel4->CMAR  = (uint32_t)stringTemp;      // 存储器地址
    DMA1_Channel4->CCR  |= 1 << 0;                    // 开启DMA传输
}
#endif

/******************************************************************************
 * 函  数： UART1_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART1_GetRxNum(void)
{
    return xUART1.usRxNum ;
}

/******************************************************************************
 * 函  数： UART1_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 缓存地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART1_GetRxData(void)
{
    return xUART1.puRxData ;
}

/******************************************************************************
 * 函  数： UART1_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART1_ClearRx(void)
{
    xUART1.usRxNum = 0 ;
}
#endif  // endif UART1_EN





//////////////////////////////////////////////////////////////   USART-2   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART2_EN

static xUSATR_TypeDef  xUART2 = { 0 };                      // 定义 UART2 的收发结构体
static uint8_t uaUART2RxData[UART2_RX_BUF_SIZE];            // 定义 UART2 的接收缓存
static uint8_t uaUART2TxFiFoData[UART2_TX_BUF_SIZE];        // 定义 UART2 的发送缓存

/******************************************************************************
 * 函  数： UART2_Init
 * 功  能： 初始化USART2的通信引脚、协议参数、中断优先级
 *          引脚：TX-PA2、RX-PA3
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t  ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART2_Init(uint32_t ulBaudrate)
{
    // 使能相关时钟
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;                       // 使能外设：USART2
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                         // 使能GPIO：GPIOA
             
#ifdef USE_STDPERIPH_DRIVER                                                          // 标准库 配置
    // TX引脚 配置                                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                                      // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                                      // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                                 // 引脚功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                // 输出速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                           // 初始化引脚
    // RX引脚 配置                                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;                                      // RX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;                                   // RX引脚工作模式：上拉输入; 如果使用浮空输入，引脚空置时可能产生误输入; 当电路上为一主多从电路时，可以使用复用开漏模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                           // 初始化引脚
    // 中断 配置                                                                     
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                                      // 优先级配置结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART2_IRQn;                               // 中断编号
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 2 ;                      // 抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;                              // 子优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                                 // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                  // 初始化中断优先级
    //USART 设置                                                                     
    USART_DeInit(USART2);                                                            // 复位串口
    USART_InitTypeDef USART_InitStructure = {0};                                     // UART配置结构体 
    USART_InitStructure.USART_BaudRate   = ulBaudrate;                               // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 位长
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;                         // 停止位
    USART_InitStructure.USART_Parity     = USART_Parity_No;                          // 奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 使能工作模式
    USART_Init(USART2, &USART_InitStructure);                                        // 初始化串口
    // 打开中断
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);                                   // 发送中断 使能
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                                   // 接收中断 使能
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                                   // 空闲中断 使能
    // 使能串口
    USART_Cmd(USART2, ENABLE);                                                       // 使能串口, 开始工作
#endif   

#ifdef USE_HAL_DRIVER                                           // HAL库 配置    
    // 关闭串口                                                 
    USART2 -> CR1  =   0;                                       // 关闭串口，配置清零                                                                
    // TX引脚 配置                                              
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                  // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_2 ;                        // 引脚 TX-PA2
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // 初始化引脚工作模式
    // RX引脚配置                                               
    GPIO_InitStruct.Pin   =  GPIO_PIN_3;                        // 引脚 RX-PA3
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                     // 初始化引脚工作模式                                                                
    // 中断优选级配置                                           
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);                    // 配置中断线的优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);                            // 使能中断线                                                                
    // 计算波特率参数                                           
    float    temp;                                              
    uint16_t mantissa, fraction;                                
    SystemCoreClockUpdate();                                    // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);    // 波特率公式计算; USART2挂载在APB1, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                            // 整数部分
    fraction = (temp - mantissa) * 16;                          // 小数部分
    USART2 -> BRR  = mantissa << 4 | fraction;                  // 设置波特率
    // 串口通信参数配置
    USART2 -> CR1  =   0;                                       // 清0
    USART2 -> CR1 |=   0x01 << 2;                               // 接收使能[02]: 0=失能、1=使能
    USART2 -> CR1 |=   0x01 << 3;                               // 发送使能[03]：0=失能、1=使能
    USART2 -> CR1 |=   0x00 << 9;                               // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART2 -> CR1 |=   0x00 << 10;                              // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART2 -> CR1 |=   0x00 << 12;                              // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART2 -> CR2  =   0;                                       // 数据清0
    USART2 -> CR2 &= ~(0x03 << 12);                             // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART2 -> CR3  =   0;                                       // 数据清0
    USART2 -> CR3 &= ~(0x01 << 6);                              // DMA接收[6]: 0=禁止、1=使能
    USART2 -> CR3 &= ~(0x01 << 7);                              // DMA发送[7]: 0=禁止、1=使能
    // 串口中断设置
    USART2 -> CR1 &= ~(0x01 << 7);                              // 关闭发送中断
    USART2 -> CR1 |=   0x01 << 5;                               // 使能接收中断: 接收缓冲区非空
    USART2 -> CR1 |=   0x01 << 4;                               // 使能空闲中断：超过1字节时间没收到新数据
    USART2 -> SR   = ~(0x00F0);                                 // 清理中断
    // 开启USART2
    USART2 -> CR1 |=   0x01 << 13;                              // 使能UART开始工作
#endif

    // 清理中断
    USART2->SR = ~(0x00F0);                                     // 清理中断
    // 关联缓冲区
    xUART2.puRxData = uaUART2RxData;                            // 获取接收缓冲区的地址
    xUART2.puTxFiFoData = uaUART2TxFiFoData;                    // 获取发送缓冲区的地址
    // 输出提示
    printf("UART2 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： USART2_IRQHandler
 * 功  能： USART2的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
/*
 * 注意：此函数在本工程中已禁用。
 * 原因：USART2 由 CubeMX/HAL 配置，并在 stm32f1xx_it.c 中实现了中断处理（用于 HLW8032 解析）。
 * 如需恢复本 BSP 的 USART2 中断处理，请取消下方 #if 0，并同时注释掉 stm32f1xx_it.c 中的 USART2_IRQHandler。
 */
#if 0
void USART2_IRQHandler(void)
{
    static uint16_t cnt = 0;                                        // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART2_RX_BUF_SIZE];                      // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((USART2->SR & 1 << 7) && (USART2->CR1 & 1 << 7))            // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART2->DR = xUART2.puTxFiFoData[xUART2.usTxFiFoTail++];    // 把要发送的字节，放入USART的发送寄存器
        if (xUART2.usTxFiFoTail == UART2_TX_BUF_SIZE)               // 如果数据指针到了尾部，就重新标记到0
            xUART2.usTxFiFoTail = 0;
        if (xUART2.usTxFiFoTail == xUART2.usTxFiFoData)
            USART2->CR1 &= ~(1 << 7);                               // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (USART2->SR & (1 << 5))                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART2_RX_BUF_SIZE))//||xUART2.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("警告：UART2单帧接收量，已超出接收缓存大小\r!");
            USART2->DR;                                             // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART2->DR ;                                // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位；
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (USART2->SR & (1 << 4))                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART2.usRxNum  = 0;                                        // 把接收到的数据字节数清0
        memcpy(xUART2.puRxData, rxTemp, UART2_RX_BUF_SIZE);         // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART2.usRxNum  = cnt;                                      // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                    // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART2_RX_BUF_SIZE);                       // 接收数据缓存数组，清零; 准备下一次的接收
        USART2 ->SR;
        USART2 ->DR;                                                // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}
#endif

/******************************************************************************
 * 函  数： UART2_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData     需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART2_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART2.puTxFiFoData[xUART2.usTxFiFoData++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART2.usTxFiFoData == UART2_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART2.usTxFiFoData = 0;
    }

    if ((USART2->CR1 & 1 << 7) == 0)                             // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART2->CR1 |= 1 << 7;                                   // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART2_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART2_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                             // 新建一个可变参数列表
    va_start(ap, pcString);                                 // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                  // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                             // 清空可变参数列表
    UART2_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART2_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART2_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART2_ClearRx();                                              // 清0
    UART2_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART2_GetRxNum())                                     // 判断是否接收到数据
        {
            UART2_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART2_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART2_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART2_GetRxNum(void)
{
    return xUART2.usRxNum ;
}

/******************************************************************************
 * 函  数： UART2_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART2_GetRxData(void)
{
    return xUART2.puRxData ;
}

/******************************************************************************
 * 函  数： UART2_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART2_ClearRx(void)
{
    xUART2.usRxNum = 0 ;
}
#endif  // endif UART2_EN





//////////////////////////////////////////////////////////////   USART-3   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART3_EN

static xUSATR_TypeDef  xUART3 = { 0 };                      // 定义 UART3 的收发结构体
static uint8_t uaUart3RxData[UART3_RX_BUF_SIZE];            // 定义 UART3 的接收缓存
static uint8_t uaUart3TxFiFoData[UART3_TX_BUF_SIZE];        // 定义 UART3 的发送缓存

/******************************************************************************
 * 函  数： UART3_Init
 * 功  能： 初始化USART3的通信引脚、协议参数、中断优先级
 *          引脚：TX-PB10、RX-PB11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART3_Init(uint32_t ulBaudrate)
{
    // 使能相关时钟
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                       // 使能外设：USART3
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                         // 使能GPIO：GPIOB
                   
#ifdef USE_STDPERIPH_DRIVER                                                          // 标准库 配置
    // TX引脚 配置                                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                                      // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                                     // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                                 // 引脚功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                // 输出速度
    GPIO_Init(GPIOB, &GPIO_InitStructure);                                           // 初始化引脚
    // RX引脚 配置                                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                                     // RX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;                                   // RX引脚工作模式：上拉输入; 如果使用浮空输入，引脚空置时可能产生误输入; 当电路上为一主多从电路时，可以使用复用开漏模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);                                           // 初始化引脚
    // 中断 配置                                                                     
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                                      // 优先级配置结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART3_IRQn;                               // 中断编号
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 2 ;                      // 抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;                              // 子优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                                 // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                  // 初始化中断优先级
    //USART 设置                                                                     
    USART_DeInit(USART3);                                                            // 复位串口
    USART_InitTypeDef USART_InitStructure = {0};                                     // UART配置结构体 
    USART_InitStructure.USART_BaudRate   = ulBaudrate;                               // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 位长
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;                         // 停止位
    USART_InitStructure.USART_Parity     = USART_Parity_No;                          // 奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 使能工作模式
    USART_Init(USART3, &USART_InitStructure);                                        // 初始化串口
    // 打开中断
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);                                   // 发送中断 使能
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                   // 接收中断 使能
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                                   // 空闲中断 使能
    // 使能串口
    USART_Cmd(USART3, ENABLE);                                                       // 使能串口, 开始工作
#endif
    
#ifdef USE_HAL_DRIVER      
    // 关闭串口                                                 
    USART3 -> CR1  =   0;                                       // 关闭串口，配置清零                                                                
    // 配置TX引脚                                               
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                  // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_10 ;                       // 引脚 TX-PB10
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                     // 初始化引脚工作模式
    // 配置RX引脚                                               
    GPIO_InitStruct.Pin   =  GPIO_PIN_11;                       // 引脚 RX-PB11
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                     // 初始化引脚工作模式                                                                
    // 中断优选级配置                                           
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);                    // 配置中断线的优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);                            // 使能中断线                                                                
    // 计算波特率参数                                           
    float    temp;                                              
    uint16_t mantissa, fraction;                                
    SystemCoreClockUpdate();                                    // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);    // 波特率公式计算; USART3挂载在APB1, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                            // 整数部分
    fraction = (temp - mantissa) * 16;                          // 小数部分
    USART3 -> BRR  = mantissa << 4 | fraction;                  // 设置波特率
    // 串口通信参数配置
    USART3 -> CR1  =   0;                                       // 清0
    USART3 -> CR1 |=   0x01 << 2;                               // 接收使能[02]: 0=失能、1=使能
    USART3 -> CR1 |=   0x01 << 3;                               // 发送使能[03]：0=失能、1=使能
    USART3 -> CR1 |=   0x00 << 9;                               // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART3 -> CR1 |=   0x00 << 10;                              // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART3 -> CR1 |=   0x00 << 12;                              // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART3 -> CR2  =   0;                                       // 数据清0
    USART3 -> CR2 &= ~(0x03 << 12);                             // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART3 -> CR3  =   0;                                       // 数据清0
    USART3 -> CR3 &= ~(0x01 << 6);                              // DMA接收[6]: 0=禁止、1=使能
    USART3 -> CR3 &= ~(0x01 << 7);                              // DMA发送[7]: 0=禁止、1=使能
    // 串口中断设置
    USART3 -> CR1 &= ~(0x01 << 7);                              // 关闭发送中断
    USART3 -> CR1 |=   0x01 << 5;                               // 使能接收中断: 接收缓冲区非空
    USART3 -> CR1 |=   0x01 << 4;                               // 使能空闲中断：超过1字节时间没收到新数据
    USART3 -> SR   = ~(0x00F0);                                 // 清理中断
    // 打开串口
    USART3 -> CR1 |=   0x01 << 13;                              // 使能UART开始工作
#endif

    // 清理中断
    USART3->SR = ~(0x00F0);                                     // 清理中断
    // 关联缓冲区
    xUART3.puRxData = uaUart3RxData;                            // 获取接收缓冲区的地址
    xUART3.puTxFiFoData = uaUart3TxFiFoData;                    // 获取发送缓冲区的地址
    // 输出提示
    printf("UART3 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： USART3_IRQHandler
 * 功  能： USART3的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/ 
/* 
 * 注意：此函数已被禁用！
 * 原因：本工程使用 CubeMX 生成的 HAL 库配置 USART3，中断处理函数在 stm32f1xx_it.c 中
 * 如需使用本文件的 USART3 功能，请取消下面的注释并注释掉 stm32f1xx_it.c 中的 USART3_IRQHandler
 */
/*
void USART3_IRQHandler(void)
{
    static uint16_t cnt = 0;                                        // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART3_RX_BUF_SIZE];                      // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((USART3->SR & 1 << 7) && (USART3->CR1 & 1 << 7))            // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART3->DR = xUART3.puTxFiFoData[xUART3.usTxFiFoTail++];    // 把要发送的字节，放入USART的发送寄存器
        if (xUART3.usTxFiFoTail == UART3_TX_BUF_SIZE)               // 如果数据指针到了尾部，就重新标记到0
            xUART3.usTxFiFoTail = 0;
        if (xUART3.usTxFiFoTail == xUART3.usTxFiFoData)
            USART3->CR1 &= ~(1 << 7);                               // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (USART3->SR & (1 << 5))                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART3_RX_BUF_SIZE))//||xUART3.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("警告：UART3单帧接收量，已超出接收缓存大小\r!");
            USART3->DR;                                             // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART3->DR ;                                // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (USART3->SR & (1 << 4))                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART3.usRxNum  = 0;                                        // 把接收到的数据字节数清0
        memcpy(xUART3.puRxData, rxTemp, UART3_RX_BUF_SIZE);         // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART3.usRxNum  = cnt;                                      // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                    // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART3_RX_BUF_SIZE);                       // 接收数据缓存数组，清零; 准备下一次的接收
        USART3 ->SR;
        USART3 ->DR;                                                // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}
*/

/******************************************************************************
 * 函  数： UART3_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData   需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART3_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                           // 把数据放入环形缓冲区
    {
        xUART3.puTxFiFoData[xUART3.usTxFiFoData++] = puData[i];    // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART3.usTxFiFoData == UART3_TX_BUF_SIZE)              // 如果指针位置到达缓冲区的最大值，则归0
            xUART3.usTxFiFoData = 0;
    }

    if ((USART3->CR1 & 1 << 7) == 0)                               // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART3->CR1 |= 1 << 7;                                     // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART3_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART3_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                              // 新建一个可变参数列表
    va_start(ap, pcString);                                  // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                   // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                              // 清空可变参数列表
    UART3_SendData((uint8_t *)mBuffer, strlen(mBuffer));     // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART3_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART3_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART3_ClearRx();                                              // 清0
    UART3_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART3_GetRxNum())                                     // 判断是否接收到数据
        {
            UART3_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART3_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART3_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART3_GetRxNum(void)
{
    return xUART3.usRxNum ;
}

/******************************************************************************
 * 函  数： UART3_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART3_GetRxData(void)
{
    return xUART3.puRxData ;
}

/******************************************************************************
 * 函  数： UART3_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART3_ClearRx(void)
{
    xUART3.usRxNum = 0 ;
}
#endif  // endif UART3_EN





//////////////////////////////////////////////////////////////   UART-4   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART4_EN

static xUSATR_TypeDef  xUART4 = { 0 };                      // 定义 UART4 的收发结构体
static uint8_t uaUart4RxData[UART4_RX_BUF_SIZE];            // 定义 UART4 的接收缓存
static uint8_t uaUart4TxFiFoData[UART4_TX_BUF_SIZE];        // 定义 UART4 的发送缓存

/******************************************************************************
 * 函  数： UART4_Init
 * 功  能： 初始化UART4的通信引脚、协议参数、中断优先级
 *          引脚：TX-PC10、RX-PC11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART4_Init(uint32_t ulBaudrate)
{
    // 使能相关时钟
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;                        // 使能外设：UART4
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                         // 使能GPIO：GPIOC
              
#ifdef USE_STDPERIPH_DRIVER                                                          // 标准库 配置
    // TX引脚 配置                                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                                      // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                                     // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                                 // 引脚功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                // 输出速度
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                           // 初始化引脚
    // RX引脚 配置                                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                                     // RX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;                                   // RX引脚工作模式：上拉输入; 如果使用浮空输入，引脚空置时可能产生误输入; 当电路上为一主多从电路时，可以使用复用开漏模式
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                           // 初始化引脚
    // 中断 配置                                                                     
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                                      // 优先级配置结构体
    NVIC_InitStructure .NVIC_IRQChannel = UART4_IRQn;                                // 中断编号
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 2 ;                      // 抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;                              // 子优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                                 // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                  // 初始化中断优先级
    //USART 设置                                                                     
    USART_DeInit(UART4);                                                             // 复位串口
    USART_InitTypeDef USART_InitStructure = {0};                                     // UART配置结构体 
    USART_InitStructure.USART_BaudRate   = ulBaudrate;                               // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 位长
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;                         // 停止位
    USART_InitStructure.USART_Parity     = USART_Parity_No;                          // 奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 使能工作模式
    USART_Init(UART4, &USART_InitStructure);                                         // 初始化串口
    // 打开中断
    USART_ITConfig(UART4, USART_IT_TXE, DISABLE);                                    // 发送中断 使能
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                                    // 接收中断 使能
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);                                    // 空闲中断 使能
    // 使能串口
    USART_Cmd(UART4, ENABLE);                                                        // 使能串口, 开始工作
#endif   
    
#ifdef USE_HAL_DRIVER       
    // 关闭串口                                                 
    UART4 -> CR1  =   0;                                        // 关闭串口，配置清零                                                                
    // 配置TX引脚                                               
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                  // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_10 ;                       // 引脚 TX-PC10
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                     // 初始化引脚工作模式
    // 配置RX引脚                                                      
    GPIO_InitStruct.Pin   =  GPIO_PIN_11;                       // 引脚 RX-PC11
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                     // 初始化引脚工作模式                                                                
    // 中断优选级配置                                           
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);                     // 配置中断线的优先级
    HAL_NVIC_EnableIRQ(UART4_IRQn);                             // 使能中断线                                                                
    // 计算波特率参数                                           
    float    temp;                                              
    uint16_t mantissa, fraction;                                
    SystemCoreClockUpdate();                                    // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);    // 波特率公式计算; UART4挂载在APB1, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                            // 整数部分
    fraction = (temp - mantissa) * 16;                          // 小数部分
    UART4 -> BRR  = mantissa << 4 | fraction;                   // 设置波特率
    // 串口通信参数配置
    UART4 -> CR1  =   0;                                        // 清0
    UART4 -> CR1 |=   0x01 << 2;                                // 接收使能[02]: 0=失能、1=使能
    UART4 -> CR1 |=   0x01 << 3;                                // 发送使能[03]：0=失能、1=使能
    UART4 -> CR1 |=   0x00 << 9;                                // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    UART4 -> CR1 |=   0x00 << 10;                               // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    UART4 -> CR1 |=   0x00 << 12;                               // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    UART4 -> CR2  =   0;                                        // 数据清0
    UART4 -> CR2 &= ~(0x03 << 12);                              // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    UART4 -> CR3  =   0;                                        // 数据清0
    UART4 -> CR3 &= ~(0x01 << 6);                               // DMA接收[6]: 0=禁止、1=使能
    UART4 -> CR3 &= ~(0x01 << 7);                               // DMA发送[7]: 0=禁止、1=使能
    // 串口中断设置                                            
    UART4 -> CR1 &= ~(0x01 << 7);                               // 关闭发送中断
    UART4 -> CR1 |=   0x01 << 5;                                // 使能接收中断: 接收缓冲区非空
    UART4 -> CR1 |=   0x01 << 4;                                // 使能空闲中断：超过1字节时间没收到新数据
    UART4 -> SR   = ~(0x00F0);                                  // 清理中断
    // 打开串口                                                
    UART4 -> CR1 |=   0x01 << 13;                               // 使能UART开始工作
#endif

    // 清理中断
    UART4->SR = ~(0x00F0);                                      // 清理中断
    // 关联缓冲区                                              
    xUART4.puRxData = uaUart4RxData;                            // 获取接收缓冲区的地址
    xUART4.puTxFiFoData = uaUart4TxFiFoData;                    // 获取发送缓冲区的地址
    // 输出提示
    printf("UART4 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： UART4_IRQHandler
 * 功  能： UART4的中断处理函数
 *          接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void UART4_IRQHandler(void)
{
    static uint16_t cnt = 0;                                        // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART4_RX_BUF_SIZE];                      // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((UART4->SR & 1 << 7) && (UART4->CR1 & 1 << 7))              // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        UART4->DR = xUART4.puTxFiFoData[xUART4.usTxFiFoTail++];     // 把要发送的字节，放入USART的发送寄存器
        if (xUART4.usTxFiFoTail == UART4_TX_BUF_SIZE)               // 如果数据指针到了尾部，就重新标记到0
            xUART4.usTxFiFoTail = 0;
        if (xUART4.usTxFiFoTail == xUART4.usTxFiFoData)
            UART4->CR1 &= ~(1 << 7);                                // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (UART4->SR & (1 << 5))                                       // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART4_RX_BUF_SIZE))//||xUART4.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("警告：UART4单帧接收量，已超出接收缓存大小\r!");
            UART4->DR;                                              // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = UART4->DR ;                                 // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (UART4->SR & (1 << 4))                                       // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART4.usRxNum  = 0;                                        // 把接收到的数据字节数清0
        memcpy(xUART4.puRxData, rxTemp, UART4_RX_BUF_SIZE);         // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART4.usRxNum  = cnt;                                      // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                    // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART4_RX_BUF_SIZE);                       // 接收数据缓存数组，清零; 准备下一次的接收
        UART4 ->SR;
        UART4 ->DR;                                                 // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： USART3_IRQHandler
 * 功  能： USART3的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/

/* 
 * 注意：此函数已被禁用！
 * 原因：本工程使用 CubeMX 生成的 HAL 库配置 USART3，中断处理函数在 stm32f1xx_it.c 中
 * 如需使用本文件的 USART3 功能，请取消下面的注释并注释掉 stm32f1xx_it.c 中的 USART3_IRQHandler
 */
/*
void USART3_IRQHandler(void)
{
    static uint16_t cnt = 0;                                        // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART3_RX_BUF_SIZE];                      // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((USART3->SR & 1 << 7) && (USART3->CR1 & 1 << 7))            // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART3->DR = xUART3.puTxFiFoData[xUART3.usTxFiFoTail++];    // 把要发送的字节，放入USART的发送寄存器
        if (xUART3.usTxFiFoTail == UART3_TX_BUF_SIZE)               // 如果数据指针到了尾部，就重新标记到0
            xUART3.usTxFiFoTail = 0;
        if (xUART3.usTxFiFoTail == xUART3.usTxFiFoData)
            USART3->CR1 &= ~(1 << 7);                               // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (USART3->SR & (1 << 5))                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART3_RX_BUF_SIZE))//||xUART3.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("警告：UART3单帧接收量，已超出接收缓存大小\r!");
            USART3->DR;                                             // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART3->DR ;                                // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (USART3->SR & (1 << 4))                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART3.usRxNum  = 0;                                        // 把接收到的数据字节数清0
        memcpy(xUART3.puRxData, rxTemp, UART3_RX_BUF_SIZE);         // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART3.usRxNum  = cnt;                                      // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                    // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART3_RX_BUF_SIZE);                       // 接收数据缓存数组，清零; 准备下一次的接收
        USART3 ->SR;
        USART3 ->DR;                                                // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}
*/

/******************************************************************************
 * 函  数： UART4_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData   需发送数据的地址
 *          uint8_t  usNum    发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART4_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART4.puTxFiFoData[xUART4.usTxFiFoData++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART4.usTxFiFoData == UART4_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART4.usTxFiFoData = 0;
    }

    if ((UART4->CR1 & 1 << 7) == 0)                              // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        UART4->CR1 |= 1 << 7;                                    // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART4_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART4_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                     // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                                   // 新建一个可变参数列表
    va_start(ap, pcString);                                       // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                        // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                                   // 清空可变参数列表
    UART4_SendData((uint8_t *)mBuffer, strlen(mBuffer));          // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART4_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART4_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART4_ClearRx();                                              // 清0
    UART4_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART4_GetRxNum())                                     // 判断是否接收到数据
        {
            UART4_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART4_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART4_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART4_GetRxNum(void)
{
    return xUART4.usRxNum ;
}

/******************************************************************************
 * 函  数： UART4_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART4_GetRxData(void)
{
    return xUART4.puRxData ;
}

/******************************************************************************
 * 函  数： UART4_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART4_ClearRx(void)
{
    xUART4.usRxNum = 0 ;
}
#endif  // endif UART4_EN





//////////////////////////////////////////////////////////////   UART-5   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART5_EN

static xUSATR_TypeDef  xUART5 = { 0 };                      // 定义 UART5 的收发结构体
static uint8_t uaUart5RxData[UART5_RX_BUF_SIZE];            // 定义 UART5 的接收缓存
static uint8_t uaUart5TxFiFoData[UART5_TX_BUF_SIZE];        // 定义 UART5 的发送缓存

/******************************************************************************
 * 函  数： UART5_Init
 * 功  能： 初始化UART5的通信引脚、协议参数、中断优先级
 *          引脚：TX-PC12、RX-PD2
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART5_Init(uint32_t ulBaudrate)
{
    // 使能相关时钟
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;                        // 使能外设：UART5
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                         // 使能GPIO：GPIOC
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;                         // 使能GPIO：GPIOD
           
#ifdef USE_STDPERIPH_DRIVER                                                          // 标准库 配置
    // TX引脚 配置                                                                   
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                                      // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                                     // TX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                                 // 引脚功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                // 输出速度
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                           // 初始化引脚
    // RX引脚 配置                                                                   
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                                      // RX 引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;                                   // RX引脚工作模式：上拉输入; 如果使用浮空输入，引脚空置时可能产生误输入; 当电路上为一主多从电路时，可以使用复用开漏模式
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                           // 初始化引脚
    // 中断 配置                                                                     
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                                      // 优先级配置结构体
    NVIC_InitStructure .NVIC_IRQChannel = UART5_IRQn;                                // 中断编号
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 2 ;                      // 抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;                              // 子优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                                 // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                  // 初始化中断优先级
    //USART 设置                                                                     
    USART_DeInit(UART5);                                                             // 复位串口
    USART_InitTypeDef USART_InitStructure = {0};                                     // UART配置结构体 
    USART_InitStructure.USART_BaudRate   = ulBaudrate;                               // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // 位长
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;                         // 停止位
    USART_InitStructure.USART_Parity     = USART_Parity_No;                          // 奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 使能工作模式
    USART_Init(UART5, &USART_InitStructure);                                         // 初始化串口
    // 打开中断
    USART_ITConfig(UART5, USART_IT_TXE, DISABLE);                                    // 发送中断 使能
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);                                    // 接收中断 使能
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);                                    // 空闲中断 使能
    // 使能串口                                                                 
    USART_Cmd(UART5, ENABLE);                                                        // 使能串口, 开始工作
#endif   

#ifdef USE_HAL_DRIVER       
    // 关闭串口                                                 
    UART5 -> CR1  =   0;                                        // 关闭串口，配置清零                                                                
    // 配置TX引脚                                               
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                  // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_12 ;                       // 引脚 TX-PC12
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                     // 初始化引脚工作模式
    // 配置RX引脚                                                        
    GPIO_InitStruct.Pin   =  GPIO_PIN_2;                        // 引脚 RX-PD2
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;                    // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                        // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;               // 引脚速率
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);                     // 初始化引脚工作模式                                                                
    // 中断优选级配置                                           
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);                     // 配置中断线的优先级
    HAL_NVIC_EnableIRQ(UART5_IRQn);                             // 使能中断线                                                                
    // 计算波特率参数                                           
    float    temp;                                              
    uint16_t mantissa, fraction;                                
    SystemCoreClockUpdate();                                    // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);    // 波特率公式计算; UART5挂载在APB1, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                            // 整数部分
    fraction = (temp - mantissa) * 16;                          // 小数部分
    UART5 -> BRR  = mantissa << 4 | fraction;                   // 设置波特率
    // 串口通信参数配置
    UART5 -> CR1  =   0;                                        // 清0
    UART5 -> CR1 |=   0x01 << 2;                                // 接收使能[02]: 0=失能、1=使能
    UART5 -> CR1 |=   0x01 << 3;                                // 发送使能[03]：0=失能、1=使能
    UART5 -> CR1 |=   0x00 << 9;                                // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    UART5 -> CR1 |=   0x00 << 10;                               // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    UART5 -> CR1 |=   0x00 << 12;                               // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    UART5 -> CR2  =   0;                                        // 数据清0
    UART5 -> CR2 &= ~(0x03 << 12);                              // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    UART5 -> CR3  =   0;                                        // 数据清0
    UART5 -> CR3 &= ~(0x01 << 6);                               // DMA接收[6]: 0=禁止、1=使能
    UART5 -> CR3 &= ~(0x01 << 7);                               // DMA发送[7]: 0=禁止、1=使能
    // 串口中断设置
    UART5 -> CR1 &= ~(0x01 << 7);                               // 关闭发送中断
    UART5 -> CR1 |=   0x01 << 5;                                // 使能接收中断: 接收缓冲区非空
    UART5 -> CR1 |=   0x01 << 4;                                // 使能空闲中断：超过1字节时间没收到新数据
    UART5 -> SR   = ~(0x00F0);                                  // 清理中断
    // 打开串口
    UART5 -> CR1 |=   0x01 << 13;                               // 使能UART开始工作
#endif

    // 清理中断
    UART5->SR = ~(0x00F0);                                      // 清理中断
    // 关联缓冲区
    xUART5.puRxData = uaUart5RxData;                            // 获取接收缓冲区的地址
    xUART5.puTxFiFoData = uaUart5TxFiFoData;                    // 获取发送缓冲区的地址
    // 输出提示
    printf("UART5 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： UART5_IRQHandler
 * 功  能： UART5的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void UART5_IRQHandler(void)
{
    static uint16_t cnt = 0;                                        // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART5_RX_BUF_SIZE];                      // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    // 发送中断：用于把环形缓冲的数据，逐字节发出
    if ((UART5->SR & 1 << 7) && (UART5->CR1 & 1 << 7))              // 检查TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        UART5->DR = xUART5.puTxFiFoData[xUART5.usTxFiFoTail++];     // 把要发送的字节，放入USART的发送寄存器
        if (xUART5.usTxFiFoTail == UART5_TX_BUF_SIZE)               // 如果数据指针到了尾部，就重新标记到0
            xUART5.usTxFiFoTail = 0;
        if (xUART5.usTxFiFoTail == xUART5.usTxFiFoData)
            UART5->CR1 &= ~(1 << 7);                                // 已发送完成，关闭发送缓冲区空置中断 TXEIE
        return;
    }

    // 接收中断：用于逐个字节接收，存放到临时缓存
    if (UART5->SR & (1 << 5))                                       // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if ((cnt >= UART5_RX_BUF_SIZE))//||xUART5.ReceivedFlag==1   // 判断1: 当前帧已接收到的数据量，已满(缓存区), 为避免溢出，本包后面接收到的数据直接舍弃．
        {
            // 判断2: 如果之前接收好的数据包还没处理，就放弃新数据，即，新数据帧不能覆盖旧数据帧，直至旧数据帧被处理．缺点：数据传输过快于处理速度时会掉包；好处：机制清晰，易于调试
            printf("警告：UART5单帧接收量，已超出接收缓存大小\r!");
            UART5->DR;                                              // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = UART5->DR ;                                 // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    // 空闲中断：用于判断一帧数据结束，整理数据到外部备读
    if (UART5->SR & (1 << 4))                                       // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        xUART5.usRxNum  = 0;                                        // 把接收到的数据字节数清0
        memcpy(xUART5.puRxData, rxTemp, UART5_RX_BUF_SIZE);         // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART5.usRxNum  = cnt;                                      // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                    // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART5_RX_BUF_SIZE);                       // 接收数据缓存数组，清零; 准备下一次的接收
        UART5 ->SR;
        UART5 ->DR;                                                 // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART5_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* pudata     需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART5_SendData(uint8_t *pudata, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART5.puTxFiFoData[xUART5.usTxFiFoData++] = pudata[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART5.usTxFiFoData == UART5_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART5.usTxFiFoData = 0;
    }

    if ((UART5->CR1 & 1 << 7) == 0)                              // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        UART5->CR1 |= 1 << 7;                                    // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART5_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART5_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                     // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                                   // 新建一个可变参数列表
    va_start(ap, pcString);                                       // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                        // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                                   // 清空可变参数列表
    UART5_SendData((uint8_t *)mBuffer, strlen(mBuffer));          // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART5_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART5_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART5_ClearRx();                                              // 清0
    UART5_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART5_GetRxNum())                                     // 判断是否接收到数据
        {
            UART5_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART5_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART5_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART5_GetRxNum(void)
{
    return xUART5.usRxNum ;
}

/******************************************************************************
 * 函  数： UART5_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART5_GetRxData(void)
{
    return xUART5.puRxData ;
}

/******************************************************************************
 * 函  数： UART5_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART5_ClearRx(void)
{
    xUART5.usRxNum = 0 ;
}
#endif  // endif UART5_EN






/////////////////////////////////////////////////////////  ModbusCRC16 校验   /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CRC16查表法值表数组
// 查表法相比直接计算CRC值，大约可减少一半运算时间
static const uint8_t aModbusCRC16[] =      
{
    // 高位
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    // 低位
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,  
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


/**********************************************************************************************************
 * 函 数:  Modbus_AddCRC16
 * 功 能:  在数据末尾追加两字节的ModbusCRC16校验值
 * 备 注： 1、函数将根据传入的原数据，计算CRC16值，在数据末尾追加两字节的CRC16校验值;
 *         2、注意传入的数组地址，要至少预留2字节的空间，用于追加CRC16的校验值;
 *         
 * 参 数:  uint8_t*  _pcData：参与计算校验值的数据
 *         uint16_t  _usLen ：数据字节数; 长度为数据的原字节数即可，不用加2;
 *
 * 返 回:  无
**********************************************************************************************************/
void  Modbus_AddCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                        // CRC值高位
    uint8_t  ucCRCLo = 0xFF;                        // CRC值低位
    uint16_t usIndex;                               // 索引
                                                    
    while (_usLen--)                                // 开始逐字节查表
    {
        usIndex = ucCRCHi ^ *_pcData++;             // 注意，这里指针地址加1了
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }
    _pcData[0] = ucCRCHi;                           // 末尾第1字节，追加CRC16的高位
    _pcData[1] = ucCRCLo;                           // 末尾第2字节，追加CRC16的低位
}



/**********************************************************************************************************
 * 函 数:  Modbus_CheckCRC16
 * 功 能:  对带ModbusCRC16校验的数据段进行校验
 *
 * 参 数:  uint8_t*  _pcData：数据段地址
 *         uint16_t  _usLen ：数据段的长度(字节数); 长度是整包数据的字节数，即包含ModbusCRC16值的长度，不用减2;
 *
 * 返 回:  0-错误、1-正确
*******************************************************************************************/
uint8_t  Modbus_CheckCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                               // CRC值高位
    uint8_t  ucCRCLo = 0xFF;                               // CRC值低位
    uint16_t usIndex;                                      // 数组的索引
   
    _usLen -= 2;                                           // 字节数-2，不计算原数据末尾两字节(ModbusCRC16值)
    
    while (_usLen--)                                       // 开始逐字节查表获得ModbusCRC16值
    {
        usIndex = ucCRCHi ^ *_pcData++;                    // 注意，这里指针地址递增加1
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }
    
    if ((ucCRCHi == *_pcData++) && (ucCRCLo == *_pcData))  // 把数据段的CRC16校验值，与原数据末尾的CRC16值相比较
        return 1;                                          // 成功匹配; 返回: 1
    
    return 0;                                              // 错误; 返回：0
}





/////////////////////////////////////////////////////////////  辅助功能   /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * 函  数： showData
 * 功  能： 把数据，经串口1，发送到串口助手上，方便观察
 * 参  数： uint8_t  *rxData   数据地址
 *          uint16_t  rxNum    字节数
 * 返回值： 无
 ******************************************************************************/
void showData(uint8_t *rxData, uint16_t rxNum)
{
#if 0                                                          // 写法1
    printf("字节数： %d \r", rxNum);                           // 显示字节数
    printf("ASCII 显示数据: %s\r", (char *)rxData);            // 显示数据，以ASCII方式显示，即以字符串的方式显示
    printf("16进制显示数据: ");                                // 显示数据，以16进制方式，显示每一个字节的值
    while (rxNum--)                                            // 逐个字节判断，只要不为'\0', 就继续
        printf("0x%X ", *rxData++);                            // 格式化
    printf("\r\r");                                            // 显示换行
#else                                                          // 写法2
    UART1_SendString("字节数：%d \r", rxNum);                  // 显示字节数
    UART1_SendString("ASCII : %s\r", (char *)rxData);          // 显示数据，以ASCII方式显示，即以字符串的方式显示
    UART1_SendString("16进制: ");                              // 显示数据，以16进制方式，显示每一个字节的值
    while (rxNum--)                                            // 逐个字节判断，只要不为'\0', 就继续
        UART1_SendString("0x%X ", *rxData++);                  // 格式化
    UART1_SendString("\r\r");                                  // 显示换行
#endif
}





//////////////////////////////////////////////////////////////  printf   //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * 功  能： printf函数支持代码
 *         【特别注意】加入以下代码, 使用printf函数时, 不再需要选择use MicroLIB
 * 参  数：
 * 返回值：
 * 备  注： 最后修改_2020年07月15日
 ******************************************************************************/
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};                                         // 标准库需要的支持函数

FILE __stdout;                             // FILE 在stdio.h文件
void _sys_exit(int x)
{
    x = x;                                 // 定义_sys_exit()以避免使用半主机模式
}

int fputc(int ch, FILE *f)                 // 重定向fputc函数，使printf的输出，由fputc输出到UART,  这里使用串口1(USART1)
{
#if 0                                      // 方式1-使用常用的方式发送数据，比较容易理解，但等待耗时大  
    while ((USART1->SR & 0X40) == 0);      // 等待上一次串口数据发送完成
    USART1->DR = (u8) ch;                  // 写DR,串口1将发送数据
#else
    UART1_SendData((uint8_t *)&ch, 1);     // 方式2-使用队列+中断方式发送数据; 无需像方式1那样等待耗时，但要借助已写好的函数、环形缓冲
#endif
    return ch;
}

// 文件结尾，需要保留至少1空行
