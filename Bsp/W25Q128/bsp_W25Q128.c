/***********************************************************************************************************************************
 ** 【文件名称】  bsp_w25qxx.c
 ** 【编写人员】  魔女开发板团队
 ** 【更新分享】  Q群文件夹
 ** 【淘    宝】  魔女开发板      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件功能】  初始化GPIO、SPI, 各功能函数
 ** 【适用平台】  STM32F103 + keil5 + HAL库
 **
 ** 【更新记录】  2023-01-27  完善注释、代码格式 
 **               2019-05-11  创建
 **               2019-12-03  大幅修改write功能, 使其更简单清晰；理解重点：明确擦扇和写页的区别, 写页指令最大缓存字节数.
 **               2020-08-15  完善代码注释，文件格式
 **               2021-02-24  多个函数增加判断，W25Qxx初始化失败时，跳过函数，防止卡死
 **
************************************************************************************************************************************/
#include "bsp_W25Q128.h"





/*****************************************************************************
 ** 变量声明
 *****************************************************************************/
typedef struct
{
    uint8_t   InitStatus;                 // 初始化状态   0:失败, 1:成功
    uint8_t   FlagGBKStorage;           // GBK字库标志; 0=没有, 1=可用; 作用: 用于判断地址段的写保护, 防止字库被错误写履盖; 并可作LCD的中文输出判断
    char      type[20];                 // 型号
    uint16_t  StartupTimes;             // 记录启动次数
} xW25Q_TypeDef;
xW25Q_TypeDef   xW25Q128;               // 设备状态

//W25Q系列芯片型号返回值
#define    W25Q80            0XEF13
#define    W25Q16            0XEF14
#define    W25Q32            0XEF15
#define    W25Q64            0XEF16
#define    W25Q128           0XEF17
#define    W25Q256           0XEF18
//#define  W25Qxx             65519     // 很多时候重新下载后读出的都是65519

#define    CS_HIGH    (W25Q128_CS_GPIO -> BSRR =  W25Q128_CS_PIN)
#define    CS_LOW     (W25Q128_CS_GPIO -> BSRR =  W25Q128_CS_PIN<<16)

/*****************************************************************************
 ** 内部函数声明
****************************************************************************/
// 5个基本功能
static uint8_t  sendByte(uint8_t d);                                  // 5_1    字节读写
static void     writeEnable(void) ;                                   // 5_2    写使能
static void     WaitReady(void) ;                                     // 5_3    等待空闲
static void     eraseSector(uint32_t addr);                           // 5_4    擦扇区
static void     writeSector(uint32_t addr, uint8_t *p, uint16_t num); // 5_5    写扇区
// 测试
static void     readID(void);
static void     spiInit(void);
static void     checkFlagGBKStorage(void);                            // 检查字库数据正确性



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
    ulTimesUS = us * 20;
    while (ulTimesUS)
        ulTimesUS--;                   // 操作外部变量，防止空循环被编译器优化掉
}




/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返回值： 无
 ******************************************************************************/
#if 0
static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 4785;
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}
#endif




/******************************************************************************
 * 函  数： W25qx_Init
 * 功  能： 初始化W25Q128所需引脚、SPI
 * 参  数：
 * 返回值： 初始化结果，0:失败、1:成功
 ******************************************************************************/
uint8_t W25Q128_Init()
{
    //delay_ms(20);                                                        // 稍作延时，令设备上电稳定后再初始化
    HAL_Delay(100);
    
    // 时钟使能;用判断端口的方式使能时钟线, 减少移植时的工作
    // 使能SPI时钟
    if (W25Q128_SPI == SPI1)        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;    // 使能 SPI1
    if (W25Q128_SPI == SPI2)        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;    // 使能 SPI2
    if (W25Q128_SPI == SPI3)        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;    // 使能 SPI3
    // 使能NSS引脚端口时钟
    if (W25Q128_CS_GPIO == GPIOA)  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;     // 使能GPIO：GPIOA
    if (W25Q128_CS_GPIO == GPIOB)  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;     // 使能GPIO：GPIOB
    if (W25Q128_CS_GPIO == GPIOC)  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;     // 使能GPIO：GPIOC
    if (W25Q128_CS_GPIO == GPIOD)  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;     // 使能GPIO：GPIOD
    if (W25Q128_CS_GPIO == GPIOE)  RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;     // 使能GPIO：GPIOE
    if (W25Q128_CS_GPIO == GPIOF)  RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;     // 使能GPIO：GPIOF
    if (W25Q128_CS_GPIO == GPIOG)  RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;     // 使能GPIO：GPIOG
    // 使能SPIx引脚端口时钟                                               
    if (W25Q128_CLK_GPIO == GPIOA)  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;    // 使能GPIO：GPIOA
    if (W25Q128_CLK_GPIO == GPIOB)  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;    // 使能GPIO：GPIOB
    if (W25Q128_CLK_GPIO == GPIOC)  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;    // 使能GPIO：GPIOC
    if (W25Q128_CLK_GPIO == GPIOD)  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;    // 使能GPIO：GPIOD
    if (W25Q128_CLK_GPIO == GPIOE)  RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;    // 使能GPIO：GPIOE
    if (W25Q128_CLK_GPIO == GPIOF)  RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;    // 使能GPIO：GPIOF
    if (W25Q128_CLK_GPIO == GPIOG)  RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;    // 使能GPIO：GPIOG

    // CS 引脚工作模式配置
    GPIO_InitTypeDef    GPIO_InitStruct = {0};             // 声明初始化要用到的结构体
    
    GPIO_InitStruct.Pin   = W25Q128_CS_PIN;                // CS 片选引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;           // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                   // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;          // 引脚速率
    HAL_GPIO_Init(W25Q128_CS_GPIO, &GPIO_InitStruct);      // 初始化
    
    CS_HIGH;                                               // CS 引脚拉高

    GPIO_InitStruct.Pin   = W25Q128_CLK_PIN;               // CLK 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;               // 工作模式
    HAL_GPIO_Init(W25Q128_CLK_GPIO, &GPIO_InitStruct);     // 初始化
    
    GPIO_InitStruct.Pin   =  W25Q128_MISO_PIN ;            // MISO 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;               // 工作模式
    HAL_GPIO_Init(W25Q128_MISO_GPIO, &GPIO_InitStruct);    // 初始化
    
    GPIO_InitStruct.Pin   =  W25Q128_MOSI_PIN;             // MOSI 引脚
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;               // 工作模式
    HAL_GPIO_Init(W25Q128_MOSI_GPIO, &GPIO_InitStruct);    // 初始化

    spiInit();                                             // spi初始化，独立封装，便于不同设备使用同一spi
                                                          
    readID();                                              // 读取芯片型号,判断通讯是否正常
    checkFlagGBKStorage();                                 // 检查字库
                                                          
    if(xW25Q128.InitStatus)                                 
        return 1;                                          // 初始化成功，返回:1
                                                
    return 0;                                              // 初始化失败，返回:0
}


/*****************************************************************************
  * spi初始化
  * 只要是w25qxx系列，都适用
  * 注意，设备的SPI初始化，必须做成一个单独的函数。因为STM32在设备间切换时，SPI的参数也得重新调换
  */
static void  spiInit(void)
{
    W25Q128_SPI -> CR1  = 0x1 << 0;       // CPHA:时钟相位,0x1=在第2个时钟边沿进行数据采样
    W25Q128_SPI -> CR1 |= 0x1 << 1;       // CPOL:时钟极性,0x1=空闲状态时，SCK保持高电平
    W25Q128_SPI -> CR1 |= 0x1 << 2;       // 主从模式:         1 = 主配置
    W25Q128_SPI -> CR1 |= 0x0 << 3;       // 波特率控制[5:3]:  0 = fPCLK /2
    W25Q128_SPI -> CR1 |= 0x0 << 7;       // 帧格式:           0 = 先发送MSB
    W25Q128_SPI -> CR1 |= 0x1 << 9;       // 软件从器件管理 :  1 = 使能软件从器件管理(软件NSS)
    W25Q128_SPI -> CR1 |= 0x1 << 8;       // 内部从器件选择,根据9位设置(失能内部NSS)
    W25Q128_SPI -> CR1 |= 0x0 << 11;      // 数据帧格式,       0 = 8位
    W25Q128_SPI -> CR1 |= 0x1 << 6;       // SPI使能           1 = 使能外设

    delay_us(20);                         // 稍作延时
}

// 5_1 发送1字节,返回1字节
// SPI通信,只一个动作:向DR写入从设命令值,同步读出数据!写读组合,按从设时序图来. 作为主设,因为收发同步,连接收发送中断也不用开,未验证其它中断对其工作的影响.
uint8_t  sendByte(uint8_t d)
{
    uint8_t retry = 0;

    while ((W25Q128_SPI ->SR & 2) == 0)   // 等待发送区为空
    {
        retry++;
        if (retry > 250)    return 0;
    }
    W25Q128_SPI ->DR = d;

    retry = 0;
    while ((W25Q128_SPI->SR & 1) == 0)    // 等待接收完数据
    {
        retry++;
        if (retry > 250)    return 0;
    }
    return W25Q128_SPI->DR ;
}

// 5_2 写使能
void writeEnable()
{
    CS_LOW ;
    sendByte(0x6);                        // 命令: Write Enable : 06h
    CS_HIGH ;
}

// 5_3 等待空闲
void WaitReady()
{
    CS_LOW ;

    sendByte(0x05);                       // 命令: Read Status Register : 05h
    while (sendByte(0xFF) & 1) {}         // 只要发送读状态寄存器指令，芯片就会持续向主机发送最新的状态寄存器内容 ，直到收到通信的停止信号。

    CS_HIGH ;
}

// 5_4 擦除一个扇区, 每扇区>150ms
void eraseSector(uint32_t addr)
{
    if (xW25Q128 .InitStatus == 0) return;  // 如果W25Qxx初始化失败，则跳过检测，防止卡死

    addr = addr * 4096;                   // 从第几扇区开始

    writeEnable();
    WaitReady();
    // 命令
    CS_LOW ;
    sendByte(0x20);                       // 命令: Sector Erase(4K) : 20h
    sendByte((uint8_t)(addr >> 16));
    sendByte((uint8_t)(addr >> 8));
    sendByte((uint8_t)addr);
    CS_HIGH ;

    WaitReady();
}

// 5_5 写扇区. 要分页写入
void writeSector(uint32_t addr, uint8_t *p, uint16_t num)
{
    if (xW25Q128 .InitStatus == 0) return;           // 如果W25Qxx初始化失败，则跳过检测，防止卡死
                                                   
    uint16_t pageRemain = 256;                     // 重要，重要，重要：W25Qxx每个页命令最大写入字节数:256字节;
    
    for (char i = 0; i < 16; i++)                  // 扇区:4096bytes, 缓存页:256bytes, 写扇区要分16次页命令写入
    {
        writeEnable();                             // 写使能
        WaitReady();                               // 等待空闲
                                                   
        CS_LOW ;                                   // 低电平,开始
        sendByte(0x02);                            // 命令: page program : 02h , 每个写页命令最大缓存256字节
        sendByte((uint8_t)(addr >> 16));           // 地址
        sendByte((uint8_t)(addr >> 8));            
        sendByte((uint8_t)addr);                   
        for (uint16_t i = 0; i < pageRemain; i++)  // 发送写入的数据
            sendByte(p[i]);                        // 高电平, 结束
        CS_HIGH ;                                  
                                                   
        WaitReady();                               // 等待空闲
                                                   
        p = p + pageRemain;                        // 缓存指针增加一页字节数
        addr = addr + pageRemain ;                 // 写地址增加一页字节数
    }
}

// 读取芯片型号,判断通讯是否正常
static void readID()
{
    uint16_t W25QxxType = 0 ;
    // 1: 读取芯片型号, 判断联接状况
    CS_LOW;
    sendByte(0x90);                       // 发送读取ID命令,命令分两分,第一字节是命令,第四字节是0
    sendByte(0x00);                       
    sendByte(0x00);                       
    sendByte(0x00);                       // 第四字节必节须是 0h
    W25QxxType  = (sendByte(0xFF)) << 8;  // uint16_t W25QxxType
    W25QxxType |= sendByte(0xFF);
    CS_HIGH;

    xW25Q128.InitStatus  = 1;
    switch (W25QxxType)
    {
        case W25Q16:
            sprintf((char *)xW25Q128.type, "%s", "W25Q16");
            break;
        case W25Q32:
            sprintf((char *)xW25Q128.type, "%s", "W25Q32");
            break;
        case W25Q64:
            sprintf((char *)xW25Q128.type, "%s", "W25Q64");
            break;
        case W25Q128:
            sprintf((char *)xW25Q128.type, "%s", "W25Q128");
            break;
        case W25Q256:
            sprintf((char *)xW25Q128.type, "%s", "W25Q256");          // 注意:W25Q256的地址是4字节
            break;
        default:
            sprintf((char *)xW25Q128.type, "%s", "Flash设备失败 !!!");
            xW25Q128.InitStatus = 0;
            printf("读取到的错误型号数据：%d\r\n", W25QxxType);
            break;
    }

    // 2:读取存储数据, 增加启动次数记录
    if (xW25Q128.InitStatus  == 1)
    {
        uint32_t Addr = 0x00;                 // 数据地址,  W25Q128最大地址:0X0100 0000
        uint8_t d[4] = {0};                   // 数据缓存， 0x0000:标志0xEE, 0x0001:标志0X00,     0x0002:数据高位, 0x0003:数据低位
        uint16_t startFlag  = 0;              // 标志
        uint16_t startNum   = 0;              // 启动次数
                                              
        W25Q128_ReadData(Addr, d, 4);         // 读取4个字节数据
        startFlag = (d[0] << 8) | d[1];       // 标志
        startNum  = (d[2] << 8) | d[3];       // 启动次数
                                              
        if (startFlag != 0xEE00)              // 没有旧记录
        {
            startNum = 1;
            d[2] = 0;
            d[3] = 1;
        }
        else
        {
            startNum++;                       // 成功读取数据， 次数增加1
            d[2] = (uint8_t)(startNum >> 8);
            d[3] = (uint8_t)startNum;
            if (STARTUPTIMES_RESET == 1)
            {
                d[2] = (uint8_t)0;            // 回复启动次数 = 0, 取消这两行即可，
                d[3] = (uint8_t)0;            // 编译烧录后，得重新注释，编译再次烧录
            }
        }
        d[0] = 0xEE;
        d[1] = 0x00;
        
        W25Q128_WriteData(Addr, d, 4);
        xW25Q128.StartupTimes = startNum;

        printf("Flash Storage Test... Type:%s , Boot Count:%d\r", xW25Q128.type, startNum);
    }
    else // 检测W25Qxx失败
    {        
        printf("Storage Check:        Type Read Error, Device Unavailable!\r");
        printf("Trying to reset chip......\r");
        //System_Reset ();
    }
}



/******************************************************************************
 * 函  数： W25Q128_ReadData
 * 功  能： 读取数据
 * 参  数： uint32_t addr  数据在W25Q128内的地址
 *          uint8_t *pData 数据缓存地址
 *          uint16_t num   连续读取的字节数
 * 返回值： 无
 ******************************************************************************/
void W25Q128_ReadData(uint32_t addr, uint8_t *pData, uint16_t num)
{
    if (xW25Q128 .InitStatus == 0) return; // 如果W25Qxx初始化失败，则跳过检测，防止卡死

    spiInit();                           // 每次读写前，先重新配置SPI，避免多个设备共用一SPI时的配置不同

    CS_LOW ;
    sendByte(0x03);                      // 发送读取命令 03h
    sendByte((uint8_t)(addr >> 16));
    sendByte((uint8_t)(addr >> 8));
    sendByte((uint8_t)addr);

    for (uint32_t i = 0; i < num; i++)
    {
        pData[i] = sendByte(0xFF);
    }

    CS_HIGH ;
}



/******************************************************************************
 * 函数名： W25Q128_WriteData
 * 功  能： 从addr处起，读取num个字节，存放到缓存p
 * 参  数： uint32_t  addr   写入地址         (W25Q128 只用3字节, W25Q256用4字节)
 *          uint8_t  *pData  要写入的数据存储区
 *          uint16_t  num    写入的字节数
 * 返  回： 无
 * 备  注： 最后更新_2020年12月15日
 ******************************************************************************/
uint8_t W25Q128_buffer[4096];                    // 开辟一段内存空间

void W25Q128_WriteData(uint32_t addr, uint8_t *pData, uint16_t num)
{
    if (xW25Q128.InitStatus == 0) return ;         // 如果w25qxx设备初始化失败，则跳过本函数，防止卡死

    // 字库段写保护, 防止字库被错误写入履盖
    if (((addr + num) > 0x00A00000) && (xW25Q128.FlagGBKStorage == 1))
    {
        printf("Warning: Data in font area, write operation skipped!!\r");
        return;
    }

    uint32_t  secPos      = addr / 4096;                 // 扇区地址,第几个扇区
    uint16_t  secOff      = addr % 4096;                 // 开始地始偏移字节数: 数据在扇区的第几字节存放
    uint16_t  secRemain   = 4096 - secOff;               // 扇区剩余空间字节数 ,用于判断够不够存放余下的数据
    uint8_t  *buf = W25Q128_buffer;                      // 原子哥代码,为什么不直接使用所声明的数组. (回看前面的疑问, 接触C有15年, 原来没下过工夫)
                                                         
    spiInit();                                           // 每次读写前，先重新配置SPI，避免多个设备共用一SPI时的配置不同
    if (num <= secRemain) secRemain = num;               
    while (1)                                            
    {                                                    
        W25Q128_ReadData(secPos * 4096, buf, 4096);      // 读取扇区内容到缓存
                                                         
        eraseSector(secPos);                             // 擦扇区
        for (uint16_t i = 0; i < secRemain ; i++)        // 原始数据写入缓存
            buf[secOff + i] = pData[i];                  
        writeSector(secPos * 4096, buf, 4096);           // 缓存数据写入设备
                                                         
        if (secRemain == num)                            // 已全部写入
            break;                                       
        else                                             
        {                                                
            // 未写完                                    
            pData = pData + secRemain ;                  // 原始数据指针偏移
            secPos ++;                                   // 新扇区
            secOff = 0;                                  // 新偏移位,扇区内数据起始地址
            num = num - secRemain ;                      // 剩余未写字节数
            secRemain = (num > 4096) ? 4096 : num;       // 计算新扇区写入字节数
        }
    }
}



// 检查字库样本的正确性
void checkFlagGBKStorage(void)
{
    if (xW25Q128 .InitStatus == 0) return;                       // 如果W25Qxx初始化失败，则跳过检测，防止卡死

    printf("GBK Font Test...      ");
    uint8_t sub = 0;
    uint8_t f = 0 ;

    for (uint32_t i = 0; i < 6128640; i = i + 1000000)
    {
        W25Q128_ReadData(GBK_STORAGE_ADDR + i, &f, 1);
        sub = sub + f;                                         // 80 , 0, 98, 79, 0, 1, 0
    }
    xW25Q128.FlagGBKStorage = (sub == 146 ? 1 : 0);            // 判断是否有字库,打开地址写保护, 防止字库被错误写入履盖

    if (xW25Q128.FlagGBKStorage == 1)   
        printf("Font Available\r");                                  // 标记字库可用
    else       
        printf(" Error, Font Unavailable!\r");
}


/******************************************************************************
 * 函数名： W25Q128_GetInitStatus
 * 功  能： 获取W25Q128芯片的初始化状态
 * 参  数： 无
 * 返  回： 0-异常、1-初始化成功
 ******************************************************************************/
uint8_t W25Q128_GetInitStatus(void)
{
    return xW25Q128.InitStatus;    
}



/******************************************************************************
 * 函数名： W25Q128_GetInitStatus
 * 功  能： 获取W25Q128芯片的型号
 * 参  数： 无
 * 返  回： 芯片型号的字符串地址
 ******************************************************************************/
char* W25Q128_GetType(void)
{
    return xW25Q128.type;    
}



/******************************************************************************
 * 函数名： W25Q128_GetFontStorageStatus
 * 功  能： 获取W25Q128芯片上GBK字库的存储状态
 * 参  数： 无
 * 返  回： 0-无字库、1-字库正常
 ******************************************************************************/
uint8_t W25Q128_GetFontStorageStatus(void)
{
    return xW25Q128.FlagGBKStorage;    
}



/******************************************************************************
 * 函数名： W25Q128_ReadFontData
 * 功  能： 从w25Q128的字库中读取出汉字字模数据
 * 参  数： uint8_t *pFont     汉字
 *          uint8_t  size      字体大小 12/16/24/32
 *          uint8_t *fontData  读取到的字模点阵数据
 * 返  回： 无
 * 备  注： 最后更新_2024年02月05日
 ******************************************************************************/
void W25Q128_ReadFontData(uint8_t *pFont, uint8_t size, uint8_t *fontData)
{
    uint8_t qh, ql;
    uint32_t foffset;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size); // 计算汉字点阵大小，单位字节数

    qh = *pFont;
    ql = *(++pFont);

    if (qh < 0x81 || ql < 0x40 || ql == 0xff || qh == 0xff)     // 非常用汉字，将用填充显示整个位置
    {
        for (uint8_t i = 0; i < csize; i++)
            *fontData++ = 0x00;                                 // 填充满格
        return;                                                 // 结束访问
    }                                                          
                                                               
    if (ql < 0x7f) ql -= 0x40;                                  // 计算要提取的汉字在字库中的偏移位置
    else        ql -= 0x41;
    qh -= 0x81;
    foffset = ((unsigned long)190 * qh + ql) * csize;           // 得到汉字在字库中的偏移位置

    switch (size)                                               // 按字体的不同，在不同字库读取字体点阵
    {        
        case 12:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x00000000, fontData, csize);  // 12号字体
            break;
        case 16:
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x0008c460, fontData, csize);  // 16号字体
            break;                                                                         
        case 24:                                                                           
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x001474E0, fontData, csize);  // 24号字体
            break;                                                                         
        case 32:                                                                           
            W25Q128_ReadData(foffset + GBK_STORAGE_ADDR + 0x002EC200, fontData, csize);  // 32号字体
            break;
    }
}


