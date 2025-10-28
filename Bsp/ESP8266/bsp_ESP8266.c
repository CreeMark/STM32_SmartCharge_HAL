/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【最后版本】  2024-07-08-02
 ** 【淘    宝】  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件名称】  bsp_ESP8266.c
 **
 ** 【文件功能】  对ESP8266常用AT指令组合 、功能，进行函数封装
 **
 ** 【适用平台】  STM32F407 + keil5 + HAL库/标准库
 **         
************************************************************************************************************************************/
#include "bsp_ESP8266.h"
#include "stdlib.h"           // C语言的标准库函数，包含atoi等函数 



/*****************************************************************************
 ** 全局变量
****************************************************************************/
#define ESP8266_RX_BUF_SIZE       1024              // 数据接收缓冲区大小，大部份情况下都不用修改

typedef struct
{
    uint8_t         Flag;                           // 状态标记; 0=未初始化或异常, 1=正常
    uint16_t        RxNum;                          // 接收到多少个字节数据; 0-无数据，非0_接收到的字节数
    uint8_t         RxData[ESP8266_RX_BUF_SIZE];    // 接收到数据的缓存; ESP8266在AT模式下，每帧数据最长为1056个字节;
    char           *APName;                         // 当创建或加入AP时的: SSID
    char           *APPassword;                     // 当创建或加入AP时的: 密码
    uint32_t        Baudrate;                       // 记录所用的串口波特率
    USART_TypeDef  *UARTx;                          // 记录所用的端口
} xESP8266_TypeDef;

xESP8266_TypeDef   xESP8266 = {0} ;                 // 定义结构体




/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返  回： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}



/******************************************************************************
 * 函  数： ESP8266_Init
 * 功  能： ESP8266初始化：通信引脚、UART、协议参数、中断优先级
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 * 参  数： USART_TypeDef  *USARTx    串口端口：USART1、USART2、USART3、UART4、UART5、USART6
 *          uint32_t        baudrate  通信波特率
 * 返  回： 无
 ******************************************************************************/
void ESP8266_Init(USART_TypeDef *USARTx, uint32_t baudrate)
{
    printf("\r\nESP8266 开始配置\r\n");
    delay_ms(200);                                                 // 重要，上电后，必须稍延时以等待8266稳定方可工作

    if (USARTx == USART1)    UART1_Init(baudrate);
    if (USARTx == USART2)    UART2_Init(baudrate);
    if (USARTx == USART3)    UART3_Init(baudrate);
    if (USARTx == UART4)     UART4_Init(baudrate);
    if (USARTx == UART5)     UART5_Init(baudrate);

    xESP8266.Flag = 0;                                             // 初始化状态
    xESP8266.UARTx   = USARTx;                                     // 记录所用串口端口
    xESP8266.Baudrate = baudrate;                                  // 记录所用的波特率
}



/******************************************************************************
 * 函  数： ESP8266_SendString
 * 功  能： 发送字符串;
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返  回： 无
 ******************************************************************************/
void ESP8266_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                                                // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                                                              // 新建一个可变参数列表
    va_start(ap, pcString);                                                                  // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                                                   // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                                                              // 清空可变参数列表

    if (xESP8266.UARTx == USART1)    UART1_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
    if (xESP8266.UARTx == USART2)    UART2_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == USART3)    UART3_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == UART4)     UART4_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == UART5)     UART5_SendData((uint8_t *)mBuffer, strlen(mBuffer));
}



/******************************************************************************
 * 函  数： ESP8266_SendAT
 * 功  能： 向ESP8266模块发送AT命令, 并等待返回信息
 * 参  数： char     *pcAT    : AT指令字符串
 *          char     *pcACK    : 期待的指令返回信息字符串
 *          uint16_t  usTimeOut: 发送命令后等待的时间，毫秒       
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_SendAT(char *pcAT, char *pcACK, uint16_t usTimeOut)
{
    ESP8266_ClearRx();                                     // 清0 接收的字节数、缓存
    ESP8266_SendString(pcAT);                              // 发送AT指令

    while (usTimeOut--)                                    // 等待指令返回执行情况
    {
        if (ESP8266_GetRxNum())                            // 判断接收
        {
            ESP8266_ClearRx();                             // 清空接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)xESP8266.RxData, pcACK))    // 收到命令确认
                return 1;                                  // 返回：1，指令操作成功
        }
        delay_ms(1);                                       // 延时; 用于超时退出处理，避免死等
    }
    ESP8266_ClearRx();                                     // 清0 接收的字节数、缓存
    return 0;                                              // 返回：0，指令操作失败
}



/******************************************************************************
 * 函  数： ESP8266_GetRxNum
 * 功  能： 获取最新一帧的字节数，可用于判断检查是否收到新的数据
 * 参  数： 无
 * 返  回： 返回接收到的字节数
 ******************************************************************************/
uint16_t ESP8266_GetRxNum(void)
{
    // UART1
    if ((xESP8266.UARTx == USART1) && (UART1_GetRxNum()))
    {
        xESP8266.RxNum = UART1_GetRxNum();
        memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);
        memcpy(xESP8266.RxData, UART1_GetRxData(), xESP8266.RxNum);
        UART1_ClearRx();
    }
    // UART2
    if ((xESP8266.UARTx == USART2) && (UART2_GetRxNum()))
    {
        xESP8266.RxNum = UART2_GetRxNum();
        memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);
        memcpy(xESP8266.RxData, UART2_GetRxData(), xESP8266.RxNum);
        UART2_ClearRx();
    }
    // UART3
    if ((xESP8266.UARTx == USART3) && (UART3_GetRxNum()))
    {
        xESP8266.RxNum = UART3_GetRxNum();
        memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);
        memcpy(xESP8266.RxData, UART3_GetRxData(), xESP8266.RxNum);
        UART3_ClearRx();
    }
    // UART4
    if ((xESP8266.UARTx == UART4)  && (UART4_GetRxNum()))
    {
        xESP8266.RxNum = UART4_GetRxNum();
        memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);
        memcpy(xESP8266.RxData, UART4_GetRxData(), xESP8266.RxNum);
        UART4_ClearRx();
    }
    // UART5
    if ((xESP8266.UARTx == UART5)  && (UART5_GetRxNum()))
    {
        xESP8266.RxNum = UART5_GetRxNum();
        memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);
        memcpy(xESP8266.RxData, UART5_GetRxData(), xESP8266.RxNum);
        UART5_ClearRx();
    }

    return xESP8266.RxNum;
}



/******************************************************************************
 * 函  数： ESP8266_GetRxData
 * 功  能： 获取最新一帧的数据 (地址）
 * 参  数： 无
 * 返  回： 缓存地址 (uint8_t*)
 ******************************************************************************/
uint8_t *ESP8266_GetRxData(void)
{
    return xESP8266.RxData ;
}



/******************************************************************************
 * 函  数： ESP8266_ClearRx
 * 功  能： 清0接收到的最后一帧的接收字节数;
 * 参  数： 无
 * 返  回： 无
 ******************************************************************************/
void ESP8266_ClearRx(void)
{

    xESP8266.RxNum = 0;                                    // 清0，接收字节数
    //memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);     // 清0，接收缓存
}




//////////////////////////////////////////////////////////////  常用AT指令封装   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * 函    数： ESP8266_AT
 * 功    能： 发送AT命令：AT
 *            可用于测试线路连接是否正常，和测试代码收发机制的正确性
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT(void)
{
    xESP8266.Flag = ESP8266_SendAT("AT\r\n",   "OK", 1500) ;       // 测试AT指令，以判断是否能连接ESP8266

    if (xESP8266.Flag)
    {
        printf("模块连接测试:  成功\r\n");                         // AT指令测试成功
    }
    else
    {
        printf("模块连接测试:  失败！    ");                       // AT指令测试失败
        printf("检查：ESP8266与USART的电路连接、跳线帽！！\r\n");  // 输出错误建议
    }

    return xESP8266.Flag;                                          // 返回, 初始化状态，0_失败，1_正常
}



/******************************************************************************
 * 函    数： ESP8266_AT_RESTORE
 * 功    能： 发送AT指令：AT+RESTORE
 *            恢复模块的出厂设置
 *            可用于清除芯片中已保存的wifi名称、密码
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_RESTORE(void)
{
    uint8_t status = 0 ;
    status = ESP8266_SendAT("AT+RESTORE\r\n", "ready", 3000);                         // 恢复模块的出厂设置
    status ? printf("恢复出厂设置:  成功\r\n") : printf("恢复出厂设置:  失败\r\n");   // 输出提示信息
    return status;                                                                    // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_RST
 * 功    能： 发送AT指令: AT+RST
 *            复位、重启
 * 参    数： 无
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_RST(void)
{
    uint8_t status = 0 ;
    status = ESP8266_SendAT("AT+RST\r\n", "ready", 3000);                            // 恢复模块的出厂设置
    status ? printf("重启 ESP8266:  成功\r\n") : printf("重启 ESP8266:  失败\r\n");  // 输出提示信息
    return status;                                                                   // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_CWMODE
 * 功    能： 设置ESP8266的工作模式
 * 参    数： 1-STA模式
 *            2-AP模式
 *            3-STA+AP
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_CWMODE(uint8_t mode)
{
    uint8_t status = 0 ;
    char str[100] = {0};

    if (mode > 3)
    {
        printf("配置工作模式:  失败！参数范围：1、2、3; 参数解释：1-STA, 2-AP, 3-STA+AP\r\n");
    }

    sprintf(str, "AT+CWMODE=%d\r\n", mode);                                                // 工作模式：1-STA, 2-AP, 3-STA+AP
    status = ESP8266_SendAT(str,    "OK", 3000)  ;                                         // 发送给8266，并判断指令是否执行成功
    if (mode == 1)
        status ?  printf("配置 STA模式:  成功\r\n") : printf("配置 STA模式:  失败\r\n");   // 输出提示信息
    if (mode == 2)
        status ?  printf("配置 AP 模式:  成功\r\n") : printf("配置 AP 模式:  失败\r\n");   // 输出提示信息
    if (mode == 3)
        status ?  printf("配置 STA+AP :  成功\r\n") : printf("配置 STA+AP :  失败\r\n");   // 输出提示信息
    return status;                                                                         // 返回：0-失败、1成功
}



/******************************************************************************
 * 函    数： ESP8266_AT_CIPMUX
 * 功    能： 设置 多连接
 * 备    注： 透传必须AT+CIPMUX=0;
 * 参    数： uint8_t value   0_单连接，1_多连接
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t  ESP8266_AT_CIPMUX(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMUX=%d\r\n", value);
    ESP8266_SendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("设置连接方式:  成功\r\n") : printf("设置连接方式: 失败\r\n");   // 多 连 接: 0_单连接，1_多连接
    return flag;
}



/******************************************************************************
 * 函    数： ESP8266_AT_CIPMODE
 * 功    能： 设置 传输方式
 * 参    数： uint8_t value   0_普通传输，1_透明传输
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t   ESP8266_AT_CIPMODE(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMODE=%d\r\n", value);
    ESP8266_SendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("设置传输方式:  成功\r\n") : printf("设置传输方式: 失败\r\n");   // 传输方式，0_普通传输，1_透明传输
    return flag;
}



/******************************************************************************
 * 函    数： ESP8266_AT_SetPassThrough
 * 功    能： 设置 透传
 * 参    数： 0-关闭、1-开启
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_AT_SetPassThrough(uint8_t value)
{
    if (value == 1)
    {
        uint8_t status = 1;
        status &= ESP8266_SendAT("AT+CIPMODE=1\r\n", "OK", 3000);                         // 传输方式，0_普通传输，1_透明传输
        status &= ESP8266_SendAT("AT+CIPSEND\r\n",    ">", 3000);                         // 数据传输，当已设置透传时，无需参数
        status ? printf("打开透明传输:  成功\r\n") : printf("打开透明传输:  失败\r\n");
        return status;
    }
    ESP8266_SendString("+++");
    printf("已关闭透明传输！\r\n");
    return 1;
}



/******************************************************************************
 * 函  数： ESP8266_Function_JoinAP
 * 功  能： 连接AP
 * 参  数： char* SSID       WiFi名称
 *          char* passWord   WiFi密码
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_Function_JoinAP(char *SSID, char *passWord)
{
    char strTemp[60];
    uint8_t linkStatus = 0;
    // 把ESP8266重新配置成SAT模式
    ESP8266_SendAT("AT+RESTORE\r\n", "ready", 3000)  ? printf("恢复出厂设置:  成功\r\n") : printf("恢复出厂设置:  失败\r\n");   // 恢复模块的出厂设置
    ESP8266_SendAT("AT+CWMODE=1\r\n",   "OK", 3000)  ? printf("配置 STA模式:  成功\r\n") : printf("配置 STA模式:  失败\r\n");   // 工作模式：1_STA, 2_AP, 3_STA+AP
    ESP8266_SendAT("AT+RST\r\n",     "ready", 3000)  ? printf("重启 ESP8266:  成功\r\n") : printf("重启 ESP8266:  失败\r\n");   // 重启模块: 设置工作模式后，需重启才生效
    //ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 3000)  ? printf("单连接模式  :  成功\r\n") : printf("单连接模式  :  失败\r\n");   // 多 连 接: 0_单连接，1_多连接
    //连接至指定WiFi热点
    printf("准备连接Wifi:  %s, %s\r\n", SSID, passWord);
    sprintf(strTemp, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, passWord);
    printf("开始连接AP ... ");
    ESP8266_SendAT(strTemp,        "OK\r\n", 6000)  ? printf("成功\r\n") : printf("失败\r\n");
    // 检测连接状态
    printf("查询连接状态: ");
    linkStatus = ESP8266_Function_GetIPStatus();
    switch (linkStatus)
    {
        case 0:
            printf(" 失败，原因：获取失败！\r\n");
            break;
        case 2:
            printf(" 成功，已获得IP\r\n");
            return 1;
        case 3:
            printf(" 失败，原因：已连接，但未获得IP！\r\n");
            break;
        case 4:
            printf(" 失败，原因：失去连接！\r\n");
            break;
        case 5:
            printf(" 失败，原因：没有连接\r\n");
            break;
        default:
            break;
    }
    return 0;
}



/******************************************************************************
 * 函  数： ESP8266_Function_CreationAP
 * 功  能： 把模块设置成AP模式, 并建立热点网络
 * 参  数： char* SSID       WiFi名称
 *          char* passWord   WiFi密码
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_Function_CreationAP(char *SSID, char *passWord)
{
    char strTemp[60];

    printf("准备建立SSID：%s, %s\r\n", SSID, passWord);
    // 把ESP8266重新配置成AP模式
    ESP8266_SendAT("AT+RESTORE\r\n", "ready", 1000)  ? printf("恢复出厂设置: 成功\r\n") : printf("恢复出厂设置: 失败\r\n");   // 恢复模块的出厂设置
    ESP8266_SendAT("AT+CWMODE=2\r\n",   "OK", 3000)  ? printf("配置为AP模式: 成功\r\n") : printf("配置 STA模式: 失败\r\n");   // 工作模式：1_STA, 2_AP, 3_STA+AP
    ESP8266_SendAT("AT+RST\r\n",     "ready", 3000)  ? printf("重启 ESP8266: 成功\r\n") : printf("重启 ESP8266: 失败\r\n");   // 重启模块: 设置工作模式后，需重启才生效
    // 配置WiFi热点
    sprintf(strTemp, "AT+CWSAP=\"%s\",\"%s\",11,0\r\n", SSID, passWord);
    printf("开始建立AP... ");
    if (ESP8266_SendAT(strTemp, "OK\r\n", 10000))
    {
        printf("成功\r\n");
        return 1;
    }
    else
    {
        printf("失败\r\n");
        return 0;
    }
}



/******************************************************************************
 * 函  数： ESP8266_Function_GetIPStatus
 * 功  能： 获取连接状态
 * 参  数： 无
 * 返  回:  0_获取状态失败
 *          2_获得ip
 *          3_建立连接
 *          4_失去连接
 ******************************************************************************/
uint8_t ESP8266_Function_GetIPStatus(void)
{
    if (ESP8266_SendAT("AT+CIPSTATUS\r\n", "OK", 10000))
    {
        if (strstr((char *)xESP8266.RxData, "STATUS:2"))    return 2;
        if (strstr((char *)xESP8266.RxData, "STATUS:3"))    return 3;
        if (strstr((char *)xESP8266.RxData, "STATUS:4"))    return 4;
        if (strstr((char *)xESP8266.RxData, "STATUS:5"))    return 5;
    }
    return 0;
}



/******************************************************************************
 * 函  数： ESP8266_Function_ConnectTCP
 * 功  能： 建立TCP连接
 * 参  数： char *IP        IP地址
 *          uint16_t port   端口
 * 返  回： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t ESP8266_Function_ConnectTCP(char *IP, uint16_t port)
{
    char strTemp[100];
    uint8_t status = 0;

    printf("建立TCP通信... ");
    sprintf(strTemp, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", IP, port);
    status = ESP8266_SendAT(strTemp, "OK\r\n", 5000);
    if (status)
        printf("成功\r\n");
    else
        printf("失败, 请检查APP：IP地址、端口号、是否已断开旧的连接状态\r\n");
    return status;
}



/******************************************************************************
 * 函  数： ESP8266_Function__GetIP
 * 功  能： 获取公网、局域网IP地址
 * 备  注： 获得的数据，共8个字节
 *          公网IP    [0]、[1]、[2]、[3]
 *          局域网IP  [4]、[5]、[6]、[7]
 * 参  数： 无
 * 返  回:  uint8_t*： 0_获取失败、非0_数据地址
 ******************************************************************************/
uint8_t* ESP8266_Function_GetIP(void)
{
    static uint8_t aData[8];                                     // 用于保存获得的数据
    char strTem[100]  = {0};                                     // 存放IP字符串
    char *strData = NULL;                                        // 数据地址
    char *addr1 = NULL;                                          // 目标数据前的字符串样本地址
    char *addr2 = NULL;                                          // 目标数据后的字符串样本地址

    /** 获取模块在局域网中的IP地址数据 **/
    if (0 == ESP8266_SendAT("AT+CIFSR\r\n",  "OK", 1000))
    {
        printf("获取局域网IP:  失败!\r\n");                      // 查询本地IP地址、MAC地址
        return 0;                                                
    }                                                            
    /** 提取局域网IP的字符串 **/                                 
    strData = (char *)ESP8266_GetRxData();                       // 取得最后一条AT指令发送后返回的数据
    addr1 = strstr(strData, "+CIFSR:STAIP") + 14;                // 从原始数据中，获取IP地址字符串的开始地址; +14是因为要减去这段字符串的长度
    addr2 = strstr(strData, "+CIFSR:STAMAC");                    // 从原始数据中，获取IP地址字符串的结束地址
    memcpy(strTem, addr1, addr2 - addr1 - 3);                    // 提取IP地址段的字符串; -3是因为IP后面有一个双引号和\r\n
    aData[4] = atoi(strtok(strTem, "."));                        // 转换第1个字节，即IP地址的第1个字段：xx.--.--.--
    aData[5] = atoi(strtok(NULL, "."));                          // 转换第2个字节，即IP地址的第2个字段: --.xx.--.--
    aData[6] = atoi(strtok(NULL, "."));                          // 转换第3个字节，即IP地址的第3个字段: --.--.xx.--
    aData[7] = atoi(strtok(NULL, "."));                          // 转换第4个字节，即IP地址的第4个字段: --.--.--.xx

    /** 连接服务器 **/
    if (0 == ESP8266_SendAT("AT+CIPMUX=0\r\n",   "OK", 1000))    // 多连接：0_关闭、1_多连接; 有些模式必须在单连接模式下运行，如透明传输等
    {
        printf("设置单路连接： 失败!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSTART=\"TCP\",\"quan.suning.com\",80\r\n", "OK", 1000))  // 连接服务器,指TCP或UDP的连接
    {
        printf("连接到服务器:  失败!\r\n");     
        return 0;
    }
    /** 获取公网IP数据 **/
    if (0 == ESP8266_SendAT("AT+CIPMODE=1\r\n",  "OK", 1000))    // 透明传输：0_关闭、1_打开
    {                                                        
        printf("打开透传模式:  失败!\r\n");                  
        return 0;                                            
    }                                                        
    if (0 == ESP8266_SendAT("AT+CIPSEND\r\n",    "OK", 1000))    // 传输数据
    {
        printf("准备获取数据:  失败!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("GET http://quan.suning.com/getSysTime.do HTTP/1.1\r\nHost: quan.suning.com\r\n\r\n",   "x-request-ip", 3000))  // 获取数据
    {
        printf("获取公网地址:  失败!\r\n");     
        return 0;
    }
    /** 提取公网IP的字符串 **/
    strData = (char *)ESP8266_GetRxData();                       // 取得最后一条AT指令发送后返回的数据
    addr1 = strstr(strData, "x-request-ip");                     // 从原始数据中，获取IP地址前的字样
    addr2 = strstr(strData, "x-tt-trace-tag");                   // 从原始数据中，获取IP地址后的字样
    memset(strTem, 0, sizeof(strTem));                           // 清0数组的旧数据
    memcpy(strTem, addr1 + 14, addr2 - addr1 - 14);              // 提取IP地址段的字符串
    /** 处理IP字符串 **/                                         
    aData[0] = atoi(strtok(strTem, "."));                        // 转换第1个字节，即IP地址的第1个字段：xx.--.--.--
    aData[1] = atoi(strtok(NULL, "."));                          // 转换第2个字节，即IP地址的第2个字段: --.xx.--.--
    aData[2] = atoi(strtok(NULL, "."));                          // 转换第3个字节，即IP地址的第3个字段: --.--.xx.--
    aData[3] = atoi(strtok(NULL, "."));                          // 转换第4个字节，即IP地址的第4个字段: --.--.--.xx
    /** 断开服务连接 **/                                         
    ESP8266_SendString("+++");                                   // 退出透传
    delay_ms(50);                                                // 稍作延时，因为要等"+++"执行完毕
    if (0 == ESP8266_SendAT("AT+CIPCLOSE\r\n",   "OK", 3000))    // 断开服务器连接, 即断开TCP、UDP
    {                                                        
        printf("断开 TCP连接:  失败!\r\n");                  
        return 0;                                            
    }                                                        
                                                             
    return aData;                                                // 获取成功，返回: uint8_t*：数据地址
}



/******************************************************************************
 * 函  数： ESP8266_Function_GetDate
 * 功  能： 获取当前网络时间
 * 备  注： 获得的数据，共6个字节
 *          [0]年(即：2000+[0]年)、[1]月、[2]日、[3]时、[4]分、[5]秒
 * 参  数： 无             
 * 返  回:  uint8_t*： 0_获取失败、非0_数据地址
 ******************************************************************************/
uint8_t* ESP8266_Function_GetDate(void)
{    
    static uint8_t pDate[6] = {0};
    memset(pDate, 0, 6);
    
    /** 连接服务器 **/     
    if (0 == ESP8266_SendAT("AT+CIPMUX=0\r\n",   "OK", 1000))     // 多 连 接：0_关闭、1_多连接; 有些模式必须在单连接模式下运行，如透明传输等
    {
        printf("设置单路连接： 失败!\r\n");     
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSTART=\"TCP\",\"quan.suning.com\",80\r\n", "OK", 1000))  // 连接服务器,指TCP或UDP的连接
    {
        printf("连接到服务器:  失败!\r\n");                   
        return 0;
    }
    
    /** 获取数据 **/
    if (0 == ESP8266_SendAT("AT+CIPMODE=1\r\n",  "OK", 1000))     // 透明传输：0_关闭、1_打开
    {
        printf("打开透传模式:  失败!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSEND\r\n",    "OK", 1000))     // 开始传输数据
    {
        printf("准备获取数据:  失败!\r\n");    
        return 0;
    }          
    if (0 == ESP8266_SendAT("GET http://quan.suning.com/getSysTime.do HTTP/1.1\r\nHost: quan.suning.com\r\n\r\n",   "sysTime1", 5000))  // 发送指令 
    {
        printf("获取时间数据:  失败!\r\n");                   
        return 0;                                            
    }         
    
    /** 把获取的字符串处理成数值 **/                         
    char strTime[5] = {0};                                        // 定义一个字符数组，用于存放要转换的数据
    char *pStrData = (char *)ESP8266_GetRxData();                 // 取得最后一条AT指令发送后返回的数据
    // 年                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 13, 2);        // 在原数据中，复制年份的字符串: sysTime1字样开始的内存地址，在它后面的第11个字节起的4个字节
    pDate[0] = atof(strTime);                                     // 字符串转换为数值; atof函数是C语言标准库的函数，需要引用头文件：#include <stdlib.h>
    // 月                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 15, 2);        // 在原数据中，复制月份的字符串: sysTime1字样开始的内存地址，在它后面的第15个字节起的2个字节
    pDate[1] = atof(strTime);                                     // 把字符串转换为数值
    // 日                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 17, 2);        // 在原数据中，复制日期的字符串: sysTime1字样开始的内存地址，在它后面的第17个字节起的2个字节
    pDate[2] = atof(strTime);                                     // 把字符串转换为数值
    // 时                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 19, 2);        // 在原数据中，复制时钟的字符串: sysTime1字样开始的内存地址，在它后面的第19个字节起的2个字节
    pDate[3] = atof(strTime);                                     // 把字符串转换为数值
    // 分                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 21, 2);        // 在原数据中，复制分钟的字符串: sysTime1字样开始的内存地址，在它后面的第21个字节起的2个字节
    pDate[4] = atof(strTime);                                     // 把字符串转换为数值
    // 秒                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 23, 2);        // 在原数据中，复制秒钟的字符串: sysTime1字样开始的内存地址，在它后面的第23个字节起的2个字节
    pDate[5] = atof(strTime);                                     // 把字符串转换为数值

    /** 断开服务器连接 **/                                    
    ESP8266_SendString("+++");                                    // 退出透传
    delay_ms(30);                                                 // 稍作延时，等"+++"执行完毕
    if (0 == ESP8266_SendAT("AT+CIPCLOSE\r\n",   "OK", 3000))     // 断开服务器连接, 即断开TCP、UDP
    {                                                             
        printf("断开 TCP连接:  失败!\r\n");                       
        return 0;                                                 
    }                                                             
    return pDate;                                                 // 获取成功，返回: uint8_t*：数据地址
}

