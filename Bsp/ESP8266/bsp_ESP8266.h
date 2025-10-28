#ifndef __ESP8266__H
#define __ESP8266__H
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【最后版本】  2024-07-08-02
 ** 【淘    宝】  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件名称】  bsp_ESP8266.h
 **
 ** 【 功  能 】  ESP8266通信底层驱动
 **               需搭配: bsp_UART.c 和 bsp_UART.h 两个文件使用
 **               调用本h文件中声明的函数，即可完成初始化、发送、接收.
 **
 ** 【适用平台】  STM32F407 + keil5 + 标准库/标准库
 **
 ** 【串口引脚】  当调用初始化函数：ESP8266_Init(串口、波特率)时, 波特率默认是115200; 而串口参数，有如下6个可选，注意名称的变化：
 **               1- USART1   使用引脚：TX_PA9   RX_PA10       
 **               2- USART2   使用引脚：TX_PA2   RX_PA3
 **               3- USART3   使用引脚：TX_PB10  RX_PB11
 **               4- UART4    使用引脚：TX_PC10  RX_PC11
 **               5- UART5    使用引脚：TX_PC12  RX_PD2
 **               6- USART6   使用引脚：TX_PC6   RX_PC7
 **
 ** 【移植说明】  1- 如果使用CubeMX配置的工程，无须对UART进行任何配置。
 **               2- 不管是HAL库、标准库工程，均可通过下面操作，实现移植和使用。
 **               3- 复制本工程路径文件夹BSP下的两个子文件夹：ESP8266、UART，粘贴到目标工程文件夹中;
 **               4- 添加头文件路径： Keil > Option > C/C++ > Include Paths，添加刚才两个子文件夹的路径地址;
 **               5- 添加C文件到工程: Keil > 左侧工程管理器中双击目标文件夹 > 选择 bsp_ESP8266.c 和 bsp_UART.c ;
 **               6- 添加文件引用:    #include "bsp_ESP8266.h" 和 #include "bsp_uart.h＂，即哪个文件要用ESP8266功能，就在其代码文件顶部添加引用;
 **
 ** 【代码使用】  调用以下基础函数：可完成AT模式下的几乎所有通信操作。
 **               void      ESP8266_Init(USART_TypeDef *USARTx, uint32_t baudrate);     // 初始化所用串口、波特率
 **               void      ESP8266_SendString(char *str);                              // 发送字符串
 **               uint8_t   ESP8266_SendAT(char *cmd, char *res, uint16_t timeOut);     // 发送AT命令，并等待返回期待的信息
 **               uint16_t  ESP8266_GetRxNum(void);                                     // 获取接收到的字节数，当字节数>0时，表示接收到新一帧数据
 **               uint8_t  *ESP8266_GetRxData(void);                                    // 获取接收到的数据(缓存的地址)
 **               void      ESP8266_ClearRx(void);                                      // 清理ESP8266的接收缓存，包括接收长度变量和数据存放缓存
 **
 ** 【更新记录】  2024-06-17  修改函数、注释;
 **
************************************************************************************************************************************/
#include "stdio.h"        //  引用C语言标准库，它定义了标准输入输出函数; 如：printf、scanf、sprintf、fopen 等; 
#include "stdarg.h"       //  引用C语言标准库，它定义了处理可变参数的宏; 如：va_start、va_arg、va_end、va_list 等;
#include "bsp_UART.h"




/*****************************************************************************
 ** 声明全局函数
 ** 说明：函数看着比较多，但实际操作时，通过6个基础函数，即可完成绝大部分操作;
 **       扩展函数，是对常用功能的封装，其内部也是通过基础函数实现;
****************************************************************************/
// 基础函数：可完成AT命令下绝大部分操作
void      ESP8266_Init(USART_TypeDef *USARTx, uint32_t baudrate);          // 初始化所用串口、波特率
void      ESP8266_SendString(const char *pcString, ...);                   // 发送字符串; 参数：格式如同printf; 
uint8_t   ESP8266_SendAT(char *pcStr, char *pcRes, uint16_t usTimeOut);    // 发送AT命令，并等待返回期待的信息
uint16_t  ESP8266_GetRxNum(void);                                          // 获取接收到的字节数，当字节数>0时，表示接收到新一帧数据
uint8_t*  ESP8266_GetRxData(void);                                         // 获取接收到的数据(缓存的地址)
void      ESP8266_ClearRx(void);                                           // 清理ESP8266的接收缓存，包括接收长度变量和数据存放缓存
// 扩展函数1：常用AT指令封装                                               
uint8_t   ESP8266_AT(void);                                                // 发送AT命令，可用于测试线路连接是否正常，和测试代码收发机制的正确性
uint8_t   ESP8266_AT_RESTORE(void);                                        // 恢复出厂设置; 可用于清除芯片中已保存的wifi名称、密码
uint8_t   ESP8266_AT_RST(void);                                            // 复位、重启
uint8_t   ESP8266_AT_CWMODE(uint8_t mode);                                 // 设置工作模式; 参数：1-STA、2-AP、3-STA+AP
uint8_t   ESP8266_AT_CIPMUX(uint8_t value);                                // 设置：多连接，0_单连接，1_多连接;备注：透传必须AT+CIPMUX=0;
uint8_t   ESP8266_AT_CIPMODE(uint8_t value);                               // 设置：传输方式，0_普通传输，1_透明传输
uint8_t   ESP8266_AT_SetPassThrough(uint8_t value);                        // 设置 透传; 参数：0-关、1-开; 返回：0-执行失败、1-执行成功
// 扩展函数2：常用功能 (基于上面的基础函数实现)                            
uint8_t   ESP8266_Function_JoinAP(char *SSID, char *passWord);             // 配置为 STA模式，加入某热点
uint8_t   ESP8266_Function_CreationAP(char *SSID, char *passWord);         // 配置为 AP 模式，建立热点
uint8_t   ESP8266_Function_GetIPStatus(void);                              // 获取连接状态
uint8_t   ESP8266_Function_ConnectTCP(char *IP, uint16_t port);            // 以TCP通信方式与目标连接
uint8_t*  ESP8266_Function_GetIP(void);                                    // 获取公网、局域网IP地址; 返回：0-失败、非0值-数据地址; 数据共8个字节, 公网IP：[0]、[1]、[2]、[3], 局域网IP：[4]、[5]、[6]、[7]
uint8_t*  ESP8266_Function_GetDate(void);                                  // 获取当前网络时间; 返回：0-失败、非0值-数据地址; 数据共6个字节：[0]年(即：2000+[0]年)、[1]月、[2]日、[3]时、[4]分、[5]秒




#endif


