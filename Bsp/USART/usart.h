#ifndef __USART_H__
#define __USART_H__

#include "sys.h"


void USART1_Init(u32 pclk2, u32 bound);//串口1初始化
void USART1_Txdata( u8 *data );//发送一串字符
void USART1_Txnumber(u32 number);//USART1发送ascll类型的数字
void USART2_Init( u32 pclk2, u32 bound );//串口2初始化 与IC通信 数据接收再中断中执行
void USART3_Init( u32 pclk2, u32 bound );//串口3初始化
void USART3_Txdata( u8 *data );//USART3发送ascll类型的数据
void USART3_TxdataN( u8 *data,u8 nmb );//USART3发送ascll类型的数据
//USART2发送ascll类型的数据
//data为字符串的首地址
void USART2_Txdata( u8 *data );

#endif
