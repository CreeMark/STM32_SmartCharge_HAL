#ifndef __USART_H__
#define __USART_H__

#include "sys.h"


void USART1_Init(u32 pclk2, u32 bound);//����1��ʼ��
void USART1_Txdata( u8 *data );//����һ���ַ�
void USART1_Txnumber(u32 number);//USART1����ascll���͵�����
void USART2_Init( u32 pclk2, u32 bound );//����2��ʼ�� ��ICͨ�� ���ݽ������ж���ִ��
void USART3_Init( u32 pclk2, u32 bound );//����3��ʼ��
void USART3_Txdata( u8 *data );//USART3����ascll���͵�����
void USART3_TxdataN( u8 *data,u8 nmb );//USART3����ascll���͵�����
//USART2����ascll���͵�����
//dataΪ�ַ������׵�ַ
void USART2_Txdata( u8 *data );

#endif
