#include "usart.h"
#include "sys.h"
//#include "led.h"
#include "stdio.h"	 


//串口1初始化 无接收
void USART1_Init(u32 pclk2, u32 bound)
{
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
	mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置 
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断 
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
}


// printf重定向已由bsp_UART.c实现，此处注释避免符号冲突
#if 0
#pragma import(__use_no_semihosting)   
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef' d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定向fputc函数
//printf的输出，指向fputc，由fputc输出到串口
//这里使用串口1(USART1)输出printf信息
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//等待上一次串口数据发送完成  
	USART1->DR = (u8) ch;      	//写DR,串口1将发送数据
	return ch;
}
#endif


//串口2初始化
void USART2_Init( u32 pclk2, u32 bound )
{
	float temp;
	u16 integer;//usartdiv整数
	u16 decimal;//usartdiv小数
	
	temp = (pclk2*1000000)/(bound*16);////得到usartdiv
	integer = temp;//得到整数部分
	decimal = (temp-integer)*16;//得到小数部分(小数部分向前移动四位)
	bound = decimal+(integer<<4);//得到配置波特率要写入usartdiv的值
	
	RCC->APB1ENR |= 1<<17;//开启USART2的时钟
	RCC->APB1RSTR |= 1<<17;//串口2复位
	RCC->APB1RSTR &= ~(1<<17);//关闭复位
	RCC->APB2ENR |= 1<<2;//打开PA的时钟
	
	GPIOA->CRL &= ~(0xFF<<8);//复位PA2 3的状态
	GPIOA->CRL |= 0xb<<8;//设置PA2为tx
	GPIOA->CRL |= 4<<12;//设置PA3为RX

	USART2->BRR = bound;//波特率设置
	USART2->CR1 = 0;//复位串口2的设置
	USART2->CR1 |= 1<<2;//接收使能
	USART2->CR1 |= 1<<3;//发送使能
	USART2->CR1 |= 1<<5;//开启接收中断
	USART2->CR1 |= 1<<13;//USART2使能
	// MY_NVIC_Init(2,2,USART2_IRQn,2);//抢占2响应2 // 已在USART2_Init内部配置中断优先级
}



//串口3初始化
void USART3_Init( u32 pclk2, u32 bound )
{
	float temp;
	u16 integer;//usartdiv整数
	u16 decimal;//usartdiv小数
	
	temp = (pclk2*1000000)/(bound*16);////得到usartdiv
	integer = temp;//得到整数部分
	decimal = (temp-integer)*16;//得到小数部分(小数部分向前移动四位)
	bound = decimal+(integer<<4);//得到配置波特率要写入usartdiv的值
	
	RCC->APB1ENR |= 1<<18;//开启USART3的时钟
	RCC->APB1RSTR |= 1<<18;//串口3复位
	RCC->APB1RSTR &= ~(1<<18);//关闭复位
	RCC->APB2ENR |= 1<<3;//打开PB的时钟
	
	GPIOB->CRH &= 0xFFFF00FF;//复位PB10 11
	GPIOB->CRH |= 0x00008B00;//设置为输出

	USART3->BRR = bound;//波特率设置
	USART3->CR1 = 0;//复位串口3的设置
	USART3->CR1 |= 1<<2;//接收使能
	USART3->CR1 |= 1<<3;//发送使能
	USART3->CR1 |= 1<<5;//开启接收中断
	USART3->CR1 |= 1<<13;//USART3使能
	// MY_NVIC_Init(2,2,USART3_IRQn,2);//抢占2响应2 // 已在USART3_Init内部配置中断优先级
}


//USART3发送ascll类型的数据
//data为字符串的首地址
void USART3_Txdata( u8 *data )
{
	u8 i=0;
	while(data[i] != '\0')
	{
		USART3->DR = data[i];//发送一个字节
		while( (USART3->SR&0x40)==0);//等待发送完毕
		i++;
	}
}


//USART3发送ascll类型的数据
//data为字符串的首地址
void USART3_TxdataN( u8 *data,u8 nmb )
{
	u8 i=0;
	for( i=0; i<nmb; i++ )
	{
		USART3->DR = data[i];//发送一个字节
		while( (USART3->SR&0x40)==0);//等待发送完毕
	}
}


//USART1发送ascll类型的数据
//data为字符串的首地址
void USART1_Txdata( u8 *data )
{
	u8 i=0;
	while(data[i] != '\0')
	{
		USART1->DR = data[i];//发送一个字节
		while( (USART1->SR&0x40)==0);//等待发送完毕
		i++;
	}
}


//USART2发送ascll类型的数据
//data为字符串的首地址
void USART2_Txdata( u8 *data )
{
	u8 i=0;
	while(data[i] != '\0')
	{
		USART2->DR = data[i];//发送一个字节
		while( (USART2->SR&0x40)==0);//等待发送完毕
		i++;
	}
}


//USART1发送ascll类型的数字
//number是要写入的数字
void USART1_Txnumber(u32 number)
{
	u32 data=1;
	u8 bit;
	if(number<=0)
	{
		USART1->DR='0';
		while((USART1->SR&0x40)==0);
		return;
	}
	while(number>=data)data*=10;//计算长度
	while(1)
	{
		data /= 10;
		bit = number/data;
		number %= data;
		USART1->DR=bit+'0';
		while((USART1->SR&0x40)==0);
		if(data==1)return;
	}
}







