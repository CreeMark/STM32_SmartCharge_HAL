#ifndef DHT11_H
#define DHT11_H

#include "main.h"

// 核心修改：从PA0改为PB4，同步调整GPIO端口和引脚配置
// PB4对应GPIOB的CRL寄存器（引脚0-7用CRL，8-15用CRH），掩码需对应Bit16-19（PB4的4*4=16）
#define DHT11_IO_IN()  {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=0X00080000;}  // PB4输入模式（浮空输入）
#define DHT11_IO_OUT() {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=0X00030000;}  // PB4推挽输出模式（50MHz）


// IO操作函数：端口改为GPIOB，引脚改为GPIO_PIN_4
#define	DHT11_DQ_OUT(X)  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, X)
#define	DHT11_DQ_IN      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)


// 函数声明（无修改，保持与源文件一致）
uint8_t DHT11_Init(void);//初始化DHT11
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);//读取数据
uint8_t DHT11_Read_Byte(void);//读取一个字节
uint8_t DHT11_Read_Bit(void);//读取一位
uint8_t DHT11_Check(void);//检测DHT11
void DHT11_Rst(void);//复位DHT11   

#endif
