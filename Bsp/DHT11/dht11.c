/***************STM32F103C8T6**********************
 * 文件名  ：DHT11.c
 * 描述    ：DHT11传感器
 * 备注    : DHT11温度湿度传感器
 * 接口    ：PB4-DATA  // 已从PA0修改为PB4

********************LIGEN*************************/

#include "dht11.h"
#include "delay.h"

// 1. 修改引脚定义：从GPIO_Pin_0（PA0）改为GPIO_PIN_4（PB4）
#define DT GPIO_PIN_4
// 新增端口定义：明确使用GPIOB（原代码默认GPIOA，此处显式定义方便后续IO方向切换）
#define DHT11_PORT GPIOB
      
//复位DHT11（无修改，仅依赖DT和DHT11_PORT定义）
void DHT11_Rst(void)	   
{                 
	DHT11_IO_OUT(); 	//SET OUTPUT
	DHT11_DQ_OUT(GPIO_PIN_RESET); 	//拉低DQ
	delay_ms(20);    	//拉低至少18ms
	DHT11_DQ_OUT(GPIO_PIN_SET); 	//DQ=1 
	delay_us(30);     	//主机拉高20~40us
}

//等待DHT11的回应（无修改，仅依赖DT和DHT11_PORT定义）
//返回1:未检测到DHT11的存在
//返回0:存在
uint8_t DHT11_Check(void) 	   
{   
	uint8_t retry=0;
	DHT11_IO_IN();//SET INPUT	 
    while (DHT11_DQ_IN&&retry<100)//DHT11会拉低40~80us
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=100)return 1;
	else retry=0;
    while (!DHT11_DQ_IN&&retry<100)//DHT11拉低后会再次拉高40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}

//从DHT11读取一个位（无修改，仅依赖DT和DHT11_PORT定义）
//返回值：1/0
uint8_t DHT11_Read_Bit(void) 			 
{
 	uint8_t retry=0;
	while(DHT11_DQ_IN&&retry<100)//等待变为低电平
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!DHT11_DQ_IN&&retry<100)//等待变高电平
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//等待40us
	if(DHT11_DQ_IN)return 1;
	else return 0;		   
}

//从DHT11读取一个字节（无修改，依赖Read_Bit函数）
//返回值：读到的数据
uint8_t DHT11_Read_Byte(void)    
{        
	uint8_t i,dat;
	dat=0;
	for (i=0;i<8;i++) 
	{
		dat<<=1; 
		dat|=DHT11_Read_Bit();
	}						    
	return dat;
}

//从DHT11读取一次数据（无修改，依赖Read_Byte函数）
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi)    
{        
 	uint8_t buf[5];
	uint8_t i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//读取40位数据
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];
			*temp=buf[2];
		}
	}
	else return 1;
	return 0;	    
}

//初始化DHT11的IO口 DQ 同时检测DHT11的存在（核心修改处）
//返回1:不存在
//返回0:存在    	 
uint8_t DHT11_Init(void)
{	 

    GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 2. 修改时钟使能：从GPIOA时钟改为GPIOB时钟 */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* 3. 修改引脚：从GPIOA的Pin0改为GPIOB的Pin4 */
  HAL_GPIO_WritePin(DHT11_PORT, DT, GPIO_PIN_SET);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = DT;  // 使用修改后的DT（PB4）
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);  // 端口改为DHT11_PORT（GPIOB）

  // 延迟一段时间让传感器稳定
  delay_ms(100);

  DHT11_Rst();  //复位DHT11
	uint8_t check_result = DHT11_Check();//等待DHT11的回应
  
  // 调试信息：输出初始化结果
  if(check_result == 0) {
      // 成功
  } else {
      // 失败 - 可能是硬件连接问题或时序问题
  }
  
  return check_result;
} 
