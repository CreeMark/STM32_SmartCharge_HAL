#include "bsp_CAN.h"
#include "stdio.h"
#include "string.h"



xCAN_InfoDef  xCAN1 = {0};      // 定义一个数据结构体，用于管理收、发的数据和信息



/******************************************************************************
 * 函  数： CAN1_SendData
 * 功  能： CAN发送数据函数
 * 参  数： uint8_t* msgData   需发送数据的地址
 *          uint8_t  len       发送的字节数; 最大值：8
 * 返回值： 发送状态; 成功-0、错误-1、忙错误-2、超时-3   
 ******************************************************************************/
uint8_t CAN1_SendData(uint8_t *msgData, uint8_t len)
{ 
    // 定义两个变量
    static uint32_t TxMailbox = 0;                                                 // 用于记录发送成功时所用的邮箱编号：0~2; 被发送函数HAL_CAN_AddTxMessage()赋值
    static uint32_t txStatus  = 0;                                                 // 用于记录发送状态; 成功-0、错误-1、忙错误-2、超时-3; 同上，被发送函数HAL_CAN_AddTxMessage()赋值
    // 限制字节数
    if(len>8)                                                                      // 判断字节是否超过8字节
        len=8;                                                                     // 如果超过8字节，只发送前8个字节
    // 配置帧信息
    xCAN1.TxHeader.ExtId = CAN_TX_ID;                                              // 帧ID, 在bsp_CAN.h中定义
    xCAN1.TxHeader.IDE   = CAN_ID_EXT;                                             // 帧格式; 标准帧: CAN_ID_STD，注意修改上行为StdID; 扩展帧: CAN_ID_EXT, 注意修改上行为ExtID
    xCAN1.TxHeader.RTR   = CAN_RTR_DATA;                                           // 数据帧; 
    xCAN1.TxHeader.DLC   = len;                                                    // 数据字节数，
    xCAN1.TxHeader.TransmitGlobalTime = DISABLE;                                   // 使能时间戳添加到最后两个字节：Data[6]、Data[7]
    // 等待发送邮箱空闲
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);                          // 每次发送前，要等待至有发送邮箱空闲。共有3个发送邮箱。如果返回值为0，即没有发送邮箱空闲，继续等    
    // 发送
    txStatus = HAL_CAN_AddTxMessage(&hcan1, &xCAN1.TxHeader, msgData, &TxMailbox); // 发送   
    // 返回发送状态
    return txStatus;                                                               // 返回发送状态; 成功-0、错误-1、忙错误-2、超时-3   
}



/******************************************************************************
 * 函  数： CAN1_FilterInit
 * 功  能： CAN1筛选器初始化
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef sFilterConfig;

    // 筛选器配置
    sFilterConfig.FilterBank = 0;                                                                            // 筛选器组0 
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                        // 屏蔽位模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                                       // 32位
    sFilterConfig.FilterIdHigh         = (((uint32_t)CAN_RX_ID << 3) & 0xFFFF0000) >> 16;                    // 要筛选的ID高位
    sFilterConfig.FilterIdLow          = (((uint32_t)CAN_RX_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;  // 要筛选的ID低位
    sFilterConfig.FilterMaskIdHigh     = 0x0000;                                                             // 筛选器高16位的位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterMaskIdLow      = 0x0000;                                                             // 筛选器低16位的位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                       // 筛选器被关联到FIFO 0 
    sFilterConfig.FilterActivation = ENABLE;                                                                 // 使能筛选器 
    sFilterConfig.SlaveStartFilterBank = 14;                                                                 // CAN1、CAN2共用28个筛选器(0~27)，此值设置CAN2的筛选器从哪个编号开始; 两个特别的值：当值=0时，不会为CAN1分配任何筛选器，当值=28时，CAN可以使用CAN1的所有筛选器。

    // 初始化筛选器
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)                                              // 初始化筛选器，并检查返回值：成功-0、错误-1、忙错误-2、超时-3    
    {
        Error_Handler();                                                                                     // 错误处理
    }
    
    // 开启CAN接收
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)                         // 开启CAN接收，并检查返回值：成功-0、错误-1、忙错误-2、超时-3  
    {
        Error_Handler();                                                                                     // 错误处理
    }
    
    // 开启CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK)                                                                     // 开启CAN，并检查返回值：成功-0、错误-1、忙错误-2、超时-3  
    {       
        Error_Handler();                                                                                     // 错误处理
    }
}



/******************************************************************************
 * 函  数： HAL_CAN_RxFifo0MsgPendingCallback
 * 功  能： CAN接收中断的回调函数
 *          注意：只有在筛选器中，配置接收规则使用FiFO0时，接收到新一帧数据时才会触发此回调函数
 *          如果使用的是FIFO1, 就会触发HAL_CAN_RxFifo1MsgPendingCallback()
 * 参  数： CAN_HandleTypeDef  *CanNum  
 * 返回值： 无
 ******************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanNum)
{    
    memset(xCAN1.RxData, 0, 9);                                                  // 清0; 存放接收有效数据的数组; CAN一帧数据最大有效负载8字节，数组中开辟9个字节，是为了适配以字符串输出调试信息，最后的1字节0='\0'，是字符串结束符;   
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &xCAN1.RxHeader, xCAN1.RxData);   // 把收到的数据存放到结构体备用，帧信息：RxHeader，数据：RxData
    xCAN1.RxNum = xCAN1.RxHeader.DLC;                                            // 接收字节数、标志位; 外部判断此值接收字节数是否大于0，以判断是否接收到新一帧数据
}


// 文件结尾，需要保留至少1空行
