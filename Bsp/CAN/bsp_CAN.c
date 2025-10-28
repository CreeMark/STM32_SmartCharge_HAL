#include "bsp_CAN.h"
#include "stdio.h"
#include "string.h"



xCAN_InfoDef  xCAN1 = {0};      // ����һ�����ݽṹ�壬���ڹ����ա��������ݺ���Ϣ



/******************************************************************************
 * ��  ���� CAN1_SendData
 * ��  �ܣ� CAN�������ݺ���
 * ��  ���� uint8_t* msgData   �跢�����ݵĵ�ַ
 *          uint8_t  len       ���͵��ֽ���; ���ֵ��8
 * ����ֵ�� ����״̬; �ɹ�-0������-1��æ����-2����ʱ-3   
 ******************************************************************************/
uint8_t CAN1_SendData(uint8_t *msgData, uint8_t len)
{ 
    // ������������
    static uint32_t TxMailbox = 0;                                                 // ���ڼ�¼���ͳɹ�ʱ���õ������ţ�0~2; �����ͺ���HAL_CAN_AddTxMessage()��ֵ
    static uint32_t txStatus  = 0;                                                 // ���ڼ�¼����״̬; �ɹ�-0������-1��æ����-2����ʱ-3; ͬ�ϣ������ͺ���HAL_CAN_AddTxMessage()��ֵ
    // �����ֽ���
    if(len>8)                                                                      // �ж��ֽ��Ƿ񳬹�8�ֽ�
        len=8;                                                                     // �������8�ֽڣ�ֻ����ǰ8���ֽ�
    // ����֡��Ϣ
    xCAN1.TxHeader.ExtId = CAN_TX_ID;                                              // ֡ID, ��bsp_CAN.h�ж���
    xCAN1.TxHeader.IDE   = CAN_ID_EXT;                                             // ֡��ʽ; ��׼֡: CAN_ID_STD��ע���޸�����ΪStdID; ��չ֡: CAN_ID_EXT, ע���޸�����ΪExtID
    xCAN1.TxHeader.RTR   = CAN_RTR_DATA;                                           // ����֡; 
    xCAN1.TxHeader.DLC   = len;                                                    // �����ֽ�����
    xCAN1.TxHeader.TransmitGlobalTime = DISABLE;                                   // ʹ��ʱ�����ӵ���������ֽڣ�Data[6]��Data[7]
    // �ȴ������������
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);                          // ÿ�η���ǰ��Ҫ�ȴ����з���������С�����3���������䡣�������ֵΪ0����û�з���������У�������    
    // ����
    txStatus = HAL_CAN_AddTxMessage(&hcan1, &xCAN1.TxHeader, msgData, &TxMailbox); // ����   
    // ���ط���״̬
    return txStatus;                                                               // ���ط���״̬; �ɹ�-0������-1��æ����-2����ʱ-3   
}



/******************************************************************************
 * ��  ���� CAN1_FilterInit
 * ��  �ܣ� CAN1ɸѡ����ʼ��
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef sFilterConfig;

    // ɸѡ������
    sFilterConfig.FilterBank = 0;                                                                            // ɸѡ����0 
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                                                        // ����λģʽ
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                                                       // 32λ
    sFilterConfig.FilterIdHigh         = (((uint32_t)CAN_RX_ID << 3) & 0xFFFF0000) >> 16;                    // Ҫɸѡ��ID��λ
    sFilterConfig.FilterIdLow          = (((uint32_t)CAN_RX_ID << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF;  // Ҫɸѡ��ID��λ
    sFilterConfig.FilterMaskIdHigh     = 0x0000;                                                             // ɸѡ����16λ��λƥ�䣬λ0-��λ��ͨ����λ1-��λ��Ҫ��IDλ��ͬ
    sFilterConfig.FilterMaskIdLow      = 0x0000;                                                             // ɸѡ����16λ��λƥ�䣬λ0-��λ��ͨ����λ1-��λ��Ҫ��IDλ��ͬ
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                                                       // ɸѡ����������FIFO 0 
    sFilterConfig.FilterActivation = ENABLE;                                                                 // ʹ��ɸѡ�� 
    sFilterConfig.SlaveStartFilterBank = 14;                                                                 // CAN1��CAN2����28��ɸѡ��(0~27)����ֵ����CAN2��ɸѡ�����ĸ���ſ�ʼ; �����ر��ֵ����ֵ=0ʱ������ΪCAN1�����κ�ɸѡ������ֵ=28ʱ��CAN����ʹ��CAN1������ɸѡ����

    // ��ʼ��ɸѡ��
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)                                              // ��ʼ��ɸѡ��������鷵��ֵ���ɹ�-0������-1��æ����-2����ʱ-3    
    {
        Error_Handler();                                                                                     // ������
    }
    
    // ����CAN����
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)                         // ����CAN���գ�����鷵��ֵ���ɹ�-0������-1��æ����-2����ʱ-3  
    {
        Error_Handler();                                                                                     // ������
    }
    
    // ����CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK)                                                                     // ����CAN������鷵��ֵ���ɹ�-0������-1��æ����-2����ʱ-3  
    {       
        Error_Handler();                                                                                     // ������
    }
}



/******************************************************************************
 * ��  ���� HAL_CAN_RxFifo0MsgPendingCallback
 * ��  �ܣ� CAN�����жϵĻص�����
 *          ע�⣺ֻ����ɸѡ���У����ý��չ���ʹ��FiFO0ʱ�����յ���һ֡����ʱ�Żᴥ���˻ص�����
 *          ���ʹ�õ���FIFO1, �ͻᴥ��HAL_CAN_RxFifo1MsgPendingCallback()
 * ��  ���� CAN_HandleTypeDef  *CanNum  
 * ����ֵ�� ��
 ******************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanNum)
{    
    memset(xCAN1.RxData, 0, 9);                                                  // ��0; ��Ž�����Ч���ݵ�����; CANһ֡���������Ч����8�ֽڣ������п���9���ֽڣ���Ϊ���������ַ������������Ϣ������1�ֽ�0='\0'�����ַ���������;   
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &xCAN1.RxHeader, xCAN1.RxData);   // ���յ������ݴ�ŵ��ṹ�屸�ã�֡��Ϣ��RxHeader�����ݣ�RxData
    xCAN1.RxNum = xCAN1.RxHeader.DLC;                                            // �����ֽ�������־λ; �ⲿ�жϴ�ֵ�����ֽ����Ƿ����0�����ж��Ƿ���յ���һ֡����
}


// �ļ���β����Ҫ��������1����
