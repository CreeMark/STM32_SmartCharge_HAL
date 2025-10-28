/***********************************************************************************************************************************
 ** �������д��  ħŮ�������Ŷ�
 ** �����汾��  2024-07-08-02
 ** ����    ����  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_ESP8266.c
 **
 ** ���ļ����ܡ�  ��ESP8266����ATָ����� �����ܣ����к�����װ
 **
 ** ������ƽ̨��  STM32F407 + keil5 + HAL��/��׼��
 **         
************************************************************************************************************************************/
#include "bsp_ESP8266.h"
#include "stdlib.h"           // C���Եı�׼�⺯��������atoi�Ⱥ��� 



/*****************************************************************************
 ** ȫ�ֱ���
****************************************************************************/
#define ESP8266_RX_BUF_SIZE       1024              // ���ݽ��ջ�������С���󲿷�����¶������޸�

typedef struct
{
    uint8_t         Flag;                           // ״̬���; 0=δ��ʼ�����쳣, 1=����
    uint16_t        RxNum;                          // ���յ����ٸ��ֽ�����; 0-�����ݣ���0_���յ����ֽ���
    uint8_t         RxData[ESP8266_RX_BUF_SIZE];    // ���յ����ݵĻ���; ESP8266��ATģʽ�£�ÿ֡�����Ϊ1056���ֽ�;
    char           *APName;                         // �����������APʱ��: SSID
    char           *APPassword;                     // �����������APʱ��: ����
    uint32_t        Baudrate;                       // ��¼���õĴ��ڲ�����
    USART_TypeDef  *UARTx;                          // ��¼���õĶ˿�
} xESP8266_TypeDef;

xESP8266_TypeDef   xESP8266 = {0} ;                 // ����ṹ��




/******************************************************************************
 * ��  ���� delay_ms
 * ��  �ܣ� ms ��ʱ����
 * ��  ע�� 1��ϵͳʱ��168MHz
 *          2���򹴣�Options/ c++ / One ELF Section per Function
            3�������Ż�����Level 3(-O3)
 * ��  ���� uint32_t  ms  ����ֵ
 * ��  �أ� ��
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // ʹ��volatile��������ֹ�������������Ż�
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // �����ⲿ��������ֹ��ѭ�����������Ż���
}



/******************************************************************************
 * ��  ���� ESP8266_Init
 * ��  �ܣ� ESP8266��ʼ����ͨ�����š�UART��Э��������ж����ȼ�
 *          Э�飺������-None-8-1
 *          ���ͣ������ж�
 *          ���գ�����+�����ж�
 * ��  ���� USART_TypeDef  *USARTx    ���ڶ˿ڣ�USART1��USART2��USART3��UART4��UART5��USART6
 *          uint32_t        baudrate  ͨ�Ų�����
 * ��  �أ� ��
 ******************************************************************************/
void ESP8266_Init(USART_TypeDef *USARTx, uint32_t baudrate)
{
    printf("\r\nESP8266 ��ʼ����\r\n");
    delay_ms(200);                                                 // ��Ҫ���ϵ�󣬱�������ʱ�Եȴ�8266�ȶ����ɹ���

    if (USARTx == USART1)    UART1_Init(baudrate);
    if (USARTx == USART2)    UART2_Init(baudrate);
    if (USARTx == USART3)    UART3_Init(baudrate);
    if (USARTx == UART4)     UART4_Init(baudrate);
    if (USARTx == UART5)     UART5_Init(baudrate);

    xESP8266.Flag = 0;                                             // ��ʼ��״̬
    xESP8266.UARTx   = USARTx;                                     // ��¼���ô��ڶ˿�
    xESP8266.Baudrate = baudrate;                                  // ��¼���õĲ�����
}



/******************************************************************************
 * ��  ���� ESP8266_SendString
 * ��  �ܣ� �����ַ���;
 *          �÷���ο�printf����ʾ���е�չʾ
 *          ע�⣬������ֽ���Ϊ512-1���ַ������ں������޸�����
 * ��  ���� const char *pcString, ...   (��ͬprintf���÷�)
 * ��  �أ� ��
 ******************************************************************************/
void ESP8266_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                                                // ����һ������, ���������������0
    va_list ap;                                                                              // �½�һ���ɱ�����б�
    va_start(ap, pcString);                                                                  // �б�ָ���һ���ɱ����
    vsnprintf(mBuffer, 512, pcString, ap);                                                   // �����в���������ʽ�����������; ����2�������Ʒ��͵�����ֽ���������ﵽ���ޣ���ֻ��������ֵ-1; ���1�ֽ��Զ���'\0'
    va_end(ap);                                                                              // ��տɱ�����б�

    if (xESP8266.UARTx == USART1)    UART1_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // ���ֽڴ�Ż��λ��壬�Ŷ�׼������
    if (xESP8266.UARTx == USART2)    UART2_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == USART3)    UART3_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == UART4)     UART4_SendData((uint8_t *)mBuffer, strlen(mBuffer));
    if (xESP8266.UARTx == UART5)     UART5_SendData((uint8_t *)mBuffer, strlen(mBuffer));
}



/******************************************************************************
 * ��  ���� ESP8266_SendAT
 * ��  �ܣ� ��ESP8266ģ�鷢��AT����, ���ȴ�������Ϣ
 * ��  ���� char     *pcAT    : ATָ���ַ���
 *          char     *pcACK    : �ڴ���ָ�����Ϣ�ַ���
 *          uint16_t  usTimeOut: ���������ȴ���ʱ�䣬����       
 * ��  �أ� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_SendAT(char *pcAT, char *pcACK, uint16_t usTimeOut)
{
    ESP8266_ClearRx();                                     // ��0 ���յ��ֽ���������
    ESP8266_SendString(pcAT);                              // ����ATָ��

    while (usTimeOut--)                                    // �ȴ�ָ���ִ�����
    {
        if (ESP8266_GetRxNum())                            // �жϽ���
        {
            ESP8266_ClearRx();                             // ��ս����ֽ���; ע�⣺���յ����������� ����û�б���0��
            if (strstr((char *)xESP8266.RxData, pcACK))    // �յ�����ȷ��
                return 1;                                  // ���أ�1��ָ������ɹ�
        }
        delay_ms(1);                                       // ��ʱ; ���ڳ�ʱ�˳�������������
    }
    ESP8266_ClearRx();                                     // ��0 ���յ��ֽ���������
    return 0;                                              // ���أ�0��ָ�����ʧ��
}



/******************************************************************************
 * ��  ���� ESP8266_GetRxNum
 * ��  �ܣ� ��ȡ����һ֡���ֽ������������жϼ���Ƿ��յ��µ�����
 * ��  ���� ��
 * ��  �أ� ���ؽ��յ����ֽ���
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
 * ��  ���� ESP8266_GetRxData
 * ��  �ܣ� ��ȡ����һ֡������ (��ַ��
 * ��  ���� ��
 * ��  �أ� �����ַ (uint8_t*)
 ******************************************************************************/
uint8_t *ESP8266_GetRxData(void)
{
    return xESP8266.RxData ;
}



/******************************************************************************
 * ��  ���� ESP8266_ClearRx
 * ��  �ܣ� ��0���յ������һ֡�Ľ����ֽ���;
 * ��  ���� ��
 * ��  �أ� ��
 ******************************************************************************/
void ESP8266_ClearRx(void)
{

    xESP8266.RxNum = 0;                                    // ��0�������ֽ���
    //memset(xESP8266.RxData, 0, ESP8266_RX_BUF_SIZE);     // ��0�����ջ���
}




//////////////////////////////////////////////////////////////  ����ATָ���װ   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * ��    ���� ESP8266_AT
 * ��    �ܣ� ����AT���AT
 *            �����ڲ�����·�����Ƿ��������Ͳ��Դ����շ����Ƶ���ȷ��
 * ��    ���� ��
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_AT(void)
{
    xESP8266.Flag = ESP8266_SendAT("AT\r\n",   "OK", 1500) ;       // ����ATָ����ж��Ƿ�������ESP8266

    if (xESP8266.Flag)
    {
        printf("ģ�����Ӳ���:  �ɹ�\r\n");                         // ATָ����Գɹ�
    }
    else
    {
        printf("ģ�����Ӳ���:  ʧ�ܣ�    ");                       // ATָ�����ʧ��
        printf("��飺ESP8266��USART�ĵ�·���ӡ�����ñ����\r\n");  // ���������
    }

    return xESP8266.Flag;                                          // ����, ��ʼ��״̬��0_ʧ�ܣ�1_����
}



/******************************************************************************
 * ��    ���� ESP8266_AT_RESTORE
 * ��    �ܣ� ����ATָ�AT+RESTORE
 *            �ָ�ģ��ĳ�������
 *            ���������оƬ���ѱ����wifi���ơ�����
 * ��    ���� ��
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_AT_RESTORE(void)
{
    uint8_t status = 0 ;
    status = ESP8266_SendAT("AT+RESTORE\r\n", "ready", 3000);                         // �ָ�ģ��ĳ�������
    status ? printf("�ָ���������:  �ɹ�\r\n") : printf("�ָ���������:  ʧ��\r\n");   // �����ʾ��Ϣ
    return status;                                                                    // ���أ�0-ʧ�ܡ�1�ɹ�
}



/******************************************************************************
 * ��    ���� ESP8266_AT_RST
 * ��    �ܣ� ����ATָ��: AT+RST
 *            ��λ������
 * ��    ���� ��
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_AT_RST(void)
{
    uint8_t status = 0 ;
    status = ESP8266_SendAT("AT+RST\r\n", "ready", 3000);                            // �ָ�ģ��ĳ�������
    status ? printf("���� ESP8266:  �ɹ�\r\n") : printf("���� ESP8266:  ʧ��\r\n");  // �����ʾ��Ϣ
    return status;                                                                   // ���أ�0-ʧ�ܡ�1�ɹ�
}



/******************************************************************************
 * ��    ���� ESP8266_AT_CWMODE
 * ��    �ܣ� ����ESP8266�Ĺ���ģʽ
 * ��    ���� 1-STAģʽ
 *            2-APģʽ
 *            3-STA+AP
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_AT_CWMODE(uint8_t mode)
{
    uint8_t status = 0 ;
    char str[100] = {0};

    if (mode > 3)
    {
        printf("���ù���ģʽ:  ʧ�ܣ�������Χ��1��2��3; �������ͣ�1-STA, 2-AP, 3-STA+AP\r\n");
    }

    sprintf(str, "AT+CWMODE=%d\r\n", mode);                                                // ����ģʽ��1-STA, 2-AP, 3-STA+AP
    status = ESP8266_SendAT(str,    "OK", 3000)  ;                                         // ���͸�8266�����ж�ָ���Ƿ�ִ�гɹ�
    if (mode == 1)
        status ?  printf("���� STAģʽ:  �ɹ�\r\n") : printf("���� STAģʽ:  ʧ��\r\n");   // �����ʾ��Ϣ
    if (mode == 2)
        status ?  printf("���� AP ģʽ:  �ɹ�\r\n") : printf("���� AP ģʽ:  ʧ��\r\n");   // �����ʾ��Ϣ
    if (mode == 3)
        status ?  printf("���� STA+AP :  �ɹ�\r\n") : printf("���� STA+AP :  ʧ��\r\n");   // �����ʾ��Ϣ
    return status;                                                                         // ���أ�0-ʧ�ܡ�1�ɹ�
}



/******************************************************************************
 * ��    ���� ESP8266_AT_CIPMUX
 * ��    �ܣ� ���� ������
 * ��    ע�� ͸������AT+CIPMUX=0;
 * ��    ���� uint8_t value   0_�����ӣ�1_������
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t  ESP8266_AT_CIPMUX(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMUX=%d\r\n", value);
    ESP8266_SendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("�������ӷ�ʽ:  �ɹ�\r\n") : printf("�������ӷ�ʽ: ʧ��\r\n");   // �� �� ��: 0_�����ӣ�1_������
    return flag;
}



/******************************************************************************
 * ��    ���� ESP8266_AT_CIPMODE
 * ��    �ܣ� ���� ���䷽ʽ
 * ��    ���� uint8_t value   0_��ͨ���䣬1_͸������
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t   ESP8266_AT_CIPMODE(uint8_t value)
{
    uint8_t flag = 0;
    char strTemp[50];
    sprintf(strTemp, "AT+CIPMODE=%d\r\n", value);
    ESP8266_SendAT(strTemp, "OK", 3000)  ? (flag = 1) : (flag = 0) ;
    flag ? printf("���ô��䷽ʽ:  �ɹ�\r\n") : printf("���ô��䷽ʽ: ʧ��\r\n");   // ���䷽ʽ��0_��ͨ���䣬1_͸������
    return flag;
}



/******************************************************************************
 * ��    ���� ESP8266_AT_SetPassThrough
 * ��    �ܣ� ���� ͸��
 * ��    ���� 0-�رա�1-����
 *
 * �� �� ֵ�� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_AT_SetPassThrough(uint8_t value)
{
    if (value == 1)
    {
        uint8_t status = 1;
        status &= ESP8266_SendAT("AT+CIPMODE=1\r\n", "OK", 3000);                         // ���䷽ʽ��0_��ͨ���䣬1_͸������
        status &= ESP8266_SendAT("AT+CIPSEND\r\n",    ">", 3000);                         // ���ݴ��䣬��������͸��ʱ���������
        status ? printf("��͸������:  �ɹ�\r\n") : printf("��͸������:  ʧ��\r\n");
        return status;
    }
    ESP8266_SendString("+++");
    printf("�ѹر�͸�����䣡\r\n");
    return 1;
}



/******************************************************************************
 * ��  ���� ESP8266_Function_JoinAP
 * ��  �ܣ� ����AP
 * ��  ���� char* SSID       WiFi����
 *          char* passWord   WiFi����
 * ��  �أ� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_Function_JoinAP(char *SSID, char *passWord)
{
    char strTemp[60];
    uint8_t linkStatus = 0;
    // ��ESP8266�������ó�SATģʽ
    ESP8266_SendAT("AT+RESTORE\r\n", "ready", 3000)  ? printf("�ָ���������:  �ɹ�\r\n") : printf("�ָ���������:  ʧ��\r\n");   // �ָ�ģ��ĳ�������
    ESP8266_SendAT("AT+CWMODE=1\r\n",   "OK", 3000)  ? printf("���� STAģʽ:  �ɹ�\r\n") : printf("���� STAģʽ:  ʧ��\r\n");   // ����ģʽ��1_STA, 2_AP, 3_STA+AP
    ESP8266_SendAT("AT+RST\r\n",     "ready", 3000)  ? printf("���� ESP8266:  �ɹ�\r\n") : printf("���� ESP8266:  ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    //ESP8266_SendAT("AT+CIPMUX=0\r\n", "OK", 3000)  ? printf("������ģʽ  :  �ɹ�\r\n") : printf("������ģʽ  :  ʧ��\r\n");   // �� �� ��: 0_�����ӣ�1_������
    //������ָ��WiFi�ȵ�
    printf("׼������Wifi:  %s, %s\r\n", SSID, passWord);
    sprintf(strTemp, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, passWord);
    printf("��ʼ����AP ... ");
    ESP8266_SendAT(strTemp,        "OK\r\n", 6000)  ? printf("�ɹ�\r\n") : printf("ʧ��\r\n");
    // �������״̬
    printf("��ѯ����״̬: ");
    linkStatus = ESP8266_Function_GetIPStatus();
    switch (linkStatus)
    {
        case 0:
            printf(" ʧ�ܣ�ԭ�򣺻�ȡʧ�ܣ�\r\n");
            break;
        case 2:
            printf(" �ɹ����ѻ��IP\r\n");
            return 1;
        case 3:
            printf(" ʧ�ܣ�ԭ�������ӣ���δ���IP��\r\n");
            break;
        case 4:
            printf(" ʧ�ܣ�ԭ��ʧȥ���ӣ�\r\n");
            break;
        case 5:
            printf(" ʧ�ܣ�ԭ��û������\r\n");
            break;
        default:
            break;
    }
    return 0;
}



/******************************************************************************
 * ��  ���� ESP8266_Function_CreationAP
 * ��  �ܣ� ��ģ�����ó�APģʽ, �������ȵ�����
 * ��  ���� char* SSID       WiFi����
 *          char* passWord   WiFi����
 * ��  �أ� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_Function_CreationAP(char *SSID, char *passWord)
{
    char strTemp[60];

    printf("׼������SSID��%s, %s\r\n", SSID, passWord);
    // ��ESP8266�������ó�APģʽ
    ESP8266_SendAT("AT+RESTORE\r\n", "ready", 1000)  ? printf("�ָ���������: �ɹ�\r\n") : printf("�ָ���������: ʧ��\r\n");   // �ָ�ģ��ĳ�������
    ESP8266_SendAT("AT+CWMODE=2\r\n",   "OK", 3000)  ? printf("����ΪAPģʽ: �ɹ�\r\n") : printf("���� STAģʽ: ʧ��\r\n");   // ����ģʽ��1_STA, 2_AP, 3_STA+AP
    ESP8266_SendAT("AT+RST\r\n",     "ready", 3000)  ? printf("���� ESP8266: �ɹ�\r\n") : printf("���� ESP8266: ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    // ����WiFi�ȵ�
    sprintf(strTemp, "AT+CWSAP=\"%s\",\"%s\",11,0\r\n", SSID, passWord);
    printf("��ʼ����AP... ");
    if (ESP8266_SendAT(strTemp, "OK\r\n", 10000))
    {
        printf("�ɹ�\r\n");
        return 1;
    }
    else
    {
        printf("ʧ��\r\n");
        return 0;
    }
}



/******************************************************************************
 * ��  ���� ESP8266_Function_GetIPStatus
 * ��  �ܣ� ��ȡ����״̬
 * ��  ���� ��
 * ��  ��:  0_��ȡ״̬ʧ��
 *          2_���ip
 *          3_��������
 *          4_ʧȥ����
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
 * ��  ���� ESP8266_Function_ConnectTCP
 * ��  �ܣ� ����TCP����
 * ��  ���� char *IP        IP��ַ
 *          uint16_t port   �˿�
 * ��  �أ� 0-ִ��ʧ�ܡ�1-ִ������
 ******************************************************************************/
uint8_t ESP8266_Function_ConnectTCP(char *IP, uint16_t port)
{
    char strTemp[100];
    uint8_t status = 0;

    printf("����TCPͨ��... ");
    sprintf(strTemp, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", IP, port);
    status = ESP8266_SendAT(strTemp, "OK\r\n", 5000);
    if (status)
        printf("�ɹ�\r\n");
    else
        printf("ʧ��, ����APP��IP��ַ���˿ںš��Ƿ��ѶϿ��ɵ�����״̬\r\n");
    return status;
}



/******************************************************************************
 * ��  ���� ESP8266_Function__GetIP
 * ��  �ܣ� ��ȡ������������IP��ַ
 * ��  ע�� ��õ����ݣ���8���ֽ�
 *          ����IP    [0]��[1]��[2]��[3]
 *          ������IP  [4]��[5]��[6]��[7]
 * ��  ���� ��
 * ��  ��:  uint8_t*�� 0_��ȡʧ�ܡ���0_���ݵ�ַ
 ******************************************************************************/
uint8_t* ESP8266_Function_GetIP(void)
{
    static uint8_t aData[8];                                     // ���ڱ����õ�����
    char strTem[100]  = {0};                                     // ���IP�ַ���
    char *strData = NULL;                                        // ���ݵ�ַ
    char *addr1 = NULL;                                          // Ŀ������ǰ���ַ���������ַ
    char *addr2 = NULL;                                          // Ŀ�����ݺ���ַ���������ַ

    /** ��ȡģ���ھ������е�IP��ַ���� **/
    if (0 == ESP8266_SendAT("AT+CIFSR\r\n",  "OK", 1000))
    {
        printf("��ȡ������IP:  ʧ��!\r\n");                      // ��ѯ����IP��ַ��MAC��ַ
        return 0;                                                
    }                                                            
    /** ��ȡ������IP���ַ��� **/                                 
    strData = (char *)ESP8266_GetRxData();                       // ȡ�����һ��ATָ��ͺ󷵻ص�����
    addr1 = strstr(strData, "+CIFSR:STAIP") + 14;                // ��ԭʼ�����У���ȡIP��ַ�ַ����Ŀ�ʼ��ַ; +14����ΪҪ��ȥ����ַ����ĳ���
    addr2 = strstr(strData, "+CIFSR:STAMAC");                    // ��ԭʼ�����У���ȡIP��ַ�ַ����Ľ�����ַ
    memcpy(strTem, addr1, addr2 - addr1 - 3);                    // ��ȡIP��ַ�ε��ַ���; -3����ΪIP������һ��˫���ź�\r\n
    aData[4] = atoi(strtok(strTem, "."));                        // ת����1���ֽڣ���IP��ַ�ĵ�1���ֶΣ�xx.--.--.--
    aData[5] = atoi(strtok(NULL, "."));                          // ת����2���ֽڣ���IP��ַ�ĵ�2���ֶ�: --.xx.--.--
    aData[6] = atoi(strtok(NULL, "."));                          // ת����3���ֽڣ���IP��ַ�ĵ�3���ֶ�: --.--.xx.--
    aData[7] = atoi(strtok(NULL, "."));                          // ת����4���ֽڣ���IP��ַ�ĵ�4���ֶ�: --.--.--.xx

    /** ���ӷ����� **/
    if (0 == ESP8266_SendAT("AT+CIPMUX=0\r\n",   "OK", 1000))    // �����ӣ�0_�رա�1_������; ��Щģʽ�����ڵ�����ģʽ�����У���͸�������
    {
        printf("���õ�·���ӣ� ʧ��!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSTART=\"TCP\",\"quan.suning.com\",80\r\n", "OK", 1000))  // ���ӷ�����,ָTCP��UDP������
    {
        printf("���ӵ�������:  ʧ��!\r\n");     
        return 0;
    }
    /** ��ȡ����IP���� **/
    if (0 == ESP8266_SendAT("AT+CIPMODE=1\r\n",  "OK", 1000))    // ͸�����䣺0_�رա�1_��
    {                                                        
        printf("��͸��ģʽ:  ʧ��!\r\n");                  
        return 0;                                            
    }                                                        
    if (0 == ESP8266_SendAT("AT+CIPSEND\r\n",    "OK", 1000))    // ��������
    {
        printf("׼����ȡ����:  ʧ��!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("GET http://quan.suning.com/getSysTime.do HTTP/1.1\r\nHost: quan.suning.com\r\n\r\n",   "x-request-ip", 3000))  // ��ȡ����
    {
        printf("��ȡ������ַ:  ʧ��!\r\n");     
        return 0;
    }
    /** ��ȡ����IP���ַ��� **/
    strData = (char *)ESP8266_GetRxData();                       // ȡ�����һ��ATָ��ͺ󷵻ص�����
    addr1 = strstr(strData, "x-request-ip");                     // ��ԭʼ�����У���ȡIP��ַǰ������
    addr2 = strstr(strData, "x-tt-trace-tag");                   // ��ԭʼ�����У���ȡIP��ַ�������
    memset(strTem, 0, sizeof(strTem));                           // ��0����ľ�����
    memcpy(strTem, addr1 + 14, addr2 - addr1 - 14);              // ��ȡIP��ַ�ε��ַ���
    /** ����IP�ַ��� **/                                         
    aData[0] = atoi(strtok(strTem, "."));                        // ת����1���ֽڣ���IP��ַ�ĵ�1���ֶΣ�xx.--.--.--
    aData[1] = atoi(strtok(NULL, "."));                          // ת����2���ֽڣ���IP��ַ�ĵ�2���ֶ�: --.xx.--.--
    aData[2] = atoi(strtok(NULL, "."));                          // ת����3���ֽڣ���IP��ַ�ĵ�3���ֶ�: --.--.xx.--
    aData[3] = atoi(strtok(NULL, "."));                          // ת����4���ֽڣ���IP��ַ�ĵ�4���ֶ�: --.--.--.xx
    /** �Ͽ��������� **/                                         
    ESP8266_SendString("+++");                                   // �˳�͸��
    delay_ms(50);                                                // ������ʱ����ΪҪ��"+++"ִ�����
    if (0 == ESP8266_SendAT("AT+CIPCLOSE\r\n",   "OK", 3000))    // �Ͽ�����������, ���Ͽ�TCP��UDP
    {                                                        
        printf("�Ͽ� TCP����:  ʧ��!\r\n");                  
        return 0;                                            
    }                                                        
                                                             
    return aData;                                                // ��ȡ�ɹ�������: uint8_t*�����ݵ�ַ
}



/******************************************************************************
 * ��  ���� ESP8266_Function_GetDate
 * ��  �ܣ� ��ȡ��ǰ����ʱ��
 * ��  ע�� ��õ����ݣ���6���ֽ�
 *          [0]��(����2000+[0]��)��[1]�¡�[2]�ա�[3]ʱ��[4]�֡�[5]��
 * ��  ���� ��             
 * ��  ��:  uint8_t*�� 0_��ȡʧ�ܡ���0_���ݵ�ַ
 ******************************************************************************/
uint8_t* ESP8266_Function_GetDate(void)
{    
    static uint8_t pDate[6] = {0};
    memset(pDate, 0, 6);
    
    /** ���ӷ����� **/     
    if (0 == ESP8266_SendAT("AT+CIPMUX=0\r\n",   "OK", 1000))     // �� �� �ӣ�0_�رա�1_������; ��Щģʽ�����ڵ�����ģʽ�����У���͸�������
    {
        printf("���õ�·���ӣ� ʧ��!\r\n");     
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSTART=\"TCP\",\"quan.suning.com\",80\r\n", "OK", 1000))  // ���ӷ�����,ָTCP��UDP������
    {
        printf("���ӵ�������:  ʧ��!\r\n");                   
        return 0;
    }
    
    /** ��ȡ���� **/
    if (0 == ESP8266_SendAT("AT+CIPMODE=1\r\n",  "OK", 1000))     // ͸�����䣺0_�رա�1_��
    {
        printf("��͸��ģʽ:  ʧ��!\r\n");    
        return 0;
    }
    if (0 == ESP8266_SendAT("AT+CIPSEND\r\n",    "OK", 1000))     // ��ʼ��������
    {
        printf("׼����ȡ����:  ʧ��!\r\n");    
        return 0;
    }          
    if (0 == ESP8266_SendAT("GET http://quan.suning.com/getSysTime.do HTTP/1.1\r\nHost: quan.suning.com\r\n\r\n",   "sysTime1", 5000))  // ����ָ�� 
    {
        printf("��ȡʱ������:  ʧ��!\r\n");                   
        return 0;                                            
    }         
    
    /** �ѻ�ȡ���ַ����������ֵ **/                         
    char strTime[5] = {0};                                        // ����һ���ַ����飬���ڴ��Ҫת��������
    char *pStrData = (char *)ESP8266_GetRxData();                 // ȡ�����һ��ATָ��ͺ󷵻ص�����
    // ��                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 13, 2);        // ��ԭ�����У�������ݵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�11���ֽ����4���ֽ�
    pDate[0] = atof(strTime);                                     // �ַ���ת��Ϊ��ֵ; atof������C���Ա�׼��ĺ�������Ҫ����ͷ�ļ���#include <stdlib.h>
    // ��                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 15, 2);        // ��ԭ�����У������·ݵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�15���ֽ����2���ֽ�
    pDate[1] = atof(strTime);                                     // ���ַ���ת��Ϊ��ֵ
    // ��                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 17, 2);        // ��ԭ�����У��������ڵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�17���ֽ����2���ֽ�
    pDate[2] = atof(strTime);                                     // ���ַ���ת��Ϊ��ֵ
    // ʱ                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 19, 2);        // ��ԭ�����У�����ʱ�ӵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�19���ֽ����2���ֽ�
    pDate[3] = atof(strTime);                                     // ���ַ���ת��Ϊ��ֵ
    // ��                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 21, 2);        // ��ԭ�����У����Ʒ��ӵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�21���ֽ����2���ֽ�
    pDate[4] = atof(strTime);                                     // ���ַ���ת��Ϊ��ֵ
    // ��                                                         
    memcpy(strTime, strstr(pStrData, "sysTime1") + 23, 2);        // ��ԭ�����У��������ӵ��ַ���: sysTime1������ʼ���ڴ��ַ����������ĵ�23���ֽ����2���ֽ�
    pDate[5] = atof(strTime);                                     // ���ַ���ת��Ϊ��ֵ

    /** �Ͽ����������� **/                                    
    ESP8266_SendString("+++");                                    // �˳�͸��
    delay_ms(30);                                                 // ������ʱ����"+++"ִ�����
    if (0 == ESP8266_SendAT("AT+CIPCLOSE\r\n",   "OK", 3000))     // �Ͽ�����������, ���Ͽ�TCP��UDP
    {                                                             
        printf("�Ͽ� TCP����:  ʧ��!\r\n");                       
        return 0;                                                 
    }                                                             
    return pDate;                                                 // ��ȡ�ɹ�������: uint8_t*�����ݵ�ַ
}

