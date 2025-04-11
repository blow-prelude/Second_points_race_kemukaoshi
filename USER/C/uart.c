/**
  ******************************************************************************
  * @file    usart2.c
  * @author  amkl
  * @version V1.0
  * @date    2022-09-22
  * @brief   USART2 HAL��ʵ��
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "string.h"
#include "stm32f4xx_hal.h"  // �������оƬ�ͺŵ���

extern UART_HandleTypeDef huart2;  // �����ⲿ����� huart2

/* ȫ�ֱ��� ------------------------------------------------------------------*/
uint8_t USART2_RX_BUF[USART2_REC_LEN] = {0}; // ���ջ�����
uint16_t USART2_RX_STA = 0;                  // ����״̬���
volatile uint8_t USART2_REC_CNT_LEN = 0;     // �������ݼ���

/* CubeMX����˵�� ------------------------------------------------------------*/
/**
  * USART3���ò��裺
  * 1. ��Connectivity��ѡ��USART3
  * 2. ģʽѡ��Ϊ"Asynchronous"
  * 3. �������ã�
  *    - Baud Rate: 115200 (���������޸�)
  *    - Word Length: 8 Bits
  *    - Stop Bits: 1
  *    - Parity: None
  *    - Hardware Flow Control: None
  * 4. GPIO���ã�
  *    - PB10: USART3_TX �� Alternate Function Push-Pull
  *    - PB11: USART3_RX �� Input Floating
  * 5. NVIC���ã�
  *    - ����USART3ȫ���ж�
  */

/* ����ʵ�� ------------------------------------------------------------------*/

/**
  * @brief USART3��ʼ��
  * @param baudrate ������
  */
void Usart2_Init(uint32_t baudrate)
{
    // CubeMX�����ɳ�ʼ�����룬ͨ����main.c��MX_USART3_UART_Init()��
    // �˴�ֻ�����������ж�
    HAL_UART_Receive_IT(&huart2, &USART2_RX_BUF[0], 1);
}

/**
  * @brief ���͵��ֽ�
  * @param Data Ҫ���͵�����
  */
void Usart2_SendByte(uint8_t Data)
{
    HAL_UART_Transmit(&huart2, &Data, 1, HAL_MAX_DELAY);
}

/**
  * @brief �����ַ���
  * @param str Ҫ���͵��ַ���
  */
void Usart3_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief USART3�жϻص�����
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // ���ݴ��뻺����
        USART2_REC_CNT_LEN++;
        
        // ��黺�������
        if(USART2_REC_CNT_LEN >= USART2_REC_LEN)
        {
            USART2_REC_CNT_LEN = 0;
        }
        
        // �������������ж�
        HAL_UART_Receive_IT(&huart2, &USART2_RX_BUF[USART2_REC_CNT_LEN], 1);
    }
}

/**
  * @brief OpenMV���ݴ���
  */
void openMv_Proc(void)
{
    if(strstr((const char*)USART2_RX_BUF, "end") != NULL)
    {
        if(strcmp((const char*)USART2_RX_BUF, "stopend") == 0)
        {
            Param.openMV_Data = 1;
        }
        
        USART2_REC_CNT_LEN = 0;
        memset(USART2_RX_BUF, 0, USART2_REC_LEN);
        
        // ���¿�ʼ����
        HAL_UART_Receive_IT(&huart2, &USART2_RX_BUF[0], 1);
    }
}

/*************************************END OF FILE************************************/
