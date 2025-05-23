/**
  ******************************************************************************
  * @file    usart2.c
  * @author  amkl
  * @version V1.0
  * @date    2022-09-22
  * @brief   USART2 HAL库实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "string.h"
#include "stm32f4xx_hal.h"  // 根据你的芯片型号调整

extern UART_HandleTypeDef huart2;  // 声明外部定义的 huart2

/* 全局变量 ------------------------------------------------------------------*/
uint8_t USART2_RX_BUF[USART2_RX_LEN] = {0}; // 接收缓冲区
uint8_t USART2_RX_STA = 0;                  // 接收状态标记
uint8_t USART2_newData;
uint8_t start_receive=0;
//volatile uint8_t USART2_REC_CNT_LEN = 0;     // 接收数据计数


/* CubeMX配置说明 ------------------------------------------------------------*/
/**
  * USART3配置步骤：
  * 1. 在Connectivity中选择USART3
  * 2. 模式选择为"Asynchronous"
  * 3. 参数配置：
  *    - Baud Rate: 115200 (根据需求修改)
  *    - Word Length: 8 Bits
  *    - Stop Bits: 1
  *    - Parity: None
  *    - Hardware Flow Control: None
  * 4. GPIO配置：
  *    - PB10: USART3_TX → Alternate Function Push-Pull
  *    - PB11: USART3_RX → Input Floating
  * 5. NVIC设置：
  *    - 启用USART3全局中断
  */

/* 函数实现 ------------------------------------------------------------------*/

/**
  * @brief USART3初始化
  * @param baudrate 波特率
  */
void Usart2_Init(uint32_t baudrate)
{
    // CubeMX已生成初始化代码，通常在main.c的MX_USART3_UART_Init()中
    // 此处只需启动接收中断
    HAL_UART_Receive_IT(&huart2, &USART2_RX_BUF[0], 1);
}

/**
  * @brief 发送单字节
  * @param Data 要发送的数据
  */
void Usart2_SendByte(uint8_t Data)
{
    HAL_UART_Transmit(&huart2, &Data, 1, HAL_MAX_DELAY);
}

/**
  * @brief 发送字符串
  * @param str 要发送的字符串
  */
void Usart2_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief USART3中断回调函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
				start_receive=1;
        USART2_RX_BUF[USART2_RX_STA&0x7FFF]=USART2_newData;
				USART2_RX_STA++;
        
        // 检查缓冲区溢出
        if(USART2_RX_STA >= USART2_RX_LEN)
        {
            USART2_RX_STA = 0;
        }
        
        // 重新启动接收中断
        HAL_UART_Receive_IT(&huart2, &USART2_newData, 1);
    }
}

/**
  * @brief OpenMV数据处理
  */
/*
void openmv_proc(void)
{
    if(strstr((const char*)usart2_rx_buf, "end") != null)
    {
        if(strcmp((const char*)usart2_rx_buf, "stopend") == 0)
        {
            param.openmv_data = 1;
        }
        
        usart2_rec_cnt_len = 0;
        memset(usart2_rx_buf, 0, usart2_rec_len);
        
        // 重新开始接收
        hal_uart_receive_it(&huart2, &usart2_rx_buf[0], 1);
    }
}
*/



/*************************************END OF FILE************************************/
