#include "motors520.h"
#include "usart.h"

#include "main.h"
#include <string.h>


#define TX_BUF_SIZE 200

/* 发送缓冲区 */
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_len = 0;
static volatile uint16_t tx_index = 0;
static volatile uint8_t tx_busy = 0;    // 1 = 正在逐字节发送

/* 接收单字节缓冲 */
static uint8_t rx_byte;

/**
 * @brief 电机串口初始化（仅开启接收中断）
 */
void Motor_Usart_Init(void)
{
    // 启动接收中断，此后接收一直常开
    HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
}

/**
 * @brief 启动发送下一个字节（内部调用）
 */
static void start_tx_byte(void)
{
    if (tx_index < tx_len) {
        HAL_UART_Transmit_IT(&huart7, &tx_buf[tx_index], 1);
    } else {
        tx_busy = 0;   // 发送完成
    }
}

/**
 * @brief 用户调用：发送一帧数据（中断逐字节方式）
 * @param pData: 数据缓冲区
 * @param Length: 长度
 */
void Send_Motor_Array(uint8_t *pData, uint16_t Length)
{
    if (Length > TX_BUF_SIZE) return;

    // 等待上一次发送完成（简单阻塞，不丢帧）
    while (tx_busy) {
        // 可加超时退出，此处简单等待
    }

    memcpy(tx_buf, pData, Length);
    tx_len = Length;
    tx_index = 0;
    tx_busy = 1;
    start_tx_byte();   // 发送第一个字节
}

/**
 * @brief 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        tx_index++;
        start_tx_byte();   // 发送下一个字节
    }
}

/**
 * @brief 接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        // 处理接收到的字节（协议解析，在 app 层实现）
        Deal_Control_Rxtemp(rx_byte);

        // 重新打开接收中断（保持常开）
        HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
    }
}

/**
 * @brief 串口错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        // 终止当前接收，清除错误标志
        HAL_UART_AbortReceive_IT(&huart7);
        // 重启接收
        HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
    }
}