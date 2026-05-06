#include "motors520.h"
#include "usart.h"

#include "main.h"
#include <string.h>

#define TX_BUF_SIZE 50

/* 当前正在发送的帧缓冲区 */
static uint8_t tx_buf[TX_BUF_SIZE];
static uint16_t tx_len = 0;
static uint16_t tx_index = 0;
static uint8_t tx_active = 0;          // 1 = 正在逐字节交互中

/* pending 帧（单帧缓存，可扩展） */
static uint8_t pending_buf[TX_BUF_SIZE];
static uint16_t pending_len = 0;
static uint8_t pending_flag = 0;       // 1 = 有 pending 帧

/* 接收单字节缓冲 */
static uint8_t rx_byte;                // 接收回调存入此处




/**
 * @brief 发送一帧数据（逐字节交互方式）
 * @param pData : 帧数据
 * @param Length: 长度（≤ TX_BUF_SIZE）
 */
void Send_Motor_Array(uint8_t *pData, uint16_t Length)
{
    if (Length > TX_BUF_SIZE)
        return;

    if (tx_active == 0) {
        // 空闲：直接启发送
        memcpy(tx_buf, pData, Length);
        tx_len = Length;
        tx_index = 0;
        tx_active = 1;

        // 发送第一个字节（中断发送，只发1字节）
        HAL_UART_Transmit_IT(&huart7, &tx_buf[0], 1);
    } else {
        // 正在发：挂起到 pending（只保留最新一帧，旧帧丢弃）
        memcpy(pending_buf, pData, Length);
        pending_len = Length;
        pending_flag = 1;
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        // 一个字节发送完毕，立刻启动接收对方应答
        HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
    }
}
// 电机反馈
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        // 1. 处理刚收到的应答字节（你的原有处理逻辑）
        Deal_Control_Rxtemp(rx_byte);

        // 2. 判断当前帧是否发完
        if (tx_active) {
            tx_index++;
            if (tx_index < tx_len) {
                // 还有字节，发送下一个
                HAL_UART_Transmit_IT(&huart7, &tx_buf[tx_index], 1);
            } else {
                // 当前帧发送完毕
                tx_active = 0;

                // 3. 检查是否有 pending 帧，立即启动
                if (pending_flag) {
                    memcpy(tx_buf, pending_buf, pending_len);
                    tx_len = pending_len;
                    tx_index = 0;
                    pending_flag = 0;
                    tx_active = 1;
                    HAL_UART_Transmit_IT(&huart7, &tx_buf[0], 1);
                }
            }
        }
        // 如果 tx_active == 0 且无 pending，则接收中断可停止（也可以保持常开，由别处管理）
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        // 有标志位错误时终止接收并重启，避免卡死
        HAL_UART_AbortReceive_IT(&huart7);
        HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
    }
}
