#include "motors520.h"
#include "usart.h"


//volatile uint8_t uart4_tx_complete = 1;  // 初始就绪


// 双缓冲区
 uint8_t send_buff1[50];
 uint8_t send_buff2[50];
static uint8_t *pending_buf = NULL;   // 指向等待发送的缓冲区（有数据但尚未启动DMA）
static uint16_t pending_len = 0;

volatile uint8_t send_busy = 0;       // 1 = DMA正在传输

void Send_Motor_Frame(uint8_t *buf, uint16_t len)
{
    if (send_busy == 0) {
        // 空闲 -> 立即发送
        send_busy = 1;
        HAL_UART_Transmit_DMA(&huart7, buf, len);
    } else {
        // 忙时：覆盖 pending 指针，只保留最新一包数据
        pending_buf = buf;
        pending_len = len;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        send_busy = 0;
        if (pending_buf != NULL) {
            // 发送排队的数据
            send_busy = 1;
            HAL_UART_Transmit_DMA(&huart7, pending_buf, pending_len);
            pending_buf = NULL;
            pending_len = 0;
        }
    }
}

void Send_Motor_Array(uint8_t *pData, uint16_t Length)
{
		Send_Motor_Frame(pData, Length);
}







