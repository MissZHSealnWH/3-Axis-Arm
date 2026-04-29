#include "uplink_drv.h"
#include "stm32h7xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

static uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
static UplinkCommand latest_cmd;
static volatile uint8_t new_cmd_flag = 0;

void Uplink_Init(void) {
    /* 启动 DMA + IDLE 接收，函数内部会自动开启 IDLE 中断 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dma_rx_buf, UPLINK_FRAME_LEN);
    /* 关闭半传输中断（避免数据未接收完就触发） */
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

/* HAL 库在 IDLE 或 DMA 满中断后会自动调用此回调 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5 && Size == UPLINK_FRAME_LEN) {
        if (dma_rx_buf[0] == 0xAA && dma_rx_buf[1] == 0x55) {
            /* 解析浮点数（小端序） */
            memcpy(&latest_cmd.x, &dma_rx_buf[2], 4);
            memcpy(&latest_cmd.y, &dma_rx_buf[6], 4);
            memcpy(&latest_cmd.z, &dma_rx_buf[10], 4);
            new_cmd_flag = 1;
        }
    }
    /* 重新启动接收（必须，否则中断只触发一次） */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dma_rx_buf, UPLINK_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

int Uplink_GetCommand(UplinkCommand *cmd) {
    if (new_cmd_flag) {
        new_cmd_flag = 0;
        *cmd = latest_cmd;
        return 1;
    }
    return 0;
}