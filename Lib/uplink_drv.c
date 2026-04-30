#include "uplink_drv.h"
#include "stm32h7xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

static uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
static UplinkCommand latest_cmd;
static volatile uint8_t new_cmd_flag = 0;

void Uplink_Init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dma_rx_buf, UPLINK_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5 && Size == UPLINK_FRAME_LEN) {
        if (dma_rx_buf[0] == FRAME_START && dma_rx_buf[4] == FRAME_LAST) {
            latest_cmd.x = (int8_t)dma_rx_buf[1];
            latest_cmd.y = (int8_t)dma_rx_buf[2];
            latest_cmd.z = (int8_t)dma_rx_buf[3];
            new_cmd_flag = 1;
        }
    }
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