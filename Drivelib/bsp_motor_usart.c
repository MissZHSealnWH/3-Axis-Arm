#include "bsp_motor_usart.h"
#include "usart.h"

volatile uint8_t uart4_tx_busy = 0;


/************************************************
函数名称 ： Send_Motor_Frame		Function name: Send_Motor_Frame
功    能 ： UART4发送一个字符	Function: UART4 sends a character
参    数 ： buf --- 数据		Parameter: buf --- data
返 回 值 ： 无					Return value: None
*************************************************/

void Send_Motor_Frame(uint8_t *buf, uint16_t len)
{
    while(uart4_tx_busy);

    uart4_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart4, buf, len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4)
    {
        uart4_tx_busy = 0;
    }
}
/************************************************
函数名称 ： Send_Motor_Array	Function name: Send_Motor_Array
功    能 ： 串口发送N个字符		Function: Serial port sends N characters
参    数 ： pData ---- 字符串	Parameter: pData ---- string
            Length --- 长度		Length --- length
返 回 值 ： 无					Return value: None
*************************************************/
void Send_Motor_Array(uint8_t *pData, uint16_t Length)
{
		Send_Motor_Frame(pData, Length);
}







