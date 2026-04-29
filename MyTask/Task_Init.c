#include "Task_Init.h"
#include "main.h"
#include "usart.h"

	
extern volatile uint8_t g_recv_flag; 
extern uint8_t g_recv_buff[RXBUFF_LEN];
extern uint8_t g_recv_buff_deal[RXBUFF_LEN];
uint8_t uart7_rx_byte;

void Task_Init(){
	
	HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);

 vPortEnterCritical();

	
	xTaskCreate(ARM2,
      	"ARM2",
        768,
        NULL,
        3,
        &ARM2_Handle); 
				
	xTaskCreate(Analysis,
      	"Analysis",
        400,
        NULL,
        3,
        &Analysis_Handle); 
	
 vPortExitCritical();
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART7)
    {
        Deal_Control_Rxtemp(uart7_rx_byte);

        // 重新打开接收（必须！）
         HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART7) {
        HAL_UART_Receive_IT(&huart7, &uart7_rx_byte, 1);  // 重启动接收
    }
}
