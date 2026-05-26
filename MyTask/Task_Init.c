#include "Task_Init.h"	
#include "main.h"
#include "usart.h"
#include "uplink_drv.h"

	
extern volatile uint8_t g_recv_flag; 
extern uint8_t g_recv_buff[RXBUFF_LEN];
extern uint8_t g_recv_buff_deal[RXBUFF_LEN];

extern uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
extern UplinkCommand latest_cmd;
extern volatile uint8_t new_cmd_flag;
 
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void Task_Init(){
	
	// 与上位机视觉通信
	Uplink_Init();

 vPortEnterCritical();
	
	xTaskCreate(ARM2,
      	"ARM2",
        768,
        NULL,
        3,
        &ARM2_Handle); 
				
	xTaskCreate(Analysis,
      	"Analysis",
        1024,
        NULL,
        3,
        &Analysis_Handle); 
				
	xTaskCreate(ZDT_Parse_Run,
      	"ZDT_Parse_Run",
        512,
        NULL,
        3,
        &ZDT_Parse_Run_Handle); 
	
 vPortExitCritical();
	
}


void Uplink_Init(void) 
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, UPLINK_FRAME_LEN);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

