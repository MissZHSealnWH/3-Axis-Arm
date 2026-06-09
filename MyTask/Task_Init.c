#include "Task_Init.h"	
#include "main.h"
#include "usart.h"
#include "uplink_drv.h"
	

extern uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
extern UplinkCommand latest_cmd;
extern volatile uint8_t new_cmd_flag;
 
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

void Uplink_Init(void) 
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buf, UPLINK_FRAME_LEN);
	__HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
}

void Task_Init(){
	
	
	vTaskDelay(500);


	
	xTaskCreate(ARM2,
      	"ARM2",
        3072,
        NULL,
        3,
        &ARM2_Handle); 
				
 vTaskDelay(500);
				
	xTaskCreate(ZDT_Parse_Run,
      	"ZDT_Parse_Run",
        512,
        NULL,
        3,
        &ZDT_Parse_Run_Handle); 

	
}

