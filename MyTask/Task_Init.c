#include "Task_Init.h"
#include "main.h"
#include "usart.h"
#include "uplink_drv.h"

	
extern volatile uint8_t g_recv_flag; 
extern uint8_t g_recv_buff[RXBUFF_LEN];
extern uint8_t g_recv_buff_deal[RXBUFF_LEN];

void Task_Init(){


  // 与电机驱动模块通信
	Motor_Usart_Init();
	
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
	
 vPortExitCritical();
	
}



