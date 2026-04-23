#include "Task_Init.h"
#include "main.h"
#include "tim.h"
#include "PWMmotor.h"
#include "usart.h"

	
extern uint8_t g_recv_flag; 
extern uint8_t g_recv_buff[RXBUFF_LEN];
extern uint8_t g_recv_buff_deal[RXBUFF_LEN];

void Task_Init(){
	
	HAL_UART_Receive_DMA(&huart4, g_recv_buff, RXBUFF_LEN);

	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	
	// TIM2、TIM5 自由度均比TIM3要更大
	
	// 电机初始化
  motor520_Init(&htim2);// 底座旋转
	motor520_Init(&htim3);// 末端
	motor520_Init(&htim5);// 底座上面的竖直前后旋转
	

 vPortEnterCritical();
	
	xTaskCreate(ARM,
      	"ARM",
        400,
        NULL,
        3,
        &ARM_Handle); 
	
	xTaskCreate(ARM2,
      	"ARM2",
        400,
        NULL,
        3,
        &ARM2_Handle); 
				
//	xTaskCreate(ARM3,
//      	"ARM3",
//        400,
//        NULL,
//        3,
//        &ARM3_Handle); 
	
 vPortExitCritical();
	
}

