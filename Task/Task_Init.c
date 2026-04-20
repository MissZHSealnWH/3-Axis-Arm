#include "Task_Init.h"
#include "main.h"
#include "tim.h"
#include "PWMmotor.h"
	
void Task_Init(){
	
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
	
 vPortExitCritical();
	
}

