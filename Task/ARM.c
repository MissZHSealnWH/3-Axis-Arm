#include "ARM.h"
#include "PID_old.h"
#include "PWMmotor.h"

/*
TIM2 CH1 PA5
TIM2 CH2 PB3
TIM5 CH1 PA0
TIM5 CH2 PA1
TIM3 CH1 PA6
TIM3 CH2 PA7
*/

TIM_PID htim2_PID = {
	.pos_pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 40.0f
	},
	.vel_pid = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f
}
};// 底座旋转

TIM_PID htim5_PID = {
	.pos_pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 40.0f
	},
	.vel_pid = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f
}
};// 底座上面的竖直前后旋转

TIM_PID htim3_PID = {
	.pos_pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 40.0f
	},
	.vel_pid = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f
}
};// 末端
TIM_Exp_Prama htim2_Exp = {
	.Exp_pos = 0.0f,
	.Exp_vel = 0.0f
};
TIM_Exp_Prama htim5_Exp = {
	.Exp_pos = 0.0f,
	.Exp_vel = 0.0f
};
TIM_Exp_Prama htim3_Exp = {
	.Exp_pos = 0.0f,
	.Exp_vel = 0.0f
};

// 获取到的编码器值
	uint32_t Tim2_val;
	uint32_t Tim3_val;
	uint32_t Tim5_val;

uint8_t the_trigger = 0;

TaskHandle_t ARM_Handle;
void ARM(void *pvParameters)
{
 TickType_t Last_wake_time = xTaskGetTickCount();
	
	for(;;)
	{
		motor520_GET_Encoder(&htim2, &Tim2_val);
		motor520_GET_Encoder(&htim3, &Tim3_val);//65535
		motor520_GET_Encoder(&htim5, &Tim5_val);
		
		vTaskDelay(1);
		
		if(the_trigger ==0)
		{
	  	PID_Control2((float)Tim2_val, htim2_Exp.Exp_pos, &htim2_PID.pos_pid);
		  PID_Control2(htim2_PID.pos_pid.pid_out, htim2_Exp.Exp_vel, &htim2_PID.vel_pid);
			if(htim2_PID.vel_pid.pid_out < 0)
			{
				motor520_Backward(&htim2, htim2_PID.vel_pid.pid_out);
			}
			else
			{
				motor520_Forward(&htim2, htim2_PID.vel_pid.pid_out);
			}
			the_trigger = 1;
		}
		
		if(the_trigger == 1)
		{
		  PID_Control2((float)Tim5_val, htim5_Exp.Exp_pos, &htim5_PID.pos_pid);
		  PID_Control2(htim5_PID.pos_pid.pid_out, htim5_Exp.Exp_vel, &htim5_PID.vel_pid);
			if(htim5_PID.vel_pid.pid_out < 0)
			{
				motor520_Backward(&htim5, htim5_PID.vel_pid.pid_out);
			}
			else
			{
				motor520_Forward(&htim5, htim5_PID.vel_pid.pid_out);
			}
			the_trigger = 2;
		}
		
		if(the_trigger == 2)
		{
			PID_Control2((float)Tim3_val, htim3_Exp.Exp_pos, &htim3_PID.pos_pid);
		  PID_Control2(htim3_PID.pos_pid.pid_out, htim3_Exp.Exp_vel, &htim3_PID.vel_pid);
			if(htim3_PID.vel_pid.pid_out < 0)
			{
				motor520_Backward(&htim3, htim3_PID.vel_pid.pid_out);
			}
			else
			{
				motor520_Forward(&htim3, htim3_PID.vel_pid.pid_out);
			}
			the_trigger = 3;
		}

		if(the_trigger == 3)
		{
		}
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}
