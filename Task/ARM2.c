#include "ARM2.h"
#include "Task_Init.h"
#include "app_motor_usart.h"
#include "bsp_motor_usart.h"
#include "PID_old.h"
#include "get_pose.h"

motor_PID Motor1_PID = {
	.pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 20.0f
	}
	
};
motor_PID Motor2_PID = {
	.pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 20.0f
	}
	
};
motor_PID Motor3_PID = {
	.pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 20.0f
	}
	
};

Param motor1;
Param motor2;
Param motor3;

extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern volatile uint8_t g_recv_flag;

TaskHandle_t ARM2_Handle;
void ARM2(void *pvParameters)
{
	
//	Contrl_Pwm(0,0,0,NULL);// 配置三路电机PWM
	
	send_upload_data(true, true, true);
	
	// 配置全部电机(广播)
	send_motor_type(MOTOR_520);// 配置电机类型
	vTaskDelay(100);
	send_pulse_phase(56);//配置减速比 查电机手册得出
	vTaskDelay(100);
	send_pulse_line(11);//配置磁环线 查电机手册得出
	vTaskDelay(100);
	send_wheel_diameter(67.00);//配置轮子直径,测量得出
	vTaskDelay(100);
	send_motor_deadzone(1900);//配置电机死区,实验得出
	vTaskDelay(100);
//	send_motor_PID(4, 0.01, 1);
//	vTaskDelay(100);
	
 TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{

		// 电机 1 (底座)
		PID_Control2(Encoder_Now[0], motor1.Exp_encoder, &Motor1_PID.pid);
		// 电机 2 `
		PID_Control2(Encoder_Now[1], motor2.Exp_encoder, &Motor2_PID.pid);
		// 电机 3 
		PID_Control2(Encoder_Now[2], motor3.Exp_encoder, &Motor3_PID.pid);
		
		Contrl_Speed(
		(int16_t)Motor1_PID.pid.pid_out, 
		(int16_t)Motor2_PID.pid.pid_out, 
		(int16_t)Motor3_PID.pid.pid_out, 
		 NULL);
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}


TaskHandle_t Analysis_Handle;
void Analysis(void *pvParameters)
{
 TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
		if(g_recv_flag == 1)
		{
			g_recv_flag = 0;
			Deal_data_real();
		}
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}


