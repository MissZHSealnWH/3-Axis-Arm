#include "ARM2.h"
#include "Task_Init.h"
#include "app_motor_usart.h"
#include "bsp_motor_usart.h"
#include "PID_old.h"
#include "get_pose.h"

motor_PID Motor1_PID;
motor_PID Motor2_PID;
motor_PID Motor3_PID;

Param motor1;
Param motor2;
Param motor3;

extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern uint8_t g_recv_flag;

TaskHandle_t ARM2_Handle;
void ARM2(void *pvParameters)
{
 TickType_t Last_wake_time = xTaskGetTickCount();
	
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
	
	for(;;)
	{
		// 电机 1 (底座)
		PID_Control2(Encoder_Offset[0], motor1.Exp_encoder, &Motor1_PID.pid);
		// 电机 2 `
		PID_Control2(Encoder_Offset[1], motor2.Exp_encoder, &Motor2_PID.pid);
		// 电机 3 
		PID_Control2(Encoder_Offset[2], motor3.Exp_encoder, &Motor3_PID.pid);
		
		Contrl_Speed((int16_t)Motor1_PID.pid.pid_out, (int16_t)Motor2_PID.pid.pid_out, (int16_t)Motor3_PID.pid.pid_out, NULL);
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}
