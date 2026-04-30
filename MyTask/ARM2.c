#include "ARM2.h"
#include "Task_Init.h"
#include "motor520.h"
#include "motors520.h"
#include "PID_old.h"
#include "get_pose.h"
#include "micro_kdl.h"
#include "uplink_drv.h"

static Mikdl_Robot robot;             // 机械臂结构体
static Mikdl_Vector3 q_curr;          // 当前关节角度
static Mikdl_Vector3 p_curr;          // 当前末端位置

// 梯形规划轨迹相关
static Mikdl_TrapProfile trap;
static Mikdl_Vector3 p_start;
static Mikdl_Vector3 dir_unit;
static float total_dist;
static uint32_t traj_start_tick;
static uint8_t traj_active = 0;



motor_PID Motor1_PID;
motor_PID Motor2_PID;
motor_PID Motor3_PID;

Param motor1;
Param motor2;
Param motor3;

static int ALL_num = 0; // 通信计数

// 电机数据
extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern volatile uint8_t g_recv_flag;
/**/

Mikdl_Robot RoboArm = {

	.l1 = 0.0f,
	.m1 = 0.0f,
	.c1 = 0.0f,
	.l2 = 0.0f,
	.m2 = 0.0f,
	.c2 = 0.0f,
	.gravity = 9.8f
};



TaskHandle_t ARM2_Handle;
void ARM2(void *pvParameters)
{
	send_upload_data(true, true, true);
	
	// 配置全部电机(广播)
	send_motor_type(MOTOR_520);      // 配置电机类型
	vTaskDelay(100);
	send_pulse_phase(56);            //配置减速比 查电机手册得出
	vTaskDelay(100);
	send_pulse_line(11);             //配置磁环线 查电机手册得出
	vTaskDelay(100);
	send_wheel_diameter(67.00);      //配置轮子直径,测量得出
	vTaskDelay(100);
	send_motor_deadzone(1900);       //配置电机死区,实验得出
	vTaskDelay(100);
	send_motor_PID(4, 0.01, 1);
	vTaskDelay(100);
	
	mikdl_robot_default(&RoboArm);
	
 TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
		if(g_recv_flag == 1)
		{
			g_recv_flag = 0;
			Deal_data_real();
		}
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
		UplinkCommand cmd;
		if (Uplink_GetCommand(&cmd)) {
				// cmd.x, cmd.y, cmd.z 包含最新的目标坐标
				// 在这里调用机械臂
			
		}
    
		
		
		
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}
