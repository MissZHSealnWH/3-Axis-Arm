#include "ARM2.h"
#include "Task_Init.h"
#include "motor520.h"
#include "motors520.h"
#include "PID_old.h"
#include "micro_kdl.h"
#include "uplink_drv.h"

static Mikdl_Robot RobotArm;             // 机械臂结构体
static Mikdl_Vector3 q_curr;          // 当前关节角度
static Mikdl_Vector3 p_curr;          // 当前末端位置

// 梯形规划轨迹相关
static Mikdl_TrapProfile trap;
static Mikdl_Vector3 p_start;
static Mikdl_Vector3 dir_unit;
static float total_dist;
static uint32_t traj_start_tick;
static uint8_t traj_active = 0;

// PID
motor_PID Motor1_PID;
motor_PID Motor2_PID;
motor_PID Motor3_PID;

// 期望参数 
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

void RobotArm_Init(void)
{
    mikdl_robot_default(&RobotArm);
    RobotArm.l1 = 0.2f;
    RobotArm.l2 = 0.15f;
    RobotArm.m1 = 0.5f;
    RobotArm.m2 = 0.3f;
    RobotArm.c1 = 0.1f;
    RobotArm.c2 = 0.075f;

    // 初始关节角度 (0, 30, 30)
    q_curr.x = 0.0f;
    q_curr.y = 0.5236f;
    q_curr.z = 0.5236f;

    mikdl_forward_kinematics(&RobotArm, &q_curr, &p_curr);
}

UplinkCommand cmd;
TaskHandle_t Analysis_Handle;
void Analysis(void *pvParameters)
{
	RobotArm_Init();
	
 TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
		if (Uplink_GetCommand(&cmd)) 
		 {
			// 根据 cmd.x, cmd.y, cmd.z 计算末端目标位置
			Mikdl_Vector3 p_target = p_curr;
			p_target.x += cmd.x * STEP_SIZE;
			p_target.y += cmd.y * STEP_SIZE;
			p_target.z += cmd.z * STEP_SIZE;

			// 计算位移
			float dx = p_target.x - p_curr.x;
			float dy = p_target.y - p_curr.y;
			float dz = p_target.z - p_curr.z;
			float dist = sqrtf(dx*dx + dy*dy + dz*dz); // 三维直线距离

			if (dist > 1e-6f) 
			{
				p_start = p_curr;              // 记录起点
				total_dist = dist;             // 总距离
				dir_unit.x = dx / dist;        // 计算方向单位向量
				dir_unit.y = dy / dist;
				dir_unit.z = dz / dist;

				mikdl_trap_init(&trap, 0.0f, dist, MAX_VEL, MAX_ACC);
				traj_start_tick = xTaskGetTickCount();
				traj_active = 1;
			}
		}

			Mikdl_Vector3 p_ref, v_ref, a_ref;
			Mikdl_Vector3 q_out, dq_out, tau_out;
			Mikdl_Vector3 f_ext = {0, 0, 0};
			
			uint32_t now = xTaskGetTickCount();

		if (traj_active) 
		{
				float t_sec = (now - traj_start_tick) * 0.001f;
				float s, s_dot, s_ddot;
				mikdl_trap_get(&trap, t_sec, &s, &s_dot, &s_ddot);

				if (t_sec >= trap.duration) {
						s = total_dist;
						s_dot = 0.0f;
						s_ddot = 0.0f;
						traj_active = 0;
				}

				p_ref.x = p_start.x + dir_unit.x * s;
				p_ref.y = p_start.y + dir_unit.y * s;
				p_ref.z = p_start.z + dir_unit.z * s;
				v_ref.x = dir_unit.x * s_dot;
				v_ref.y = dir_unit.y * s_dot;
				v_ref.z = dir_unit.z * s_dot;
				a_ref.x = dir_unit.x * s_ddot;
				a_ref.y = dir_unit.y * s_ddot;
				a_ref.z = dir_unit.z * s_ddot;
		} 
			else 
			{
			// 没有轨迹就保持当前位置，速度/加速度为0
				p_ref = p_curr;
				v_ref.x = 0; v_ref.y = 0; v_ref.z = 0;
				a_ref.x = 0; a_ref.y = 0; a_ref.z = 0;
			}
			// 调用micro_kdl解算
			int ret = mikdl_cart_to_joint_control
					(&RobotArm,
					 &p_ref, 
					 &v_ref,
					 &a_ref,
					 &f_ext,
					 &q_curr,
					 &q_out, 
					 &dq_out,
					 &tau_out);
				
				if (ret == MIKDL_SUCCESS) 
				{
					q_curr = q_out;
					mikdl_forward_kinematics(&RobotArm, &q_curr, &p_curr);
					ALL_num++;
				} 
        else 
				{
					// 解算失败保持上一周期关节角度
					q_out = q_curr;
        }
        // 将 q_out 转换成电机期望位置
        motor1.Exp_encoder = (int32_t)(q_out.x * RAD2ENC_FACTOR_JOINT1 + 0.5f);
        motor2.Exp_encoder = (int32_t)(q_out.y * RAD2ENC_FACTOR_JOINT2 + 0.5f);
        motor3.Exp_encoder = (int32_t)(q_out.z * RAD2ENC_FACTOR_JOINT3 + 0.5f);

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
