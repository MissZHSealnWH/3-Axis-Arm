#include "ARM2.h"
#include "Task_Init.h"
#include "motor520.h"
#include "motors520.h"
#include "PID_old.h"
#include "micro_kdl.h"
#include "uplink_drv.h"

static Mikdl_Robot RobotArm;          // 机械臂结构体
static Mikdl_Vector3 q_curr;          // 当前关节角度
static Mikdl_Vector3 p_curr;          // 当前末端位置
static Mikdl_Vector3 q_home;          // 上电时机械臂姿态对应的关节角
static int manual_test_done = 0;

// 梯形规划轨迹相关
static Mikdl_TrapProfile trap;
static Mikdl_Vector3 p_start;
static Mikdl_Vector3 dir_unit;
static float total_dist;
static uint32_t traj_start_tick;
static uint8_t traj_active = 0;

// PID
motor_PID Motor1_PID = {
	.pid = {
		.Kp = 0.0f,
		.Ki = 0.0f,
		.Kd = 0.0f,
		.limit = 10000.0f,
		.output_limit = 3000.0f
	}
};
motor_PID Motor2_PID = {
		.pid = {
		  .Kp = 0.0f,
	  	.Ki = 0.0f,
	  	.Kd = 0.0f,
  		.limit = 10000.0f,
	  	.output_limit = 3000.0f
	}
};
motor_PID Motor3_PID = {
		.pid = {
			.Kp = 0.0f,
	  	.Ki = 0.0f,
	  	.Kd = 0.0f,
	  	.limit = 10000.0f,
	  	.output_limit = 3000.0f
	}
};

// 期望参数 
Param motor1;
Param motor2;
Param motor3;

TEXT_Param M1;
TEXT_Param M2;
TEXT_Param M3;

int16_t vel_1;
int16_t vel_2;
int16_t vel_3;

 int ALL_num = 0; // 总通信计数

extern SemaphoreHandle_t g_EncoderMutex; 

// 电机数据
extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern volatile uint8_t g_recv_flag;


TaskHandle_t ARM2_Handle;
void ARM2(void *pvParameters)
{
	send_upload_data(true, true, true);
	
	// 配置全部电机(广播)
	send_motor_type(MOTOR_520);      // 配置电机类型
	vTaskDelay(100);
	send_pulse_phase(56);            //配置减速比
	vTaskDelay(100);
	send_pulse_line(11);             //配置磁环线
	vTaskDelay(100);
	send_wheel_diameter(67.00);      //配置轮子直径
	vTaskDelay(100);
	send_motor_deadzone(1900);       //配置电机死区
	vTaskDelay(100);
	send_motor_PID(0.7f, 0.0f, 0.1f);
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
    // 初始关节角度 (0, 90, 90)
    q_home.x = 0.0f;      // 底座0度
    q_home.y = 1.5708f;   // 大臂垂直向上
    q_home.z = -1.5708f;  // 小臂向前水平
	
	  // 此时编码器刚上电，值为0，所以实际关节角 = q_home
    q_curr = q_home;
	
	  mikdl_robot_default(&RobotArm);
    RobotArm.l1 = 0.15f;
    RobotArm.l2 = 0.1f;
    RobotArm.m1 = 0.5f;
    RobotArm.m2 = 0.3f;
    RobotArm.c1 = 0.1f;
    RobotArm.c2 = 0.075f;

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
//		if (!manual_test_done) {
//    cmd.x = 1;
//    cmd.y = 0;
//    cmd.z = 0;
//    manual_test_done = 1;

//    // 手动启动轨迹
//    Mikdl_Vector3 p_target = p_curr;
//    p_target.x += cmd.x * STEP_SIZE;
//    float dx = p_target.x - p_curr.x;
//    float dy = p_target.y - p_curr.y;
//    float dz = p_target.z - p_curr.z;
//    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
//    if (dist > 1e-6f) {
//        p_start = p_curr;
//        total_dist = dist;
//        dir_unit.x = dx / dist;
//        dir_unit.y = dy / dist;
//        dir_unit.z = dz / dist;
//        mikdl_trap_init(&trap, 0.0f, dist, MAX_VEL, MAX_ACC);
//        traj_start_tick = xTaskGetTickCount();
//        traj_active = 1;
//    }
//}
		  // 更新真实关节角和末端位置（基于编码器反馈）
			q_curr.x = q_home.x + (float)Encoder_Now[0] / RAD2ENC_FACTOR_JOINT0;
			q_curr.y = q_home.y + (float)Encoder_Now[1] / RAD2ENC_FACTOR_JOINT;
			q_curr.z = q_home.z + (float)Encoder_Now[2] / RAD2ENC_FACTOR_JOINT;
		
			mikdl_forward_kinematics(&RobotArm, &q_curr, &p_curr);
		
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

				if (t_sec >= trap.duration) 
				{
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
//					  if (q_out.y >= JOINT1_MIN && q_out.y <= JOINT1_MAX)
						if (q_out.y >= JOINT1_MIN && q_out.y <= JOINT1_MAX && q_out.z >= JOINT2_MIN && q_out.z <= JOINT2_MAX)
						{
							motor1.Exp_encoder = (int32_t)((q_out.x - q_home.x) * RAD2ENC_FACTOR_JOINT0 + 0.5f);
							motor2.Exp_encoder = (int32_t)((q_out.y - q_home.y) * RAD2ENC_FACTOR_JOINT + 0.5f);
							motor3.Exp_encoder = (int32_t)((q_out.z - q_home.z) * RAD2ENC_FACTOR_JOINT + 0.5f);
							ALL_num++;
				  	} 
				   	else
						{
							motor1.Exp_encoder = Encoder_Now[0];
							motor2.Exp_encoder = Encoder_Now[1];
							motor3.Exp_encoder = Encoder_Now[2];
							traj_active = 0;
				  	}
					}
					else 
					{
						motor1.Exp_encoder = Encoder_Now[0];
						motor2.Exp_encoder = Encoder_Now[1];
						motor3.Exp_encoder = Encoder_Now[2];
					  traj_active = 0;
					}

				// 电机 1 (底座)
 				PID_Control2(Encoder_Now[0], motor1.Exp_encoder, &Motor1_PID.pid);
				// 电机 2 `
				PID_Control2(Encoder_Now[1], motor2.Exp_encoder, &Motor2_PID.pid);
				// 电机 3 
				PID_Control2(Encoder_Now[2], motor3.Exp_encoder, &Motor3_PID.pid);
				
				vel_1 = -(int16_t)Motor1_PID.pid.pid_out;
				vel_2 = -(int16_t)Motor2_PID.pid.pid_out;
				vel_3 = -(int16_t)Motor3_PID.pid.pid_out;
				
// 备用速度前馈
//				float feedforward_1 = dq_out.x * RAD2ENC_FACTOR_JOINT0;
//				float feedforward_2 = dq_out.y * RAD2ENC_FACTOR_JOINT;
//				float feedforward_3 = dq_out.z * RAD2ENC_FACTOR_JOINT;

//				int16_t speed_1 = (int16_t)(-(Motor1_PID.pid.pid_out + feedforward_1));
//				int16_t speed_2 = (int16_t)(-(Motor2_PID.pid.pid_out + feedforward_2));
//				int16_t speed_3 = (int16_t)(-(Motor3_PID.pid.pid_out + feedforward_3));


				Contrl_Speed(vel_1, vel_2, vel_3, NULL);
//				Contrl_Speed(speed_1, speed_2, speed_3, NULL);
				
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}
