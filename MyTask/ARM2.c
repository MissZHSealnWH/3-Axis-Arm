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
PID2 Motor1_PID = {
	.Kp = 2.2f,
	.Ki = 0.0f,
	.Kd = 4.0f,
	.limit = 10000.0f,
	.output_limit = 3000.0f
};
PID2 Motor2_PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 3000.0f
};
PID2 Motor3_PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 3000.0f
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
	send_pulse_phase(56);            // 配置减速比
	vTaskDelay(100);
	send_pulse_line(11);             // 配置磁环线
	vTaskDelay(100);
	send_wheel_diameter(67.00);      // 配置轮子直径
	vTaskDelay(100);
	send_motor_deadzone(1900);       // 配置电机死区
	vTaskDelay(100);
	send_motor_PID(1.0f, 0.0f, 0.0f);
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
    // 初始关节角度 (0, 45, 90)
    q_home.x = 0.0f;      // 底座无要求
    q_home.y = 0.7854f;   // 大臂45°
    q_home.z = 1.5708f;   // 小臂垂直向上
	
	  // 此时编码器刚上电，值为0，所以实际关节角 = q_home
    q_curr = q_home;
	
	  mikdl_robot_default(&RobotArm);
    RobotArm.l1 = 0.15f;
    RobotArm.l2 = 0.1f;
    RobotArm.m1 = 0.18f;
    RobotArm.m2 = 0.163f;
    RobotArm.c1 = 0.04f;
    RobotArm.c2 = 0.02f;
	  RobotArm.gravity_dir.x =  0.0f;
	  RobotArm.gravity_dir.y =  0.0f;
	  RobotArm.gravity_dir.z = -1.0f;
	  RobotArm.gravity = 9.81f;

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
// 这段代码的存在仅用于验证cmd赋值后逆解的可行性 (注意因为debug使用编码器反馈一直为0所以达到目标值后直接归0)
//		if (!manual_test_done) 
//		{
//			cmd.x = 11;
//			cmd.y = 13;
//			cmd.z = 16;
//			manual_test_done = 1;

//			// 手动启动轨迹
//			Mikdl_Vector3 p_target = p_curr;
//			p_target.x += cmd.x * STEP_SIZE;
//			p_target.y += cmd.y * STEP_SIZE;
//			p_target.z += cmd.z * STEP_SIZE;
//			float dx = p_target.x - p_curr.x;
//			float dy = p_target.y - p_curr.y;
//			float dz = p_target.z - p_curr.z;
//			float dist = sqrtf(dx*dx + dy*dy + dz*dz);
//			if (dist > 1e-6f) 
//			{
//				p_start = p_curr;
//				total_dist = dist;
//				dir_unit.x = dx / dist;
//				dir_unit.y = dy / dist;
//				dir_unit.z = dz / dist;
//				mikdl_trap_init(&trap, 0.0f, dist, MAX_VEL, MAX_ACC);
//				traj_start_tick = xTaskGetTickCount();
//				traj_active = 1;
//			}
//		}

		  // 更新真实关节角和末端位置（基于编码器反馈）
			q_curr.x = q_home.x + (float)((float)Encoder_Now[0] / RAD2ENC_FACTOR_JOINT0);
			q_curr.y = q_home.y + (float)((float)Encoder_Now[1] / RAD2ENC_FACTOR_JOINT);
			q_curr.z = q_home.z + (float)((float)Encoder_Now[2] / RAD2ENC_FACTOR_JOINT);
		
			mikdl_forward_kinematics(&RobotArm, &q_curr, &p_curr);
			
// 此方案是以底座为坐标原点
//		if (Uplink_GetCommand(&cmd))
//		 {
//			// 根据 cmd.x, cmd.y, cmd.z 计算末端目标位置
//			Mikdl_Vector3 p_target = p_curr;
//			p_target.x += cmd.x * STEP_SIZE;
//			p_target.y += cmd.y * STEP_SIZE;
//			p_target.z += cmd.z * STEP_SIZE;

//			// 计算位移
//			float dx = p_target.x - p_curr.x;
//			float dy = p_target.y - p_curr.y;
//			float dz = p_target.z - p_curr.z;
//			float dist = sqrtf(dx*dx + dy*dy + dz*dz); // 三维直线距离

//			if (dist > 1e-6f) // 10的负六次方
//			{
//				p_start = p_curr;              // 记录起点
//				total_dist = dist;             // 总距离
//				dir_unit.x = dx / dist;        // 计算方向单位向量
//				dir_unit.y = dy / dist;        
//				dir_unit.z = dz / dist;        
//				
//				mikdl_trap_init(&trap, 0.0f, dist, MAX_VEL, MAX_ACC);
//				traj_start_tick = xTaskGetTickCount();
//				traj_active = 1;
//			}
//	   }
		if (Uplink_GetCommand(&cmd))
		{
				// 1. 计算当前末端姿态
				float phi   = q_curr.x;                 // 底座旋转角
				float theta = q_curr.y + q_curr.z;      // 大臂+小臂总俯仰角

				float c_phi = cosf(phi);
				float s_phi = sinf(phi);
				float c_theta = cosf(theta);
				float s_theta = sinf(theta);

				// 2. 定义末端坐标系的三个轴（在世界坐标系中的表示）
				// 这里假设：
				//   Z轴 — 沿小臂向外（相机前方）
				//   Y轴 — 垂直于运动平面（水平向左/右）
				//   X轴 — 由右手定则确定（Y × Z）
				Mikdl_Vector3 e_z;
				e_z.x = c_theta * c_phi;
				e_z.y = c_theta * s_phi;
				e_z.z = s_theta;

				Mikdl_Vector3 e_y;
				e_y.x = -s_phi;
				e_y.y =  c_phi;
				e_y.z = 0.0f;

				Mikdl_Vector3 e_x;
				e_x.x = e_y.y * e_z.z - e_y.z * e_z.y;
				e_x.y = e_y.z * e_z.x - e_y.x * e_z.z;
				e_x.z = e_y.x * e_z.y - e_y.y * e_z.x;

				// 3. 将 cmd (末端坐标系) 变换到世界坐标系
				float world_dx = cmd.x * e_x.x + cmd.y * e_y.x + cmd.z * e_z.x;
				float world_dy = cmd.x * e_x.y + cmd.y * e_y.y + cmd.z * e_z.y;
				float world_dz = cmd.x * e_x.z + cmd.y * e_y.z + cmd.z * e_z.z;

				// 4. 计算目标位置（世界坐标）
				Mikdl_Vector3 p_target = p_curr;
				p_target.x += world_dx * STEP_SIZE;
				p_target.y += world_dy * STEP_SIZE;
				p_target.z += world_dz * STEP_SIZE;

				// 5. 后续距离计算与轨迹启动（保持不变）
				float dx = p_target.x - p_curr.x;
				float dy = p_target.y - p_curr.y;
				float dz = p_target.z - p_curr.z;
				float dist = sqrtf(dx*dx + dy*dy + dz*dz);

				if (dist > 1e-6f) 
				{
						p_start = p_curr;
						total_dist = dist;
						dir_unit.x = dx / dist;
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
			
// 进行debug时保持注释状态
//					if (ret == MIKDL_SUCCESS) 
//					{
////					  if (q_out.y >= JOINT1_MIN && q_out.y <= JOINT1_MAX)
//						if (q_out.y >= JOINT1_MIN && q_out.y <= JOINT1_MAX && q_out.z >= JOINT2_MIN && q_out.z <= JOINT2_MAX)
//						{
//							motor1.Exp_encoder = (int32_t)((q_out.x - q_home.x) * RAD2ENC_FACTOR_JOINT0 + 0.5f);
//							motor2.Exp_encoder = (int32_t)((q_out.y - q_home.y) * RAD2ENC_FACTOR_JOINT + 0.5f);
//							motor3.Exp_encoder = (int32_t)((q_out.z - q_home.z) * RAD2ENC_FACTOR_JOINT + 0.5f);
//							ALL_num++;
//				  	} 
//				   	else
//						{
//							motor1.Exp_encoder = Encoder_Now[0];
//							motor2.Exp_encoder = Encoder_Now[1];
//							motor3.Exp_encoder = Encoder_Now[2];
//							traj_active = 0;
//				  	}
//					}
//					else 
//					{
//						motor1.Exp_encoder = Encoder_Now[0];
//						motor2.Exp_encoder = Encoder_Now[1];
//						motor3.Exp_encoder = Encoder_Now[2];
//					  traj_active = 0;
//					}

				// 电机 1 (底座)
 				PID_Control2((float)Encoder_Now[0], (float)motor1.Exp_encoder, &Motor1_PID);
				// 电机 2 `
				PID_Control2((float)Encoder_Now[1], (float)motor2.Exp_encoder, &Motor2_PID);
				// 电机 3 
				PID_Control2((float)Encoder_Now[2], (float)motor3.Exp_encoder, &Motor3_PID);
				
// 原始输入
//				vel_1 = -(int16_t)Motor1_PID.pid_out;
//				vel_2 = -(int16_t)Motor2_PID.pid_out;
//				vel_3 = -(int16_t)Motor3_PID.pid_out;

// 重力补偿
				vel_1 = -(int16_t)(Motor1_PID.pid_out);
				vel_2 = -(int16_t)(Motor2_PID.pid_out + TAU_TO_SPEED_FF1 * tau_out.y);
				vel_3 = -(int16_t)(Motor3_PID.pid_out + TAU_TO_SPEED_FF2 * tau_out.z);
					
// 速度前馈
//				float feedforward_0 = dq_out.x * RAD2ENC_FACTOR_JOINT0;
//				float feedforward_1 = dq_out.y * RAD2ENC_FACTOR_JOINT;
//				float feedforward_2 = dq_out.z * RAD2ENC_FACTOR_JOINT;

//				int16_t speed_1 = (int16_t)(-(Motor1_PID.pid_out + feedforward_0));
//				int16_t speed_2 = (int16_t)(-(Motor2_PID.pid_out + feedforward_1));
//				int16_t speed_3 = (int16_t)(-(Motor3_PID.pid_out + feedforward_2));
//				Contrl_Speed(speed_1, speed_2, speed_3, NULL);

				Contrl_Speed(vel_1, vel_2, vel_3, NULL);

				
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}
