#include "ARM2.h"
#include "Task_Init.h"
#include "motor520.h"
#include "motors520.h"
#include "PID_old.h"
#include "micro_kdl.h"
#include "uplink_drv.h"
#include "ZDT_Emm.h"
#include "usart.h"

extern uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
extern UplinkCommand latest_cmd;
extern volatile uint8_t new_cmd_flag;
 
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static Mikdl_Robot RobotArm;          // 机械臂结构体
static Mikdl_Vector3 q_curr;          // 当前关节角度
static Mikdl_Vector3 p_curr;          // 当前末端位置
static Mikdl_Vector3 q_home;          // 上电时机械臂姿态对应的关节角

// 梯形规划轨迹相关
static Mikdl_TrapProfile trap;
static Mikdl_Vector3 p_start;
static Mikdl_Vector3 dir_unit;
static float total_dist;
static uint32_t traj_start_tick;
static uint8_t traj_active = 0;

ZDT_Emm_t ZDT_Emm[3] = 
{
    {.huart = &huart8, .addr = 0x01, .checksum = CHECKSUM_FIXED_6B},
    {.huart = &huart5, .addr = 0x02, .checksum = CHECKSUM_FIXED_6B},
    {.huart = &huart9, .addr = 0x03, .checksum = CHECKSUM_FIXED_6B}
};

Joint_t Joint[3] = {
    {.zdt = &ZDT_Emm[0], .dir = 2.5f },
    {.zdt = &ZDT_Emm[1], .dir = 1.0f },
    {.zdt = &ZDT_Emm[2], .dir = 1.0f }
};

int ALL_num = 0; // 总通信计数
uint8_t cnt = 0; // 计数反馈and发送
float start_rad[3];

static __attribute__((section(".dma_buffer"), aligned(32))) uint8_t rx_buf[3][3][32];
static uint16_t parse_size[3][3];
static volatile uint8_t rx_write_index[3];
static volatile uint8_t rx_read_index[3];
static volatile uint8_t rx_pending_count[3];



TaskHandle_t ARM2_Handle; 
void ARM2(void *pvParameters)
{
    // 启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(Joint[0].zdt->huart, rx_buf[0][0], sizeof(rx_buf[0][0]));
    HAL_UARTEx_ReceiveToIdle_DMA(Joint[1].zdt->huart, rx_buf[1][0], sizeof(rx_buf[1][0]));
    HAL_UARTEx_ReceiveToIdle_DMA(Joint[2].zdt->huart, rx_buf[2][0], sizeof(rx_buf[2][0]));
    
    // 关闭过半中断（Half Transfer Interrupt）
    __HAL_DMA_DISABLE_IT(Joint[0].zdt->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(Joint[1].zdt->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(Joint[2].zdt->huart->hdmarx, DMA_IT_HT);
    
    Emm_Init(Joint[0].zdt, 0x01, &huart8);
    Emm_Init(Joint[1].zdt, 0x02, &huart5);
    Emm_Init(Joint[2].zdt, 0x03, &huart9);

    vTaskDelay(1500);

    Emm_V5_Enable(Joint[0].zdt, true);
    Emm_V5_Enable(Joint[1].zdt, true);
    Emm_V5_Enable(Joint[2].zdt, true);

    vTaskDelay(1000);

    Emm_V5_Clear_Position(Joint[0].zdt);
    Emm_V5_Clear_Position(Joint[1].zdt);
    Emm_V5_Clear_Position(Joint[2].zdt);

    Joint[0].zdt_st.exp_pos = DEG_TO_RAD(2.5f * start_rad[0]);
    Joint[1].zdt_st.exp_pos = DEG_TO_RAD(start_rad[1]);
    Joint[2].zdt_st.exp_pos = DEG_TO_RAD(start_rad[2]);

	
 TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
    cnt++;
    cnt%=3;
    if(cnt==0)
    {
			SysParams_Emm_t cur_vel = S_VEL;
			for(uint8_t i=0;i<3;i++)
			{
				Emm_V5_Read_Sys_Param(Joint[i].zdt,cur_vel);
			}
            
    }
    else if(cnt==2)
    { 
			SysParams_Emm_t cur_pos = S_CPOS;
			for(uint8_t i=0;i<3;i++)
			{
				Emm_V5_Read_Sys_Param(Joint[i].zdt,cur_pos);
			}
    }
		
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}

void RobotArm_Init(void)
{	 
    // 初始关节角度 (0, 90, 90)
    q_home.x = 0.0f;      // 底座无要求
    q_home.y = 1.5708f;   // 大臂90°
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
		  // 更新真实关节角和末端位置
			q_curr.x = q_home.x + Joint[0].cur_pos;   // 底座
			q_curr.y = q_home.y + Joint[1].cur_pos;   // 大臂
			q_curr.z = q_home.z + Joint[2].cur_pos;   // 小臂
		
			mikdl_forward_kinematics(&RobotArm, &q_curr, &p_curr);
			
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
				//   Y轴 — 垂直于运动平面（水平向左/右
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
				if (ret == MIKDL_SUCCESS) 
				{
					Joint[0].zdt_st.exp_pos = (q_out.x - q_home.x) * Joint[0].dir;
					Joint[1].zdt_st.exp_pos = (q_out.y - q_home.y) * Joint[1].dir;
					Joint[2].zdt_st.exp_pos = (q_out.z - q_home.z) * Joint[2].dir;
					
				  for(uint8_t i = 0;i < 3;i++)
			 		{
			 			Emm_V5_Position(Joint[i].zdt, Joint[i].zdt_st.exp_pos > 0 ? DIR_CW : DIR_CCW, 20, 250,Emm_Angle_To_Pulses(RAD_TO_DEG(fabsf(Joint[i].zdt_st.exp_pos))), true, false);
			 		}
			 		ALL_num++;
				}

				
/******************************************************************************************/
					
				
 vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(10));
	}
}

TaskHandle_t ZDT_Parse_Run_Handle;
void ZDT_Parse_Run(void *parameter)
{
    for (;;) {
        uint32_t notified = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFFu, &notified, portMAX_DELAY) == pdTRUE) {
            for (uint8_t i = 0; i < 3; i++) {
                if (notified & (1u << i)) {
                    while (rx_pending_count[i] > 0U) {
                        uint8_t read_index = rx_read_index[i];
                        ZDT_Parse_One(i, rx_buf[i][read_index], parse_size[i][read_index]);
                        rx_read_index[i] = (read_index + 1U) % 3U;
                        rx_pending_count[i]--;
                    }
                }
            }
        }
    }
}

// 步进电机回调数据处理 此处回调为弧度
static void ZDT_Parse_One(uint8_t buf_index, const uint8_t *data, uint16_t size)
{
    Emm_Read_vel_pos(Joint[buf_index].zdt, data, size);

    Joint[buf_index].zdt_st.cur_pos = DEG_TO_RAD(Emm_Encoder_To_Angle(Joint[buf_index].zdt->sys_status.current_position));
    Joint[buf_index].zdt_st.cur_vel = Joint[buf_index].zdt->sys_status.velocity * 2.0f * M_PI / 30.0f;

    Joint[buf_index].cur_pos = (Joint[buf_index].zdt_st.cur_pos / Joint[buf_index].dir);
    Joint[buf_index].cur_vel = (Joint[buf_index].zdt_st.cur_vel / Joint[buf_index].dir);
 
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t buf_index = 0xFF;

    if (huart == ZDT_Emm[0].huart) buf_index = 0;
    else if (huart == ZDT_Emm[1].huart) buf_index = 1;
    else if (huart == ZDT_Emm[2].huart) buf_index = 2;

    if (buf_index != 0xFF) {
        uint8_t write_index = rx_write_index[buf_index];

        if (Size > sizeof(rx_buf[buf_index][write_index])) {
            Size = sizeof(rx_buf[buf_index][write_index]);
        }

        parse_size[buf_index][write_index] = Size;

        if (rx_pending_count[buf_index] < 3U) {
            rx_pending_count[buf_index]++;
        } else {
            rx_read_index[buf_index] = (rx_read_index[buf_index] + 1U) % 3U;
        }

        rx_write_index[buf_index] = (write_index + 1U) % 3U;

        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf[buf_index][rx_write_index[buf_index]], sizeof(rx_buf[buf_index][rx_write_index[buf_index]]));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(ZDT_Parse_Run_Handle, (1u << buf_index), eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
    }
		
		if (huart->Instance == USART1 && Size == UPLINK_FRAME_LEN) 
		{
			if (dma_rx_buf[0] == FRAME_START && dma_rx_buf[UPLINK_FRAME_LEN -1 ] == FRAME_LAST) 
			{
				latest_cmd.x =     (int8_t)dma_rx_buf[1];
				latest_cmd.y =     (int8_t)dma_rx_buf[2];
				latest_cmd.z =     (int8_t)dma_rx_buf[3];
				latest_cmd.one =   (int8_t)dma_rx_buf[4];
				latest_cmd.two =   (int8_t)dma_rx_buf[5];
				latest_cmd.three = (int8_t)dma_rx_buf[6];
				new_cmd_flag = 1;
			}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, UPLINK_FRAME_LEN);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < 3; i++) {
        if (ZDT_Emm[i].huart == huart) {
            ZDT_Emm[i].tx_busy = true;
            break;
        }
    }
    
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UART_DMAStop(huart);

    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF |
                              UART_CLEAR_FEF |
                              UART_CLEAR_NEF |
                              UART_CLEAR_PEF);

    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    // 重新启动接收并关闭过半中断
    if (huart == ZDT_Emm[0].huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(Joint[0].zdt->huart, rx_buf[0][rx_write_index[0]], sizeof(rx_buf[0][rx_write_index[0]]));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    } else if (huart == ZDT_Emm[1].huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(Joint[1].zdt->huart, rx_buf[1][rx_write_index[1]], sizeof(rx_buf[1][rx_write_index[1]]));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    } else if (huart == ZDT_Emm[2].huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(Joint[2].zdt->huart, rx_buf[2][rx_write_index[2]], sizeof(rx_buf[2][rx_write_index[2]]));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}
