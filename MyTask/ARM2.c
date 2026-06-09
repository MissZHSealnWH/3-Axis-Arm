#include "ARM2.h"
#include "Task_Init.h"
#include "uplink_drv.h"
#include "ZDT_Emm.h"
#include "usart.h"
#include <math.h>

/* ===== 机械参数 ===== */
#define L1      0.0965f  /* 大臂有效长度: 70mm臂 + 26.5mm肘偏移 */
#define L2      0.152f   /* 小臂长度 */
#define DX_BASE 0.033f   /* 底座→大臂转轴 水平偏移 22mm */
#define DZ_BASE 0.0235f  /* 底座→大臂转轴 高度偏移 */

extern SemaphoreHandle_t action_semaphore;
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

uint8_t dma_rx_buf[UPLINK_FRAME_LEN];
UplinkCommand latest_cmd;
volatile uint8_t new_cmd_flag = 0;

int Uplink_GetCommand(UplinkCommand *cmd) 
{
	if (new_cmd_flag) { new_cmd_flag = 0; *cmd = latest_cmd; return 1; }
	return 0;
}

/* ===== 数学宏 ===== */
#define S(x)  sin(x)
#define C(x)  cos(x)
#define A2(y,x) atan2(y,x)
#define SQ(x) sqrt(x)
#define FA(x) fabs(x)

/* FK: (q1底座, q2大臂, q3小臂) -> 末端 (r, z) */
static void fk(float qy, float qz, float *r, float *z)
{
	float s1=S(qy), c1=C(qy);
	float s12=S(qy+qz), c12=C(qy+qz);
	*r = DX_BASE + L1*c1 + L2*c12;
	*z = DZ_BASE + L1*s1 + L2*s12;
}

/* IK: (r, z) -> (q2, q3) 闭式解, 选离(warm_q2, warm_q3)最近的分支 */
static int ik_2d(float r, float z, float warm_q2, float warm_q3, float *o2, float *o3)
{
	float rr = r - DX_BASE;
	float zz = z - DZ_BASE;
	float d2 = rr*rr + zz*zz;
	float c3 = (d2 - L1*L1 - L2*L2) / (2.0f*L1*L2);
	if(c3 > 1.0f) c3 = 1.0f; if(c3 < -1.0f) c3 = -1.0f;
	float a3 = acos(c3), b3 = -a3;
	float q3 = (FA(a3-warm_q3) <= FA(b3-warm_q3)) ? a3 : b3;
	float q2 = A2(zz,rr) - A2(L2*S(q3), L1+L2*C(q3));
	if(rr < 1e-6f && FA(zz) < 1e-6f) { q2 = warm_q2; q3 = warm_q3; }
	*o2 = q2; *o3 = q3;
	return 1;
}

/* ===== 全局状态 ===== */
static const float q_home_x = 0;
static const float q_home_y = DEG_TO_RAD(90.0f);
static const float q_home_z = DEG_TO_RAD(-90.0f);
static float p_tgt_x, p_tgt_y, p_tgt_z;
static float last_ep[3] = {0,0,0};   /* 上次发送的电机指令, 防重复发送 */

ZDT_Emm_t ZDT_Emm[3] = 
{
    {.huart = &huart5, .addr = 0x01, .checksum = CHECKSUM_FIXED_6B},
    {.huart = &huart8, .addr = 0x02, .checksum = CHECKSUM_FIXED_6B},
    {.huart = &huart9, .addr = 0x03, .checksum = CHECKSUM_FIXED_6B}
};

Joint_t Joint[3] = {
    {.zdt = &ZDT_Emm[0], .dir = -2.0f },
    {.zdt = &ZDT_Emm[1], .dir = -2.0f },
    {.zdt = &ZDT_Emm[2], .dir =  1.0f }
};

int ALL_num = 0;
volatile uint8_t cnt = 0;
float start_rad[3] = {0,0,0};

static __attribute__((section(".dma_buffer"), aligned(32))) uint8_t rx_buf[3][3][32];
static uint16_t parse_size[3][3];
static volatile uint8_t rx_write_index[3];
static volatile uint8_t rx_read_index[3];
static volatile uint8_t rx_pending_count[3];

UplinkCommand cmd;
TaskHandle_t ARM2_Handle; 

void ARM2(void *pvParameters)
{
	Uplink_Init();

	HAL_UARTEx_ReceiveToIdle_DMA(Joint[0].zdt->huart, rx_buf[0][0], sizeof(rx_buf[0][0]));
	HAL_UARTEx_ReceiveToIdle_DMA(Joint[1].zdt->huart, rx_buf[1][0], sizeof(rx_buf[1][0]));
	HAL_UARTEx_ReceiveToIdle_DMA(Joint[2].zdt->huart, rx_buf[2][0], sizeof(rx_buf[2][0]));
	__HAL_DMA_DISABLE_IT(Joint[0].zdt->huart->hdmarx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(Joint[1].zdt->huart->hdmarx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(Joint[2].zdt->huart->hdmarx, DMA_IT_HT);

	Emm_Init(Joint[0].zdt, 0x01, &huart5);
	Emm_Init(Joint[1].zdt, 0x02, &huart8);
	Emm_Init(Joint[2].zdt, 0x03, &huart9);

	vTaskDelay(1500);
	Emm_V5_Enable(Joint[0].zdt, true);
	Emm_V5_Enable(Joint[1].zdt, true);
	Emm_V5_Enable(Joint[2].zdt, true);

	vTaskDelay(1000);
	for (int i = 0; i < 3; i++) Emm_V5_Clear_Position(Joint[i].zdt);
	vTaskDelay(50);

	Joint[0].zdt_st.exp_pos = DEG_TO_RAD(start_rad[0]);
	Joint[1].zdt_st.exp_pos = DEG_TO_RAD(start_rad[1]);
	Joint[2].zdt_st.exp_pos = DEG_TO_RAD(start_rad[2]);

	/* 初始化目标 = home姿态的FK */
	{
		float r0, z0;
		fk(q_home_y, q_home_z, &r0, &z0);
		p_tgt_x = r0; p_tgt_y = 0; p_tgt_z = z0;
	}

	TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
		cnt = (cnt+1) % 3;
		if(cnt == 0)
		{
			SysParams_Emm_t v = S_VEL;
			for(uint8_t i = 0; i < 3; i++) Emm_V5_Read_Sys_Param(Joint[i].zdt, v);
		}
		else if(cnt == 1)
		{
			if (Uplink_GetCommand(&cmd))
			{
				/* 累积目标 */
				p_tgt_x += (float)cmd.x * STEP_SIZE;
				p_tgt_y += (float)cmd.y * STEP_SIZE;
				p_tgt_z += (float)cmd.z * STEP_SIZE;

				/* IK */
				float q1 = A2(p_tgt_y, p_tgt_x);
				float r  = SQ(p_tgt_x*p_tgt_x + p_tgt_y*p_tgt_y);
				float q2, q3;
				ik_2d(r, p_tgt_z, q_home_y, q_home_z, &q2, &q3);

				/* 三个电机的exp_pos */
				float ep[3];
				ep[0] = (q1 - q_home_x) * Joint[0].dir;
				ep[1] = -(q2 - q_home_y) * Joint[1].dir;
				ep[2] = (q3 - q_home_z) * Joint[2].dir;

				/* 逐电机发送, 死区0.02°防抖 */
				for(uint8_t i = 0; i < 3; i++)
				{
					float diff = FA(ep[i] - last_ep[i]);
					if(diff < 0.0005f) continue; /* 变动<0.03°不重发 */

					float deg = RAD_TO_DEG(FA(ep[i]));
					if(deg < 0.02f) continue;    /* 死区 */

					Emm_V5_Position(Joint[i].zdt,
						ep[i] > 0 ? DIR_CW : DIR_CCW,
						10, 100,    /* 速度10, 加速度100 → 平滑 */
						Emm_Angle_To_Pulses(deg), true, false);
					last_ep[i] = ep[i];
				}
				ALL_num++;
			}
		}
		else if(cnt == 2)
		{
			SysParams_Emm_t p = S_CPOS;
			for(uint8_t i = 0; i < 3; i++) Emm_V5_Read_Sys_Param(Joint[i].zdt, p);
		}
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(7));
	}
}


TaskHandle_t ZDT_Parse_Run_Handle;
void ZDT_Parse_Run(void *parameter)
{
	(void)parameter;
	for (;;) {
		uint32_t notified = 0;
		if (xTaskNotifyWait(0, 0xFFFFFFFFu, &notified, portMAX_DELAY) == pdTRUE) {
			for (uint8_t i = 0; i < 3; i++) {
				if (notified & (1u << i)) {
					while (rx_pending_count[i] > 0U) {
						uint8_t ri = rx_read_index[i];
						ZDT_Parse_One(i, rx_buf[i][ri], parse_size[i][ri]);
						rx_read_index[i] = (ri + 1U) % 3U;
						rx_pending_count[i]--;
					}
				}
			}
		}
	}
}

static void ZDT_Parse_One(uint8_t b, const uint8_t *d, uint16_t s)
{
	Emm_Read_vel_pos(Joint[b].zdt, d, s);
	Joint[b].zdt_st.cur_pos = DEG_TO_RAD(Emm_Encoder_To_Angle(Joint[b].zdt->sys_status.current_position));
	Joint[b].zdt_st.cur_vel = Joint[b].zdt->sys_status.velocity * 2.0f * M_PI / 30.0f;
	Joint[b].cur_pos = Joint[b].zdt_st.cur_pos / Joint[b].dir;
	Joint[b].cur_vel = Joint[b].zdt_st.cur_vel / Joint[b].dir;
}

/*
 * ===== 上行协议 (ASCII) =====
 * 帧格式 (8字节):
 *   byte0: 'A' (0x41)  帧头
 *   byte1: '0'-'9'     cmd.x 幅度 (0-9)
 *   byte2: '0'-'9'     cmd.y 幅度 (0-9)
 *   byte3: '0'-'9'     cmd.z 幅度 (0-9)
 *   byte4: '0'正/'1'负  x符号
 *   byte5: '0'正/'1'负  y符号
 *   byte6: '0'正/'1'负  z符号
 *   byte7: 'B' (0x42)  帧尾
 *
 * 例: 向前5步 → "A050000B"
 *      向左3步 → "A300000B"  
 *      向后2步 → "A200100B"
 *      向上4步 → "A004000B"
 *      向下1步 → "A001001B"
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t b = 0xFF;
	if (huart == ZDT_Emm[0].huart) b = 0;
	else if (huart == ZDT_Emm[1].huart) b = 1;
	else if (huart == ZDT_Emm[2].huart) b = 2;

	if (huart->Instance == USART10 && Size == UPLINK_FRAME_LEN) 
	{
		if (dma_rx_buf[0] == FRAME_START && dma_rx_buf[UPLINK_FRAME_LEN-1] == FRAME_LAST) 
		{
			latest_cmd.x =     (int8_t)(dma_rx_buf[1] - '0');
			latest_cmd.y =     (int8_t)(dma_rx_buf[2] - '0');
			latest_cmd.z =     (int8_t)(dma_rx_buf[3] - '0');
			latest_cmd.one =   (int8_t)(dma_rx_buf[4] - '0');
			latest_cmd.two =   (int8_t)(dma_rx_buf[5] - '0');
			latest_cmd.three = (int8_t)(dma_rx_buf[6] - '0');
			if(latest_cmd.one)   latest_cmd.x = -latest_cmd.x;
			if(latest_cmd.two)   latest_cmd.y = -latest_cmd.y;
			if(latest_cmd.three) latest_cmd.z = -latest_cmd.z;
			new_cmd_flag = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buf, UPLINK_FRAME_LEN);
		__HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
	}

	if (b != 0xFF) {
		uint8_t wi = rx_write_index[b];
		if (Size > sizeof(rx_buf[b][wi])) Size = sizeof(rx_buf[b][wi]);
		parse_size[b][wi] = Size;
		if (rx_pending_count[b] < 3U) rx_pending_count[b]++;
		else rx_read_index[b] = (rx_read_index[b] + 1U) % 3U;
		rx_write_index[b] = (wi + 1U) % 3U;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf[b][rx_write_index[b]], sizeof(rx_buf[b][rx_write_index[b]]));
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

		BaseType_t xWoken = pdFALSE;
		xTaskNotifyFromISR(ZDT_Parse_Run_Handle, (1u << b), eSetBits, &xWoken);
		portYIELD_FROM_ISR(xWoken);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	for (uint8_t i = 0; i < 3; i++) { if (ZDT_Emm[i].huart == huart) { ZDT_Emm[i].tx_busy = true; break; } }
	__HAL_UART_CLEAR_IDLEFLAG(huart); HAL_UART_DMAStop(huart);
	__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF|UART_CLEAR_FEF|UART_CLEAR_NEF|UART_CLEAR_PEF);
	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
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
