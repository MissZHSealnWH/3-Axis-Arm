#include "ZDT_Emm.h"
#include <string.h>
// ============================================================================
// CRC-8校验表 (Maxim/Dallas单总线多项式 x^8 + x^5 + x^4 + 1 = 0x31)
// ============================================================================
static const uint8_t crc8_table[] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

// ============================================================================
// 内部函数声明
// ============================================================================
static void Emm_Send_Cmd(ZDT_Emm_t *zdt, const uint8_t *cmd, uint8_t len);
static uint8_t Emm_Calc_Checksum(ZDT_Emm_t *zdt, const uint8_t *data, uint8_t len);

// ============================================================================
// EMM库初始化
// ============================================================================
void Emm_Init(ZDT_Emm_t *zdt, uint8_t addr, UART_HandleTypeDef *huart)
{
    zdt->addr = addr;
    zdt->huart = huart;
    zdt->checksum = CHECKSUM_FIXED_6B;
    zdt->tx_busy = false;
    
    // 初始化系统状态
    zdt->sys_status.vbus_voltage = 0;
    zdt->sys_status.current = 0;
    zdt->sys_status.encoder_value = 0;
    zdt->sys_status.target_position = 0;
    zdt->sys_status.velocity = 0;
    zdt->sys_status.current_position = 0;
    zdt->sys_status.position_error = 0;
    zdt->sys_status.status.enabled = 0;
    zdt->sys_status.status.reached = 0;
    zdt->sys_status.status.blocked = 0;
    zdt->sys_status.status.block_protect = 0;
    zdt->sys_status.home_status.encoder_ready = 0;
    zdt->sys_status.home_status.calib_ready = 0;
    zdt->sys_status.home_status.homing = 0;
    zdt->sys_status.home_status.home_failed = 0;
}

void Emm_SetChecksum(ZDT_Emm_t *zdt, Emm_Checksum_Type checksum)
{
    zdt->checksum = checksum;
}

// ============================================================================
// 辅助函数
// ============================================================================
uint8_t Emm_Calc_XOR(const uint8_t *data, uint8_t len)
{
    uint8_t xor = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor ^= data[i];
    }
    return xor;
}

uint8_t Emm_Calc_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

static uint8_t Emm_Calc_Checksum(ZDT_Emm_t *zdt, const uint8_t *data, uint8_t len)
{
    switch (zdt->checksum) {
        case CHECKSUM_FIXED_6B:
            return 0x6B;
        case CHECKSUM_XOR:
            return Emm_Calc_XOR(data, len);
        case CHECKSUM_CRC8:
            return Emm_Calc_CRC8(data, len);
        default:
            return 0x6B;
    }
}


static __attribute__((section(".dma_buffer"), aligned(32))) uint8_t tx_buf_1[3][32] = {0};
static __attribute__((section(".dma_buffer"), aligned(32))) uint8_t tx_buf_2[3][32] = {0};
static __attribute__((section(".dma_buffer"), aligned(32))) uint8_t tx_buf_3[3][32] = {0};
static void Emm_Send_Cmd(ZDT_Emm_t *zdt, const uint8_t *cmd, uint8_t len)
{
    uint8_t index = zdt->addr - 1;
    uint8_t checksum = Emm_Calc_Checksum(zdt, cmd, len);
    uint8_t *tx_buf = NULL;
    static uint8_t count[3] = {0};
    if (index >= 3U) {
        return;
    }


    switch (index) {
        case 0:
            count[0] %= 3;
            tx_buf = tx_buf_1[count[0]];
            count[0]++;
            break;
        case 1:
            count[1] %= 3;
            tx_buf = tx_buf_2[count[1]];
            count[1]++;
            break;
        case 2:
            count[2] %= 3;
            tx_buf = tx_buf_3[count[2]];
            count[2]++;
            break;
        default:
            return;
    }

    for (uint8_t i = 0; i < len; i++) {
        tx_buf[i] = cmd[i];
    }
    tx_buf[len] = checksum;

    SCB_CleanDCache_by_Addr((void *)tx_buf, 32);

    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(zdt->huart, tx_buf, len + 1);
    zdt->tx_busy = (ret == HAL_OK);
}


// ============================================================================
// 控制动作命令
// ============================================================================
void Emm_V5_Enable(ZDT_Emm_t *zdt, bool enable)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xF3;              // 功能码
    cmd[2] = 0xAB;              // 辅助码
    cmd[3] = enable ? 0x01 : 0x00; // 使能状态
    cmd[4] = 0x00;              // 多机同步标志 (不启用)
    
    Emm_Send_Cmd(zdt, cmd, 5);
}

void Emm_V5_Velocity(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, uint8_t accel, bool sync)
{
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    uint8_t cmd[8];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xF6;              // 功能码
    cmd[2] = dir;               // 方向 (0=CW, 1=CCW)
    cmd[3] = (uint8_t)(speed >> 8);   // 速度高字节
    cmd[4] = (uint8_t)(speed & 0xFF); // 速度低字节
    cmd[5] = accel;             // 加速度档位
    cmd[6] = sync ? 0x01 : 0x00;      // 多机同步标志
    
    Emm_Send_Cmd(zdt, cmd, 7);
}

void Emm_V5_Position(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, uint8_t accel, 
                     int32_t pulses, bool absolute, bool sync)
{
    uint8_t cmd[13] = {0};
    cmd[0] = zdt->addr;                    // 地址
    cmd[1] = 0xFD;                         // 功能码
    cmd[2] = dir;                          // 方向
    cmd[3] = (uint8_t)(speed >> 8);        // 速度高字节
    cmd[4] = (uint8_t)(speed & 0xFF);      // 速度低字节
    cmd[5] = accel;                        // 加速度档位
    cmd[6] = (uint8_t)((pulses >> 24) & 0xFF); // 脉冲数
    cmd[7] = (uint8_t)((pulses >> 16) & 0xFF);
    cmd[8] = (uint8_t)((pulses >> 8) & 0xFF);
    cmd[9] = (uint8_t)(pulses & 0xFF);
    cmd[10] = absolute ? 0x01 : 0x00;      // 绝对/相对位置模式
    cmd[11] = sync ? 0x01 : 0x00;         // 多机同步标志
    
    Emm_Send_Cmd(zdt, cmd, 12);
}

void Emm_V5_Stop_Now(ZDT_Emm_t *zdt)
{
    uint8_t cmd[4];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xFE;              // 功能码
    cmd[2] = 0x98;              // 辅助码
    cmd[3] = 0x00;              // 多机同步标志
    
    Emm_Send_Cmd(zdt, cmd, 4);
}

void Emm_V5_Sync_Trigger(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xFF;              // 功能码
    cmd[2] = 0x66;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

// ============================================================================
// 原点回零命令
// ============================================================================
void Emm_V5_Set_Home_Origin(ZDT_Emm_t *zdt, bool store)
{
    uint8_t cmd[4];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x93;              // 功能码
    cmd[2] = 0x88;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    
    Emm_Send_Cmd(zdt, cmd, 4);
}

void Emm_V5_Trigger_Home(ZDT_Emm_t *zdt, Emm_HomeMode mode, bool sync)
{
    uint8_t cmd[4];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x9A;              // 功能码
    cmd[2] = (uint8_t)mode;     // 回零模式
    cmd[3] = sync ? 0x01 : 0x00; // 多机同步标志
    
    Emm_Send_Cmd(zdt, cmd, 4);
}

void Emm_V5_Abort_Home(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x9C;              // 功能码
    cmd[2] = 0x48;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

// ============================================================================
// 触发动作命令
// ============================================================================
void Emm_V5_Calibrate_Encoder(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x06;              // 功能码
    cmd[2] = 0x45;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

void Emm_V5_Clear_Position(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x0A;              // 功能码
    cmd[2] = 0x6D;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

void Emm_V5_Release_BlockProtect(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x0E;              // 功能码
    cmd[2] = 0x52;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

void Emm_V5_Restore_Defaults(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x0F;              // 功能码
    cmd[2] = 0x5F;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

// ============================================================================
// 读取参数命令
// ============================================================================
void Emm_V5_Read_Version(ZDT_Emm_t *zdt)
{
    uint8_t cmd[2];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x1F;              // 功能码
    
    Emm_Send_Cmd(zdt, cmd, 2);
}

void Emm_V5_Read_RL(ZDT_Emm_t *zdt)
{
    uint8_t cmd[2];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x20;              // 功能码
    
    Emm_Send_Cmd(zdt, cmd, 2);
}

void Emm_V5_Read_PID(ZDT_Emm_t *zdt)
{
    uint8_t cmd[2];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x21;              // 功能码
    
    Emm_Send_Cmd(zdt, cmd, 2);
}

void Emm_V5_Read_SystemStatus(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x43;              // 功能码
    cmd[2] = 0x7A;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

void Emm_V5_Read_Sys_Param(ZDT_Emm_t *zdt, SysParams_Emm_t s)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    
    switch (s) {
        case S_VER:       cmd[1] = 0x1F; break;
        case S_RL:        cmd[1] = 0x20; break;
        case S_PID:       cmd[1] = 0x21; break;
        case S_VBUS:      cmd[1] = 0x24; break;
        case S_CPHA:      cmd[1] = 0x27; break;
        case S_ENCL:      cmd[1] = 0x31; break;
        case S_CLKI:      cmd[1] = 0x32; break;
        case S_TPOS:      cmd[1] = 0x33; break;
        case S_SPOS:      cmd[1] = 0x34; break;
        case S_VEL:       cmd[1] = 0x35; break;
        case S_CPOS:      cmd[1] = 0x36; break;
        case S_PERR:      cmd[1] = 0x37; break;
        case S_FLAG:      cmd[1] = 0x3A; break;
        case S_OFLAG:     cmd[1] = 0x3B; break;
        case S_HOME_PARAMS: cmd[1] = 0x22; break;
        case S_DRIVER_CFG:   cmd[1] = 0x42; cmd[2] = 0x6C; Emm_Send_Cmd(zdt, cmd, 3); return;
        case S_SYS_STATUS:   Emm_V5_Read_SystemStatus(zdt); return;
        default:          cmd[1] = 0x00; break;
    }
    
    Emm_Send_Cmd(zdt, cmd, 2);
}

void Emm_V5_Read_DriverConfig(ZDT_Emm_t *zdt)
{
    uint8_t cmd[3];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x42;              // 功能码
    cmd[2] = 0x6C;              // 辅助码
    
    Emm_Send_Cmd(zdt, cmd, 3);
}

void Emm_V5_Read_HomeParams(ZDT_Emm_t *zdt)
{
    uint8_t cmd[2];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x22;              // 功能码
    
    Emm_Send_Cmd(zdt, cmd, 2);
}

// ============================================================================
// 修改参数命令
// ============================================================================
void Emm_V5_Set_Microstep(ZDT_Emm_t *zdt, bool store, uint8_t microstep)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x84;              // 功能码
    cmd[2] = 0x8A;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    cmd[4] = microstep;          // 细分值 (0=256细分)
    
    Emm_Send_Cmd(zdt, cmd, 5);
}

void Emm_V5_Set_Address(ZDT_Emm_t *zdt, bool store, uint8_t new_addr)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xAE;              // 功能码
    cmd[2] = 0x4B;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    cmd[4] = new_addr;          // 新地址
    
    Emm_Send_Cmd(zdt, cmd, 5);
}

void Emm_V5_Set_MotorMode(ZDT_Emm_t *zdt, bool store, Emm_MotorMode mode)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x46;              // 功能码
    cmd[2] = 0x69;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    cmd[4] = (uint8_t)mode;    // 模式
    
    Emm_Send_Cmd(zdt, cmd, 5);
}

void Emm_V5_Set_OpenCurrent(ZDT_Emm_t *zdt, bool store, uint16_t current)
{
    uint8_t cmd[6];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x44;              // 功能码
    cmd[2] = 0x33;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    cmd[4] = (uint8_t)(current >> 8);   // 电流高字节
    cmd[5] = (uint8_t)(current & 0xFF);  // 电流低字节
    
    Emm_Send_Cmd(zdt, cmd, 6);
}

void Emm_V5_Set_DriverConfig(ZDT_Emm_t *zdt, bool store, const Emm_DriverConfig *cfg)
{
    uint8_t cmd[36];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x48;              // 功能码
    cmd[2] = 0xD1;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    
    // 驱动配置参数
    cmd[4] = cfg->motor_type;
    cmd[5] = cfg->pulse_mode;
    cmd[6] = cfg->serial_mode;
    cmd[7] = cfg->en_polarity;
    cmd[8] = cfg->dir_polarity;
    cmd[9] = cfg->microstep;
    cmd[10] = cfg->interp_enable;
    cmd[11] = cfg->autoscreen_enable;
    cmd[12] = (uint8_t)(cfg->open_current >> 8);
    cmd[13] = (uint8_t)(cfg->open_current & 0xFF);
    cmd[14] = (uint8_t)(cfg->block_current >> 8);
    cmd[15] = (uint8_t)(cfg->block_current & 0xFF);
    cmd[16] = (uint8_t)(cfg->max_voltage >> 8);
    cmd[17] = (uint8_t)(cfg->max_voltage & 0xFF);
    cmd[18] = cfg->uart_baud;
    cmd[19] = cfg->can_baud;
    cmd[20] = cfg->id_addr;
    cmd[21] = cfg->checksum_type;
    cmd[22] = cfg->response_mode;
    cmd[23] = cfg->block_protect_enable;
    cmd[24] = (uint8_t)(cfg->block_rpm_thr >> 8);
    cmd[25] = (uint8_t)(cfg->block_rpm_thr & 0xFF);
    cmd[26] = (uint8_t)(cfg->block_ma_thr >> 8);
    cmd[27] = (uint8_t)(cfg->block_ma_thr & 0xFF);
    cmd[28] = (uint8_t)(cfg->block_ms_thr >> 8);
    cmd[29] = (uint8_t)(cfg->block_ms_thr & 0xFF);
    cmd[30] = (uint8_t)(cfg->position_window >> 8);
    cmd[31] = (uint8_t)(cfg->position_window & 0xFF);
    
    Emm_Send_Cmd(zdt, cmd, 32);
}

void Emm_V5_Set_PID(ZDT_Emm_t *zdt, bool store, const Emm_PIDParams *pid)
{
    uint8_t cmd[16];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x4A;              // 功能码
    cmd[2] = 0xC3;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    
    // Kp
    cmd[4] = (uint8_t)((pid->kp >> 24) & 0xFF);
    cmd[5] = (uint8_t)((pid->kp >> 16) & 0xFF);
    cmd[6] = (uint8_t)((pid->kp >> 8) & 0xFF);
    cmd[7] = (uint8_t)(pid->kp & 0xFF);
    
    // Ki
    cmd[8] = (uint8_t)((pid->ki >> 24) & 0xFF);
    cmd[9] = (uint8_t)((pid->ki >> 16) & 0xFF);
    cmd[10] = (uint8_t)((pid->ki >> 8) & 0xFF);
    cmd[11] = (uint8_t)(pid->ki & 0xFF);
    
    // Kd
    cmd[12] = (uint8_t)((pid->kd >> 24) & 0xFF);
    cmd[13] = (uint8_t)((pid->kd >> 16) & 0xFF);
    cmd[14] = (uint8_t)((pid->kd >> 8) & 0xFF);
    cmd[15] = (uint8_t)(pid->kd & 0xFF);
    
    Emm_Send_Cmd(zdt, cmd, 16);
}

void Emm_V5_Set_HomeParams(ZDT_Emm_t *zdt, bool store, const Emm_HomeParams *params)
{
    uint8_t cmd[17];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x4C;              // 功能码
    cmd[2] = 0xAE;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    
    cmd[4] = (uint8_t)params->mode;
    cmd[5] = (uint8_t)params->dir;
    cmd[6] = (uint8_t)(params->home_rpm >> 8);
    cmd[7] = (uint8_t)(params->home_rpm & 0xFF);
    cmd[8] = (uint8_t)((params->home_timeout >> 24) & 0xFF);
    cmd[9] = (uint8_t)((params->home_timeout >> 16) & 0xFF);
    cmd[10] = (uint8_t)((params->home_timeout >> 8) & 0xFF);
    cmd[11] = (uint8_t)(params->home_timeout & 0xFF);
    cmd[12] = (uint8_t)(params->slam_rpm >> 8);
    cmd[13] = (uint8_t)(params->slam_rpm & 0xFF);
    cmd[14] = (uint8_t)(params->slam_ma >> 8);
    cmd[15] = (uint8_t)(params->slam_ma & 0xFF);
    cmd[16] = params->poweron_enable;
    
    Emm_Send_Cmd(zdt, cmd, 17);
}

void Emm_V5_Store_VelocityParams(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, 
                                  uint8_t accel, bool en_ctrl)
{
    uint8_t cmd[9];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0xF7;              // 功能码
    cmd[2] = 0x1C;              // 辅助码
    cmd[3] = 0x01;              // 存储标志
    cmd[4] = dir;              // 方向
    cmd[5] = (uint8_t)(speed >> 8);   // 速度高字节
    cmd[6] = (uint8_t)(speed & 0xFF);  // 速度低字节
    cmd[7] = accel;             // 加速度档位
    cmd[8] = en_ctrl ? 0x01 : 0x00;
    
    Emm_Send_Cmd(zdt, cmd, 9);
}

void Emm_V5_Set_VelocityScale(ZDT_Emm_t *zdt, bool store, bool enable)
{
    uint8_t cmd[5];
    cmd[0] = zdt->addr;         // 地址
    cmd[1] = 0x4F;              // 功能码
    cmd[2] = 0x71;              // 辅助码
    cmd[3] = store ? 0x01 : 0x00; // 是否存储
    cmd[4] = enable ? 0x01 : 0x00; // 使能标志
    
    Emm_Send_Cmd(zdt, cmd, 5);
}


uint8_t rx_data_pp[32] = {0};
bool Emm_Read_vel_pos(ZDT_Emm_t *zdt, const uint8_t *rx_data, uint8_t len)
{
    if (len < 4) return false;
    if (rx_data[0] != zdt->addr) return false;
    if (rx_data[len - 1] != Emm_Calc_Checksum(zdt, rx_data, len - 1)) return false;
    if (rx_data[1] == 0x00) return false;
    memcpy(rx_data_pp, rx_data, len);
    switch (rx_data[1]) {
        case 0x35: {
            if (len < 6) return false;
            int16_t velocity = (int16_t)(((uint16_t)rx_data[3] << 8) | rx_data[4]);
            zdt->sys_status.velocity = (rx_data[2] == 0x01) ? -velocity : velocity;
            return true;
        }

        case 0x36: {
            if (len < 8) return false;
            int32_t position = ((int32_t)rx_data[3] << 24) |
                               ((int32_t)rx_data[4] << 16) |
                               ((int32_t)rx_data[5] << 8) |
                               (int32_t)rx_data[6];
            zdt->sys_status.current_position = (rx_data[2] == 0x01) ? -position : position;
            return true;
        }

        default:
            return false;
    }
}
bool Emm_Parse_Response(ZDT_Emm_t *zdt, const uint8_t *rx_data, uint8_t len)
{
    if (len < 4) return false;

    switch (rx_data[1]) {
        case 0x35:
        case 0x36:
            return Emm_Read_vel_pos(zdt, rx_data, len);

        default:
            return false;
    }
}

// ============================================================================
// 数据解析函数
// ============================================================================
// bool Emm_Parse_Response(ZDT_Emm_t *zdt, const uint8_t *rx_data, uint8_t len)
// {
//     if (len < 4) return false;

//     // 检查地址是否匹配
//     if (rx_data[0] != zdt->addr && rx_data[0] != 0x00) return false;

//     uint8_t func_code = rx_data[1];

//     switch (func_code) {
//         case 0x43: // 系统状态参数
//             if (len >= 31) {
//                 zdt->sys_status.vbus_voltage = (uint16_t)(rx_data[3] << 8) | rx_data[4];
//                 zdt->sys_status.current = (int16_t)((rx_data[5] << 8) | rx_data[6]);
//                 zdt->sys_status.encoder_value = (uint16_t)(rx_data[7] << 8) | rx_data[8];

//                 // 目标位置 (带符号)
//                 if (rx_data[9] == 0x01) { // 负数
//                     zdt->sys_status.target_position = -((int32_t)((rx_data[10] << 8) | rx_data[11]) << 16);
//                 } else {
//                     zdt->sys_status.target_position = (int32_t)((rx_data[10] << 8) | rx_data[11]) << 16;
//                 }

//                 // 转速 (带符号)
//                 if (rx_data[12] == 0x01) {
//                     zdt->sys_status.velocity = -(int16_t)((rx_data[13] << 8) | rx_data[14]);
//                 } else {
//                     zdt->sys_status.velocity = (int16_t)((rx_data[13] << 8) | rx_data[14]);
//                 }

//                 // 当前位置 (带符号)
//                 if (rx_data[15] == 0x01) {
//                     zdt->sys_status.current_position = -((int32_t)((rx_data[16] << 8) | rx_data[17]) << 16);
//                 } else {
//                     zdt->sys_status.current_position = (int32_t)((rx_data[16] << 8) | rx_data[17]) << 16;
//                 }

//                 // 位置误差 (带符号)
//                 if (rx_data[18] == 0x01) {
//                     zdt->sys_status.position_error = -((int32_t)((rx_data[19] << 8) | rx_data[20]) << 16);
//                 } else {
//                     zdt->sys_status.position_error = (int32_t)((rx_data[19] << 8) | rx_data[20]) << 16;
//                 }

//                 // 状态标志
//                 uint8_t status_byte = rx_data[28];
//                 zdt->sys_status.status.enabled = (status_byte & 0x01) ? 1 : 0;
//                 zdt->sys_status.status.reached = (status_byte & 0x02) ? 1 : 0;
//                 zdt->sys_status.status.blocked = (status_byte & 0x04) ? 1 : 0;
//                 zdt->sys_status.status.block_protect = (status_byte & 0x08) ? 1 : 0;

//                 // 回零状态标志
//                 uint8_t home_byte = rx_data[27];
//                 zdt->sys_status.home_status.encoder_ready = (home_byte & 0x01) ? 1 : 0;
//                 zdt->sys_status.home_status.calib_ready = (home_byte & 0x02) ? 1 : 0;
//                 zdt->sys_status.home_status.homing = (home_byte & 0x04) ? 1 : 0;
//                 zdt->sys_status.home_status.home_failed = (home_byte & 0x08) ? 1 : 0;

//                 return true;
//             }
//             break;

//         case 0x3A: // 电机状态标志位
//             if (len >= 4) {
//                 uint8_t status_byte = rx_data[2];
//                 zdt->sys_status.status.enabled = (status_byte & 0x01) ? 1 : 0;
//                 zdt->sys_status.status.reached = (status_byte & 0x02) ? 1 : 0;
//                 zdt->sys_status.status.blocked = (status_byte & 0x04) ? 1 : 0;
//                 zdt->sys_status.status.block_protect = (status_byte & 0x08) ? 1 : 0;
//                 return true;
//             }
//             break;

//         case 0x3B: // 回零状态标志位
//             if (len >= 4) {
//                 uint8_t home_byte = rx_data[2];
//                 zdt->sys_status.home_status.encoder_ready = (home_byte & 0x01) ? 1 : 0;
//                 zdt->sys_status.home_status.calib_ready = (home_byte & 0x02) ? 1 : 0;
//                 zdt->sys_status.home_status.homing = (home_byte & 0x04) ? 1 : 0;
//                 zdt->sys_status.home_status.home_failed = (home_byte & 0x08) ? 1 : 0;
//                 return true;
//             }
//             break;

//         default:
//             break;
//     }

//     return false;
// }

// ============================================================================
// 辅助转换函数
// ============================================================================
float Emm_Encoder_To_Angle(int32_t encoder)
{
    return (float)encoder * 360.0f / 65536.0f;
}

int32_t Emm_Angle_To_Encoder(float angle)
{
    float enc = angle * 65536.0f / 360.0f;
    if (enc < 0) enc += 65536.0f;
    return (uint16_t)enc;
}

// ============================================================================
// 电机参数配置 (1.8°电机, 16细分)
// ============================================================================
#define EMM_MOTOR_STEP_ANGLE    1.8f   // 电机步距角 (度)
#define EMM_MICROSTEP          16     // 细分步数
#define EMM_PULSES_PER_REV     (200 * EMM_MICROSTEP)  // 每圈脉冲数 = 3200

// ============================================================================
// 实用控制函数
// ============================================================================
void Emm_Move_Continuous(ZDT_Emm_t *zdt, int16_t rpm)
{
    // CW为正方向，CCW为负方向
    Emm_Direction dir = (rpm >= 0) ? DIR_CW : DIR_CCW;
    uint16_t speed = (rpm >= 0) ? (uint16_t)rpm : (uint16_t)(-rpm);
    
    Emm_V5_Velocity(zdt, dir, speed, 0, false);
}

void Emm_Move_Angle(ZDT_Emm_t *zdt, float angle_deg, uint16_t speed_rpm)
{
    // CW为正方向，CCW为负方向
    Emm_Direction dir = (angle_deg >= 0) ? DIR_CW : DIR_CCW;
    float abs_angle = (angle_deg >= 0) ? angle_deg : -angle_deg;
    
    // 转换为脉冲数 (1.8°电机，16细分: 3200脉冲/圈)
    int32_t pulses = (int32_t)(abs_angle * EMM_PULSES_PER_REV / 360.0f);
    
    Emm_V5_Position(zdt, dir, speed_rpm, 0, pulses, false, false);
}

void Emm_Move_Rounds(ZDT_Emm_t *zdt, float rounds, uint16_t speed_rpm)
{
    // CW为正方向，CCW为负方向
    Emm_Direction dir = (rounds >= 0) ? DIR_CW : DIR_CCW;
    float abs_rounds = (rounds >= 0) ? rounds : -rounds;
    
    // 转换为脉冲数 (16细分: 3200脉冲/圈)
    int32_t pulses = (int32_t)(abs_rounds * (float)EMM_PULSES_PER_REV);
    
    Emm_V5_Position(zdt, dir, speed_rpm, 0, pulses, false, false);
}

void Emm_Move_To_Absolute(ZDT_Emm_t *zdt, float angle_deg, uint16_t speed_rpm)
{
    // 限制角度在0-360范围内
    while (angle_deg < 0) angle_deg += 360;
    while (angle_deg >= 360) angle_deg -= 360;
    
    // 转换为脉冲数 (1.8°电机，16细分: 3200脉冲/圈)
    int32_t pulses = (int32_t)(angle_deg * EMM_PULSES_PER_REV / 360.0f);
    
    Emm_V5_Position(zdt, DIR_CW, speed_rpm, 0, pulses, true, false);
}

// ============================================================================
// 辅助函数 - 根据电机参数计算
// ============================================================================
float Emm_Pulses_To_Rounds(int32_t pulses)
{
    return (float)pulses / (float)EMM_PULSES_PER_REV;
}

int32_t Emm_Rounds_To_Pulses(float rounds)
{
    return (int32_t)(rounds * (float)EMM_PULSES_PER_REV);
}

float Emm_Angle_To_Rounds(float angle_deg)
{
    return angle_deg / 360.0f;
}

float Emm_Rounds_To_Angle(float rounds)
{
    return rounds * 360.0f;
}

float Emm_Pulses_To_Angle(int32_t pulses)
{
    return (float)pulses * 360.0f / (float)EMM_PULSES_PER_REV;
}

int32_t Emm_Angle_To_Pulses(float angle_deg)
{
    return (int32_t)(angle_deg * (float)EMM_PULSES_PER_REV / 360.0f);
}
