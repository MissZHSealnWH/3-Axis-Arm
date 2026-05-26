#ifndef ZDT_EMM_H
#define ZDT_EMM_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "usart.h"
#ifdef __cplusplus
extern "C" {
#endif



// ============================================================================
// EMM库版本信息
// ============================================================================
#define EMM_VERSION_MAJOR   5
#define EMM_VERSION_MINOR   0
#define EMM_VERSION_PATCH   0

// ============================================================================
// 通讯校验方式
// ============================================================================
typedef enum {
    CHECKSUM_FIXED_6B = 0,    // 固定校验字节 0x6B (默认)
    CHECKSUM_XOR,              // XOR异或校验
    CHECKSUM_CRC8,             // CRC-8校验
    CHECKSUM_MODBUS            // Modbus校验
} Emm_Checksum_Type;

// ============================================================================
// 电机控制模式
// ============================================================================
typedef enum {
    MOTOR_MODE_OPEN = 1,       // 开环模式
    MOTOR_MODE_FOC = 2,        // FOC矢量闭环模式
    MOTOR_MODE_PULSE = 3       // 脉冲控制模式
} Emm_MotorMode;

// ============================================================================
// 回零模式
// ============================================================================
typedef enum {
    HOME_MODE_NEAREST = 0,     // 单圈就近回零
    HOME_MODE_DIR = 1,         // 单圈方向回零
    HOME_MODE_SENLESS = 2,     // 多圈无限位碰撞回零
    HOME_MODE_ENDSTOP = 3      // 多圈有限位开关回零
} Emm_HomeMode;

// ============================================================================
// 回零方向
// ============================================================================
typedef enum {
    DIR_CW = 0,                // 逆时针 右
    DIR_CCW = 1                // 顺时针 左
} Emm_Direction;

// ============================================================================
// 通讯控制命令应答模式
// ============================================================================
typedef enum {
    RESPONSE_NONE = 0,         // 不返回确认收到命令，不返回到位命令
    RESPONSE_RECEIVE = 1,      // 只返回确认收到命令 (默认)
    RESPONSE_REACHED = 2,      // 只在位置模式命令时返回到位命令
    RESPONSE_BOTH = 3,         // 既返回确认收到命令，也在位置模式命令时返回到位命令
    RESPONSE_OTHER = 4         // 位置模式下只返回到位命令，其他返回确认收到命令
} Emm_ResponseMode;

// ============================================================================
// 串口通讯速率
// ============================================================================
typedef enum {
    BAUD_9600 = 0,
    BAUD_19200 = 1,
    BAUD_25000 = 2,
    BAUD_38400 = 3,
    BAUD_57600 = 4,
    BAUD_115200 = 5,
    BAUD_256000 = 6,
    BAUD_512000 = 7,
    BAUD_921600 = 8
} Emm_BaudRate;

// ============================================================================
// 系统参数类型
// ============================================================================
typedef enum {
    S_VER = 0,                 // 读取固件版本
    S_RL = 1,                  // 读取相电阻和相电感
    S_PID = 2,                 // 读取位置环PID参数
    S_VBUS = 3,                // 读取总线电压
    S_CBUS = 4,                // 读取总线电流
    S_CPHA = 5,                // 读取相电流
    S_ENCO = 6,                // 读取编码器原始值
    S_ENCL = 7,                // 读取经过线性化校准后的编码器值
    S_CLKI = 8,                // 读取输入脉冲数
    S_TPOS = 9,                // 读取电机目标位置
    S_SPOS = 10,               // 读取电机实时设定的目标位置
    S_VEL = 11,                // 读取电机实时转速
    S_CPOS = 12,               // 读取电机实时位置
    S_PERR = 13,               // 读取电机位置误差
    S_FLAG = 14,               // 读取电机状态标志位
    S_OFLAG = 15,              // 读取回零状态标志位
    S_HOME_PARAMS = 16,        // 读取原点回零参数
    S_DRIVER_CFG = 17,         // 读取驱动配置参数
    S_SYS_STATUS = 18          // 读取系统状态参数
} SysParams_Emm_t;

// ============================================================================
// 电机状态标志位
// ============================================================================
typedef struct {
    uint8_t enabled      : 1; // 电机使能状态
    uint8_t reached       : 1; // 电机到位标志
    uint8_t blocked       : 1; // 电机堵转标志
    uint8_t block_protect : 1; // 堵转保护标志
} Emm_StatusFlags;

// ============================================================================
// 回零状态标志位
// ============================================================================
typedef struct {
    uint8_t encoder_ready : 1; // 编码器就绪状态
    uint8_t calib_ready   : 1; // 校准表就绪状态
    uint8_t homing        : 1; // 正在回零
    uint8_t home_failed   : 1; // 回零失败
} Emm_HomeStatusFlags;

// ============================================================================
// 驱动配置参数
// ============================================================================
typedef struct {
    uint8_t motor_type;           // 电机类型 (25=1.8°, 50=0.9°)
    uint8_t pulse_mode;           // 脉冲端口控制模式
    uint8_t serial_mode;          // 通讯端口复用模式
    uint8_t en_polarity;          // En引脚有效电平
    uint8_t dir_polarity;         // Dir引脚有效方向
    uint8_t microstep;            // 细分 (0=256)
    uint8_t interp_enable;        // 细分插补功能
    uint8_t autoscreen_enable;    // 自动熄屏功能
    uint16_t open_current;        // 开环模式工作电流 (mA)
    uint16_t block_current;       // 闭环模式堵转最大电流 (mA)
    uint16_t max_voltage;         // 闭环模式最大输出电压 (mV)
    uint8_t uart_baud;            // 串口波特率
    uint8_t can_baud;             // CAN通讯速率
    uint8_t id_addr;              // ID地址 (1-255)
    uint8_t checksum_type;       // 通讯校验方式
    uint8_t response_mode;        // 控制命令应答模式
    uint8_t block_protect_enable; // 堵转保护功能
    uint16_t block_rpm_thr;       // 堵转保护转速阈值 (RPM)
    uint16_t block_ma_thr;        // 堵转保护电流阈值 (mA)
    uint16_t block_ms_thr;        // 堵转保护检测时间阈值 (ms)
    uint16_t position_window;     // 位置到达窗口 (0.1°)
} Emm_DriverConfig;

// ============================================================================
// 系统状态参数
// ============================================================================
typedef struct {
    uint16_t vbus_voltage;        // 总线电压 (mV)
    int16_t current;              // 总线电流 (mA)
    uint16_t encoder_value;       // 校准后编码器值
    int32_t target_position;       // 电机目标位置
    int16_t velocity;             // 电机实时转速 (RPM)
    int32_t current_position;     // 电机实时位置
    int32_t position_error;       // 电机位置误差
    Emm_StatusFlags status;       // 电机状态标志
    Emm_HomeStatusFlags home_status; // 回零状态标志
} Emm_SystemStatus;

// ============================================================================
// 回零参数
// ============================================================================
typedef struct {
    Emm_HomeMode mode;            // 回零模式
    Emm_Direction dir;             // 回零方向
    uint16_t home_rpm;            // 回零转速 (RPM)
    uint32_t home_timeout;        // 回零超时时间 (ms)
    uint16_t slam_rpm;            // 无限位碰撞回零检测转速 (RPM)
    uint16_t slam_ma;             // 无限位碰撞回零检测电流 (mA)
    uint16_t slam_ms;             // 无限位碰撞回零检测时间 (ms)
    uint8_t poweron_enable;       // 上电自动触发回零功能
} Emm_HomeParams;

// ============================================================================
// 速度模式参数
// ============================================================================
typedef struct {
    Emm_Direction dir;            // 旋转方向
    uint16_t speed;               // 速度 (RPM)
    uint8_t accel;               // 加速度档位 (0-255)
} Emm_VelocityParams;

// ============================================================================
// 位置模式参数
// ============================================================================
typedef struct {
    Emm_Direction dir;            // 旋转方向
    uint16_t speed;               // 速度 (RPM)
    uint8_t accel;               // 加速度档位 (0-255)
    int32_t pulses;               // 脉冲数
    bool absolute;               // true=绝对位置模式, false=相对位置模式
} Emm_PositionParams;

// ============================================================================
// PID参数
// ============================================================================
typedef struct {
    uint32_t kp;                  // 比例系数
    uint32_t ki;                  // 积分系数
    uint32_t kd;                  // 微分系数
} Emm_PIDParams;

// ============================================================================
// 电机实例
// ============================================================================
typedef struct {
    uint8_t addr;                 // 电机地址 (1-255)
    UART_HandleTypeDef *huart;    // UART句柄
    Emm_Checksum_Type checksum;   // 校验方式
    Emm_SystemStatus sys_status;  // 系统状态
    Emm_DriverConfig driver_cfg;  // 驱动配置
    Emm_HomeParams home_params;   // 回零参数
    bool tx_busy;                // 发送忙标志
} ZDT_Emm_t;

// ============================================================================
// EMM库初始化
// ============================================================================
/**
 * @brief  初始化EMM库
 * @param  zdt: 电机实例指针
 * @param  addr: 电机地址 (1-255)
 * @param  huart: UART句柄
 * @retval None
 */
void Emm_Init(ZDT_Emm_t *zdt, uint8_t addr, UART_HandleTypeDef *huart);

/**
 * @brief  设置校验方式
 * @param  zdt: 电机实例指针
 * @param  checksum: 校验方式
 * @retval None
 */
//void Emm_SetChecksum(ZDT_Emm_t *zdt, Emm_Checksum_Type checksum);

// ============================================================================
// 控制动作命令
// ============================================================================
/**
 * @brief  电机使能控制
 * @param  zdt: 电机实例指针
 * @param  enable: true=使能, false=不使能
 * @retval None
 */
void Emm_V5_Enable(ZDT_Emm_t *zdt, bool enable);

/**
 * @brief  速度模式控制
 * @param  zdt: 电机实例指针
 * @param  dir: 方向 (DIR_CW/DIR_CCW)
 * @param  speed: 速度 (RPM, 范围1-3000)
 * @param  accel: 加速度档位 (0-255, 0=不使用曲线加减速)
 * @param  sync: 多机同步标志
 * @retval None
 * @note   曲线加减速时间计算: t2-t1 = (256-acc)*50us, Vt2 = Vt1+1RPM
 */
void Emm_V5_Velocity(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, uint8_t accel, bool sync);

/**
 * @brief  位置模式控制
 * @param  zdt: 电机实例指针
 * @param  dir: 方向 (DIR_CW/DIR_CCW)
 * @param  speed: 速度 (RPM, 范围1-3000)
 * @param  accel: 加速度档位 (0-255, 0=不使用曲线加减速)
 * @param  pulses: 脉冲数 (16细分下3200脉冲=1圈)
 * @param  absolute: true=绝对位置模式, false=相对位置模式
 * @param  sync: 多机同步标志
 * @retval None
 */
void Emm_V5_Position(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, uint8_t accel, 
                     int32_t pulses, bool absolute, bool sync);

/**
 * @brief  立即停止 (紧急刹车)
 * @param  zdt: 电机实例指针
 * @retval None
 */
void Emm_V5_Stop_Now(ZDT_Emm_t *zdt);

/**
 * @brief  多机同步运动触发
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   在发送位置/速度模式命令时设置sync=true，所有电机等待此命令后同时开始运动
 */
//void Emm_V5_Sync_Trigger(ZDT_Emm_t *zdt);

// ============================================================================
// 原点回零命令
// ============================================================================
/**
 * @brief  设置单圈回零的零点位置
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @retval None
 */
//void Emm_V5_Set_Home_Origin(ZDT_Emm_t *zdt, bool store);

/**
 * @brief  触发回零操作
 * @param  zdt: 电机实例指针
 * @param  mode: 回零模式
 * @param  sync: 多机同步标志
 * @retval None
 */
//void Emm_V5_Trigger_Home(ZDT_Emm_t *zdt, Emm_HomeMode mode, bool sync);

/**
 * @brief  强制中断并退出回零操作
 * @param  zdt: 电机实例指针
 * @retval None
 */
//void Emm_V5_Abort_Home(ZDT_Emm_t *zdt);

// ============================================================================
// 触发动作命令
// ============================================================================
/**
 * @brief  触发编码器校准
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   校准前请确保电机空载
 */
//void Emm_V5_Calibrate_Encoder(ZDT_Emm_t *zdt);

/**
 * @brief  将当前位置角度清零
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   同时清零位置误差和脉冲数
 */
void Emm_V5_Clear_Position(ZDT_Emm_t *zdt);

/**
 * @brief  解除堵转保护
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   电机发生堵转后发送此命令可解除保护
 */
//void Emm_V5_Release_BlockProtect(ZDT_Emm_t *zdt);

/**
 * @brief  恢复出厂设置
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   恢复后需要重新上电并校准编码器
 */
//void Emm_V5_Restore_Defaults(ZDT_Emm_t *zdt);

// ============================================================================
// 读取参数命令
// ============================================================================
/**
 * @brief  读取固件版本
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   返回数据: [固件版本号, 硬件版本号]
 */
//void Emm_V5_Read_Version(ZDT_Emm_t *zdt);

/**
 * @brief  读取相电阻和相电感
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   返回数据: [相电阻(mΩ), 相电感(uH)]
 */
//void Emm_V5_Read_RL(ZDT_Emm_t *zdt);

/**
 * @brief  读取位置环PID参数
 * @param  zdt: 电机实例指针
 * @retval None
 */
//void Emm_V5_Read_PID(ZDT_Emm_t *zdt);

/**
 * @brief  读取系统状态参数 (综合查询)
 * @param  zdt: 电机实例指针
 * @retval None
 * @note   返回: 总线电压、电流、编码器值、位置、转速、误差、状态标志等
 */
void Emm_V5_Read_SystemStatus(ZDT_Emm_t *zdt);

/**
 * @brief  读取单个系统参数
 * @param  zdt: 电机实例指针
 * @param  s: 参数类型
 * @retval None
 */
void Emm_V5_Read_Sys_Param(ZDT_Emm_t *zdt, SysParams_Emm_t s);

/**
 * @brief  读取驱动配置参数
 * @param  zdt: 电机实例指针
 * @retval None
 */
//void Emm_V5_Read_DriverConfig(ZDT_Emm_t *zdt);

/**
 * @brief  读取原点回零参数
 * @param  zdt: 电机实例指针
 * @retval None
 */
//void Emm_V5_Read_HomeParams(ZDT_Emm_t *zdt);

// ============================================================================
// 修改参数命令
// ============================================================================
/**
 * @brief  修改细分步数
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  microstep: 细分值 (1-256)
 * @retval None
 */
//void Emm_V5_Set_Microstep(ZDT_Emm_t *zdt, bool store, uint8_t microstep);

/**
 * @brief  修改电机地址
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  new_addr: 新地址 (1-255)
 * @retval None
 */
//void Emm_V5_Set_Address(ZDT_Emm_t *zdt, bool store, uint8_t new_addr);

/**
 * @brief  切换开环/闭环模式
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  mode: 模式 (MOTOR_MODE_OPEN/MOTOR_MODE_FOC)
 * @retval None
 */
//void Emm_V5_Set_MotorMode(ZDT_Emm_t *zdt, bool store, Emm_MotorMode mode);

/**
 * @brief  修改开环模式工作电流
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  current: 电流值 (mA, 范围200-3000)
 * @retval None
 */
//void Emm_V5_Set_OpenCurrent(ZDT_Emm_t *zdt, bool store, uint16_t current);

/**
 * @brief  修改驱动配置参数
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  cfg: 配置参数结构体
 * @retval None
 */
//void Emm_V5_Set_DriverConfig(ZDT_Emm_t *zdt, bool store, const Emm_DriverConfig *cfg);

/**
 * @brief  修改位置环PID参数
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  pid: PID参数结构体
 * @retval None
 */
//void Emm_V5_Set_PID(ZDT_Emm_t *zdt, bool store, const Emm_PIDParams *pid);

/**
 * @brief  修改原点回零参数
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  params: 回零参数结构体
 * @retval None
 */
//void Emm_V5_Set_HomeParams(ZDT_Emm_t *zdt, bool store, const Emm_HomeParams *params);

/**
 * @brief  存储一组速度模式参数 (上电自动运行)
 * @param  zdt: 电机实例指针
 * @param  dir: 方向
 * @param  speed: 速度 (RPM)
 * @param  accel: 加速度档位
 * @param  en_ctrl: 是否使能En引脚控制启停
 * @retval None
 * @note   存储后每次上电自动以设定参数运行
 */
// void Emm_V5_Store_VelocityParams(ZDT_Emm_t *zdt, Emm_Direction dir, uint16_t speed, 
//                                   uint8_t accel, bool en_ctrl);

/**
 * @brief  修改通讯控制的输入速度是否缩小10倍
 * @param  zdt: 电机实例指针
 * @param  store: 是否存储到芯片
 * @param  enable: true=使能, 速度缩小10倍输入 (精确到0.1RPM)
 * @retval None
 */
//void Emm_V5_Set_VelocityScale(ZDT_Emm_t *zdt, bool store, bool enable);

// ============================================================================
// 辅助函数
// ============================================================================
/**
 * @brief  计算XOR校验
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验值
 */
//uint8_t Emm_Calc_XOR(const uint8_t *data, uint8_t len);

/**
 * @brief  计算CRC-8校验
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @retval 校验值
 */
//uint8_t Emm_Calc_CRC8(const uint8_t *data, uint8_t len);

/**
 * @brief  解析接收到的数据
 * @param  zdt: 电机实例指针
 * @param  rx_data: 接收数据指针
 * @param  len: 数据长度
 * @retval 解析成功返回true
 */
bool Emm_Parse_Response(ZDT_Emm_t *zdt, const uint8_t *rx_data, uint8_t len);

/**
 * @brief  将编码器值转换为角度
 * @param  encoder: 编码器值 (0-65535表示一圈)
 * @retval 角度值 (度)
 */
float Emm_Encoder_To_Angle(int32_t encoder);

/**
 * @brief  将角度转换为编码器值
 * @param  angle: 角度值 (度)
 * @retval 编码器值
 */
int32_t Emm_Angle_To_Encoder(float angle);

/**
 * @brief  将脉冲数转换为圈数
 * @param  pulses: 脉冲数
 * @retval 圈数
 * @note   使用配置的电机参数 (1.8°电机, 16细分)
 */
float Emm_Pulses_To_Rounds(int32_t pulses);

/**
 * @brief  将圈数转换为脉冲数
 * @param  rounds: 圈数
 * @retval 脉冲数
 * @note   使用配置的电机参数 (1.8°电机, 16细分)
 */
int32_t Emm_Rounds_To_Pulses(float rounds);

/**
 * @brief  将角度转换为圈数
 * @param  angle_deg: 角度 (度)
 * @retval 圈数
 */
float Emm_Angle_To_Rounds(float angle_deg);

/**
 * @brief  将圈数转换为角度
 * @param  rounds: 圈数
 * @retval 角度 (度)
 */
float Emm_Rounds_To_Angle(float rounds);

/**
 * @brief  将脉冲数转换为角度
 * @param  pulses: 脉冲数
 * @retval 角度 (度)
 */
float Emm_Pulses_To_Angle(int32_t pulses);

/**
 * @brief  将角度转换为脉冲数
 * @param  angle_deg: 角度 (度)
 * @retval 脉冲数
 */
int32_t Emm_Angle_To_Pulses(float angle_deg);

// ============================================================================
// 实用控制函数 (电机参数: 1.8°电机, 16细分, CW正方向)
// ============================================================================
/**
 * @brief  电机以指定速度持续转动
 * @param  zdt: 电机实例指针
 * @param  rpm: 转速 (RPM, 正值=CW, 负值=CCW)
 * @retval None
 */
void Emm_Move_Continuous(ZDT_Emm_t *zdt, int16_t rpm);

/**
 * @brief  电机转动指定角度
 * @param  zdt: 电机实例指针
 * @param  angle_deg: 角度 (度, 正值=CW, 负值=CCW)
 * @param  speed_rpm: 速度 (RPM)
 * @retval None
 */
void Emm_Move_Angle(ZDT_Emm_t *zdt, float angle_deg, uint16_t speed_rpm);

/**
 * @brief  电机转动指定圈数
 * @param  zdt: 电机实例指针
 * @param  rounds: 圈数 (正值=CW, 负值=CCW)
 * @param  speed_rpm: 速度 (RPM)
 * @retval None
 */
void Emm_Move_Rounds(ZDT_Emm_t *zdt, float rounds, uint16_t speed_rpm);

/**
 * @brief  电机移动到绝对位置 (单圈范围)
 * @param  zdt: 电机实例指针
 * @param  angle_deg: 目标角度 (度, 0-360)
 * @param  speed_rpm: 速度 (RPM)
 * @retval None
 * @note   需要先设置零点位置，零点为0°位置
 */
void Emm_Move_To_Absolute(ZDT_Emm_t *zdt, float angle_deg, uint16_t speed_rpm);
bool Emm_Read_vel_pos(ZDT_Emm_t *zdt, const uint8_t *rx_data, uint8_t len);
#ifdef __cplusplus
}
#endif

#endif /* ZDT_EMM_H */
