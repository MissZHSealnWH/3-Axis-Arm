#ifndef UPLINK_DRV_H
#define UPLINK_DRV_H

#include <stdint.h>

#define UPLINK_FRAME_LEN 8   // 2(包头包尾) + 3*1(int16_t) + 3*1
#define FRAME_START 0x41     // 包头
#define FRAME_LAST 0x42      // 包尾

// 三个轴的命令数据 (int16) + 其他待扩展
volatile typedef struct {
	int8_t x; // 上下
	int8_t y; // 左右
	int8_t z; // 摄像头前方 沿小臂向前（平行小臂）
	int8_t one;
	int8_t two;
	int8_t three;
} UplinkCommand;

void Uplink_Init(void);
int Uplink_GetCommand(UplinkCommand *cmd);  // 返回1有新命令，0无 

#endif

