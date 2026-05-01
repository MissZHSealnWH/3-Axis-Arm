#ifndef UPLINK_DRV_H
#define UPLINK_DRV_H

#include <stdint.h>

#define UPLINK_FRAME_LEN 8   // 2(包头包尾) + 3*1(int8_t) + 3*1
#define FRAME_START 0xAA     // 包头
#define FRAME_LAST 0x55      // 包尾

// 三个轴的命令数据 (char) + 其他待扩展
typedef struct {
	int8_t x;
	int8_t y;
	int8_t z;
	int8_t one;
	int8_t two;
	int8_t three;
} UplinkCommand;

void Uplink_Init(void);
int Uplink_GetCommand(UplinkCommand *cmd);  // 返回1有新命令，0无 

#endif