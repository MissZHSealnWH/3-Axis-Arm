#ifndef UPLINK_DRV_H
#define UPLINK_DRV_H

#include <stdint.h>

#define UPLINK_FRAME_LEN 5   // 2(包头包尾) + 3*1(int8_t)
#define FRAME_START 0xAA     // 包头
#define FRAME_LAST 0x55      // 包尾

// 三个轴的命令数据 (char)
typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} UplinkCommand;

void Uplink_Init(void);
int Uplink_GetCommand(UplinkCommand *cmd);  // 返回1有新命令，0无 

#endif