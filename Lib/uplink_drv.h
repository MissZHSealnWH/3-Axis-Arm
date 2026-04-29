#ifndef UPLINK_DRV_H
#define UPLINK_DRV_H

#include <stdint.h>

#define UPLINK_FRAME_LEN 14   /* 2(帧头) + 3*4(float) */

/* 三个轴的命令数据 */
typedef struct {
    float x;
    float y;
    float z;
} UplinkCommand;

void Uplink_Init(void);
int Uplink_GetCommand(UplinkCommand *cmd);  /* 返回 1 有新命令，0 无 */
#endif