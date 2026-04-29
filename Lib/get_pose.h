#ifndef _GET_POSE_H_
#define _GET_POSE_H_

#include <stdio.h>
#include <math.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 默认连杆长度（单位：米），可通过 set_link_lengths 修改
#define DEFAULT_L1_M 0.10    // 10 cm
#define DEFAULT_L2_M 0.10    // 10 cm

// 角度转换：弧度 <-> 角度（仅在需要时使用）
#define RAD2DEG(r) ((r) * 180.0 / M_PI)
#define DEG2RAD(d) ((d) * M_PI / 180.0)

// 用来返回三个电机角度的结构体(与上位机对接)
typedef struct {
    double motor1;  // 底座旋转角度(度)
    double motor2;  // 大臂摆动角度(度)
    double motor3;  // 小臂摆动角度(度)
    int valid;     // 1=有效解  0=不可达/出错
} ArmAngles;

// 用来返回三个世界坐标xyz的结构体(与上位机对接)
typedef struct {
    double x;
    double y;
    double z;
}Worldpos;


// 逆运动学函数说明：
// - 输入：目标世界坐标 x,y,z (单位：米)
// - 输出：返回 `ArmAngles` 结构体，角度单位为度。
// - 关节约定：
//   * motor1: 底座绕 Z 轴旋转角（0 指向 +X，正方向为 +Y 方向）
//   * motor2: 大臂相对于水平面的抬起角（0 = 水平向外，正值向上）
//   * motor3: 肘部弯曲角，定义为 0 = 伸直，正值为弯曲角度
// - 链接长度默认使用 DEFAULT_L1_M/DEFAULT_L2_M，可用 set_link_lengths 修改（单位：米）。
// compute_inverse_kinematics
// 功能：使用当前设置的连杆长度（通过 set_link_lengths 设置）计算逆运动学。
// 参数：x,y,z - 目标末端在世界坐标系的位置，单位：米。
// 返回：ArmAngles，motor1/motor2/motor3 为角度（度）；若不可达或输入非法，valid=0。
ArmAngles compute_inverse_kinematics(double x, double y, double z);




// 使用指定连杆长度的逆运动学（返回度）
// - x,y,z: 目标位置（米）
// - L1_m, L2_m: 连杆长度（米），可不相等
// 返回：ArmAngles.motor1/2/3 为度，valid 标记是否可达
// compute_inverse_kinematics_links
// 功能：对任意指定连杆长度计算逆运动学（支持 L1 != L2）。
// 参数：x,y,z - 目标末端位置（米）；L1_m,L2_m - 连杆长度（米）。
// 返回：ArmAngles，角度单位为度；valid=0 表示不可达或输入非法。
ArmAngles compute_inverse_kinematics_links(double x, double y, double z, double L1_m, double L2_m);




// 设置连杆长度（单位：米）
// set_link_lengths
// 功能：设置内部默认连杆长度（单位：米）。传入非正值会被忽略。
// 参数：L1_m - 第一节连杆长度（米），L2_m - 第二节连杆长度（米）。
// 返回：无。
void set_link_lengths(double L1_m, double L2_m);



#endif
