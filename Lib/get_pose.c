#include "get_pose.h"

// clamp helper
// clamp_double
// 功能：将值 v 限制在 [lo, hi] 区间内并返回。
// 用法：内部数值稳定性检查时使用。
static double clamp_double(double v, double lo, double hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

// internal globals for link lengths (meters)
static double g_L1 = DEFAULT_L1_M;
static double g_L2 = DEFAULT_L2_M;

// set_link_lengths
// 功能：设置库内部默认连杆长度（单位：米）。
// 参数：L1_m - 第一节连杆长度，L2_m - 第二节连杆长度。
// 说明：传入非正值将被忽略以保持当前设置。
void set_link_lengths(double L1_m, double L2_m) {
	if (L1_m > 0) g_L1 = L1_m;
	if (L2_m > 0) g_L2 = L2_m;
}

// compute_inverse_kinematics
// 功能：使用内部连杆长度（通过 set_link_lengths 设置）计算逆运动学。
// 参数：x,y,z - 目标末端位置，单位：米。
// 返回：ArmAngles，motor1/motor2/motor3 为角度（度），valid=0 表示不可达或输入非法。
// 关节定义：motor1 底座偏航（0 指向 +X，正为 +Y），motor2 肩部抬起角，motor3 肘部弯曲角（0=伸直）。
ArmAngles compute_inverse_kinematics(double x, double y, double z) {
	ArmAngles out;
	out.motor1 = out.motor2 = out.motor3 = 0.0;
	out.valid = 0;

	double L1d = g_L1;
	double L2d = g_L2;
	double rx = x;
	double ry = y;
	double rz = z;

	double r = sqrt(rx*rx + ry*ry); // 平面投影距离
	double d = sqrt(r*r + rz*rz);   // 目标到基座关节的直线距离
	// 基本有效性检查
	if (L1d <= 0.0 || L2d <= 0.0) return out;
	if (d <= 1e-12) return out; // 避免除零
	// 可达性检测（最大伸展）
	if (d > (L1d + L2d) + 1e-9) {
		return out; // 超出最大伸展
	}

	// 底座角度（弧度）
	double theta1 = atan2(ry, rx);

	// 肩部到目标连线与水平方向夹角
	double phi = atan2(rz, r);

	// law of cosines: 计算肩部偏角 a，使得肩部角 = phi + a
	double denom_a = 2.0 * L1d * d;
	if (fabs(denom_a) < 1e-12) return out;
	double cos_a = (L1d*L1d + d*d - L2d*L2d) / denom_a;
	cos_a = clamp_double(cos_a, -1.0, 1.0);
	double a = acos(cos_a);
	double shoulder = phi + a; // 弧度

	// 肘部夹角（link之间的夹角），acos 返回 0..pi，完全伸直时为 pi
	double denom_b = 2.0 * L1d * L2d;
	if (fabs(denom_b) < 1e-12) return out;
	double cos_b = (L1d*L1d + L2d*L2d - d*d) / denom_b;
	cos_b = clamp_double(cos_b, -1.0, 1.0);
	double b = acos(cos_b);
	// 我们希望返回的 motor3 为“弯曲角度”，即 0=伸直，对应 internal = pi - b
	double elbow_internal = M_PI - b;

	out.motor1 = RAD2DEG(theta1);
	out.motor2 = RAD2DEG(shoulder);
	out.motor3 = RAD2DEG(elbow_internal);
	out.valid = 1;
	return out;
}
// Inverse kinematics using provided link lengths (meters), returns degrees
// compute_inverse_kinematics_links
// 功能：使用指定连杆长度计算逆运动学，支持 L1 != L2。
// 参数：x,y,z - 目标末端位置（米）；L1_m,L2_m - 连杆长度（米）。
// 返回：ArmAngles 结构，角度单位为度；valid=0 表示不可达或输入非法。
// 备注：函数对极端输入（零长度、目标与基座重合、不可达）做了保护。
ArmAngles compute_inverse_kinematics_links(double x, double y, double z, double L1_m, double L2_m) {
	ArmAngles out;
	out.motor1 = out.motor2 = out.motor3 = 0.0;
	out.valid = 0;

	double L1d = L1_m;
	double L2d = L2_m;
	double rx = x;
	double ry = y;
	double rz = z;

	if (L1d <= 0.0 || L2d <= 0.0) return out;
	double r = sqrt(rx*rx + ry*ry);
	double d = sqrt(r*r + rz*rz);
	if (d <= 1e-12) return out;
	if (d > (L1d + L2d) + 1e-12) return out;
	if (d < fabs(L1d - L2d) - 1e-12) return out; // 内折不可达

	double theta1 = atan2(ry, rx);
	double phi = atan2(rz, r);

	double denom_a = 2.0 * L1d * d;
	if (fabs(denom_a) < 1e-12) return out;
	double cos_a = (L1d*L1d + d*d - L2d*L2d) / denom_a;
	cos_a = clamp_double(cos_a, -1.0, 1.0);
	double a = acos(cos_a);
	double shoulder = phi + a;

	double denom_b = 2.0 * L1d * L2d;
	if (fabs(denom_b) < 1e-12) return out;
	double cos_b = (L1d*L1d + L2d*L2d - d*d) / denom_b;
	cos_b = clamp_double(cos_b, -1.0, 1.0);
	double b = acos(cos_b);
	double elbow_internal = M_PI - b;

	out.motor1 = RAD2DEG(theta1);
	out.motor2 = RAD2DEG(shoulder);
	out.motor3 = RAD2DEG(elbow_internal);
	out.valid = 1;
	return out;
}
