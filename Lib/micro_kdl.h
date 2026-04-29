#ifndef MICRO_KDL_H
#define MICRO_KDL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} Mikdl_Vector3;

typedef struct {
    float m[3][3];
} Mikdl_Matrix3;

typedef struct {
    float l1;// 大臂长度
    float l2;// 小臂长度
    float m1;// 大臂质量
    float m2;// 小臂质量
	// 质心位置（相对于连杆近端）
    float c1;
    float c2;
    float gravity;
    Mikdl_Vector3 gravity_dir;
    float dls_lambda;
    float ik_tolerance;
    int ik_max_iterations;
} Mikdl_Robot;

typedef struct {
    float max_vel;
    float max_acc;
    float t1;
    float t2;
    float duration;
    float p_start;
    float p_end;
} Mikdl_TrapProfile;

#define MIKDL_SUCCESS 0
#define MIKDL_ERR_INVALID_ARGUMENT (-1)
#define MIKDL_ERR_NO_CONVERGENCE (-2)

#define MIKDL_DEFAULT_GRAVITY 9.81f
#define MIKDL_DEFAULT_DLS_LAMBDA 0.01f
#define MIKDL_DEFAULT_IK_TOLERANCE 1e-4f
#define MIKDL_DEFAULT_IK_MAX_ITERATIONS 32

#define MIKDL_PI 3.14159265358979323846f

void mikdl_robot_default(Mikdl_Robot* robot);

void mikdl_forward_kinematics(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* q,
    Mikdl_Vector3* p_out);

int mikdl_inverse_kinematics(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* p_des,
    const Mikdl_Vector3* q_init,
    Mikdl_Vector3* q_out);

int mikdl_cart_to_jnt_velocity(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* q,
    const Mikdl_Vector3* v_des,
    Mikdl_Vector3* dq_out);

int mikdl_cart_to_jnt_acceleration(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* q,
    const Mikdl_Vector3* dq,
    const Mikdl_Vector3* a_des,
    Mikdl_Vector3* ddq_out);

void mikdl_inverse_dynamics(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* q,
    const Mikdl_Vector3* dq,
    const Mikdl_Vector3* ddq,
    const Mikdl_Vector3* f_ext,
    Mikdl_Vector3* tau_out);

int mikdl_cart_to_joint_control(
    const Mikdl_Robot* robot,
    const Mikdl_Vector3* p_des,
    const Mikdl_Vector3* v_des,
    const Mikdl_Vector3* a_des,
    const Mikdl_Vector3* f_ext,
    const Mikdl_Vector3* q_init,
    Mikdl_Vector3* q_out,
    Mikdl_Vector3* dq_out,
    Mikdl_Vector3* tau_out);

void mikdl_trap_init(
    Mikdl_TrapProfile* prof,
    float start,
    float end,
    float max_vel,
    float max_acc);

void mikdl_trap_get(
    Mikdl_TrapProfile* prof,
    float t,
    float* p,
    float* v,
    float* a);

#ifdef __cplusplus
}
#endif

#endif