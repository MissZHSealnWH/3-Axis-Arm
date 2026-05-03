#include "micro_kdl.h"

#include <math.h>
#include <string.h>

// 绝对值
static float mikdl_fabsf(float value) {
    return value < 0.0f ? -value : value;
}

//限制value最大最小范围
static float mikdl_clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 返回三维向量的长度
static float mikdl_vec_norm(const Mikdl_Vector3* value) {
    return sqrtf(value->x * value->x + value->y * value->y + value->z * value->z);
}

// 向量加
static Mikdl_Vector3 mikdl_vec_add(Mikdl_Vector3 a, Mikdl_Vector3 b) {
    Mikdl_Vector3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

// 向量减
static Mikdl_Vector3 mikdl_vec_sub(Mikdl_Vector3 a, Mikdl_Vector3 b) {
    Mikdl_Vector3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

// 向量数乘
static Mikdl_Vector3 mikdl_vec_scale(Mikdl_Vector3 value, float scale) {
    Mikdl_Vector3 result = {value.x * scale, value.y * scale, value.z * scale};
    return result;
}

// 向量点积
static float mikdl_vec_dot(Mikdl_Vector3 a, Mikdl_Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 矩阵加分法
static Mikdl_Matrix3 mikdl_mat_add(Mikdl_Matrix3 a, Mikdl_Matrix3 b) {
    Mikdl_Matrix3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = a.m[i][j] + b.m[i][j];
        }
    }
    return result;
}

// 矩阵数乘
static Mikdl_Matrix3 mikdl_mat_scale(Mikdl_Matrix3 matrix, float scale) {
    Mikdl_Matrix3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = matrix.m[i][j] * scale;
        }
    }
    return result;
}

// 矩阵乘法
static Mikdl_Matrix3 mikdl_mat_mul(Mikdl_Matrix3 a, Mikdl_Matrix3 b) {
    Mikdl_Matrix3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                result.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }
    return result;
}

// 矩阵转置
static Mikdl_Matrix3 mikdl_mat_transpose(Mikdl_Matrix3 matrix) {
    Mikdl_Matrix3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = matrix.m[j][i];
        }
    }
    return result;
}

// 矩阵与列向量的乘积
static Mikdl_Vector3 mikdl_mat_mul_vec(const Mikdl_Matrix3* matrix, Mikdl_Vector3 value) {
    Mikdl_Vector3 result;
    result.x = matrix->m[0][0] * value.x + matrix->m[0][1] * value.y + matrix->m[0][2] * value.z;
    result.y = matrix->m[1][0] * value.x + matrix->m[1][1] * value.y + matrix->m[1][2] * value.z;
    result.z = matrix->m[2][0] * value.x + matrix->m[2][1] * value.y + matrix->m[2][2] * value.z;
    return result;
}

// 计算矩阵的行列式
static float mikdl_mat_det(Mikdl_Matrix3 matrix) {
    return matrix.m[0][0] * (matrix.m[1][1] * matrix.m[2][2] - matrix.m[1][2] * matrix.m[2][1]) -
           matrix.m[0][1] * (matrix.m[1][0] * matrix.m[2][2] - matrix.m[1][2] * matrix.m[2][0]) +
           matrix.m[0][2] * (matrix.m[1][0] * matrix.m[2][1] - matrix.m[1][1] * matrix.m[2][0]);
}

// 求逆矩阵
static int mikdl_mat_inverse(Mikdl_Matrix3 matrix, Mikdl_Matrix3* inverse) {
    float det = mikdl_mat_det(matrix);
    if (mikdl_fabsf(det) < 1e-9f) {
        return 0;
    }

    float inv_det = 1.0f / det;

    inverse->m[0][0] = (matrix.m[1][1] * matrix.m[2][2] - matrix.m[1][2] * matrix.m[2][1]) * inv_det;
    inverse->m[0][1] = (matrix.m[0][2] * matrix.m[2][1] - matrix.m[0][1] * matrix.m[2][2]) * inv_det;
    inverse->m[0][2] = (matrix.m[0][1] * matrix.m[1][2] - matrix.m[0][2] * matrix.m[1][1]) * inv_det;
    inverse->m[1][0] = (matrix.m[1][2] * matrix.m[2][0] - matrix.m[1][0] * matrix.m[2][2]) * inv_det;
    inverse->m[1][1] = (matrix.m[0][0] * matrix.m[2][2] - matrix.m[0][2] * matrix.m[2][0]) * inv_det;
    inverse->m[1][2] = (matrix.m[0][2] * matrix.m[1][0] - matrix.m[0][0] * matrix.m[1][2]) * inv_det;
    inverse->m[2][0] = (matrix.m[1][0] * matrix.m[2][1] - matrix.m[1][1] * matrix.m[2][0]) * inv_det;
    inverse->m[2][1] = (matrix.m[0][1] * matrix.m[2][0] - matrix.m[0][0] * matrix.m[2][1]) * inv_det;
    inverse->m[2][2] = (matrix.m[0][0] * matrix.m[1][1] - matrix.m[0][1] * matrix.m[1][0]) * inv_det;
    return 1;
}

// 缺少参数获取函数
static float mikdl_default_or_value(float value, float default_value) {
    return value != 0.0f ? value : default_value;
}

// 从机器人参数中提取重力方向单位向量：若未设置（全 0），默认 (0,0,-1)，并归一化。
static Mikdl_Vector3 mikdl_default_gravity_dir(const Mikdl_Robot* robot) {
    Mikdl_Vector3 dir = robot->gravity_dir;
    if (dir.x == 0.0f && dir.y == 0.0f && dir.z == 0.0f) {
        dir.x = 0.0f;
        dir.y = 0.0f;
        dir.z = -1.0f;
    }

    float norm = mikdl_vec_norm(&dir);
    if (norm < 1e-9f) {
        dir.x = 0.0f;
        dir.y = 0.0f;
        dir.z = -1.0f;
        return dir;
    }

    return mikdl_vec_scale(dir, 1.0f / norm);
}

// 获取机器人 DLS 阻尼系数
static float mikdl_default_dls(const Mikdl_Robot* robot) {
    return robot->dls_lambda > 0.0f ? robot->dls_lambda : MIKDL_DEFAULT_DLS_LAMBDA;
}

// IK 误差容限
static float mikdl_default_tol(const Mikdl_Robot* robot) {
    return robot->ik_tolerance > 0.0f ? robot->ik_tolerance : MIKDL_DEFAULT_IK_TOLERANCE;
}

// 最大迭代次数
static int mikdl_default_max_iter(const Mikdl_Robot* robot) {
    return robot->ik_max_iterations > 0 ? robot->ik_max_iterations : MIKDL_DEFAULT_IK_MAX_ITERATIONS;
}

// 正运动学计算
static void mikdl_compute_fk(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Vector3* p_out) {
    float s0 = sinf(q->x);
    float c0 = cosf(q->x);
    float s1 = sinf(q->y);
    float c1 = cosf(q->y);
    float s12 = sinf(q->y + q->z);
    float c12 = cosf(q->y + q->z);

    float r = robot->l1 * c1 + robot->l2 * c12;
    p_out->x = r * c0;
    p_out->y = r * s0;
    p_out->z = robot->l1 * s1 + robot->l2 * s12;
}

// 计算末端位置相对于关节角的 几何雅可比矩阵（3×3）
static void mikdl_compute_jacobian(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Matrix3* jacobian) {
    float s0 = sinf(q->x);
    float c0 = cosf(q->x);
    float s1 = sinf(q->y);
    float c1 = cosf(q->y);
    float s12 = sinf(q->y + q->z);
    float c12 = cosf(q->y + q->z);

    float r = robot->l1 * c1 + robot->l2 * c12;
    float a = -(robot->l1 * s1 + robot->l2 * s12);
    float b = -robot->l2 * s12;
    float dzdq1 = robot->l1 * c1 + robot->l2 * c12;
    float dzdq2 = robot->l2 * c12;

    jacobian->m[0][0] = -r * s0;
    jacobian->m[0][1] = a * c0;
    jacobian->m[0][2] = b * c0;

    jacobian->m[1][0] = r * c0;
    jacobian->m[1][1] = a * s0;
    jacobian->m[1][2] = b * s0;

    jacobian->m[2][0] = 0.0f;
    jacobian->m[2][1] = dzdq1;
    jacobian->m[2][2] = dzdq2;
}

// 计算 连杆质心位置 相对于关节角的雅可比矩阵（线速度部分）
static void mikdl_compute_com_jacobian_1(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Matrix3* jacobian) {
    float s0 = sinf(q->x);
    float c0 = cosf(q->x);
    float s1 = sinf(q->y);
    float c1 = cosf(q->y);

    jacobian->m[0][0] = -robot->c1 * c1 * s0;
    jacobian->m[0][1] = -robot->c1 * s1 * c0;
    jacobian->m[0][2] = 0.0f;

    jacobian->m[1][0] = robot->c1 * c1 * c0;
    jacobian->m[1][1] = -robot->c1 * s1 * s0;
    jacobian->m[1][2] = 0.0f;

    jacobian->m[2][0] = 0.0f;
    jacobian->m[2][1] = robot->c1 * c1;
    jacobian->m[2][2] = 0.0f;
}

// 计算 连杆质心位置 相对于关节角的雅可比矩阵（线速度部分）
static void mikdl_compute_com_jacobian_2(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Matrix3* jacobian) {
    float s0 = sinf(q->x);
    float c0 = cosf(q->x);
    float s1 = sinf(q->y);
    float c1 = cosf(q->y);
    float s12 = sinf(q->y + q->z);
    float c12 = cosf(q->y + q->z);

    float r = robot->l1 * c1 + robot->c2 * c12;
    float dr_dq1 = -robot->l1 * s1 - robot->c2 * s12;
    float dr_dq2 = -robot->c2 * s12;
    float dz_dq1 = robot->l1 * c1 + robot->c2 * c12;
    float dz_dq2 = robot->c2 * c12;

    jacobian->m[0][0] = -r * s0;
    jacobian->m[0][1] = dr_dq1 * c0;
    jacobian->m[0][2] = dr_dq2 * c0;

    jacobian->m[1][0] = r * c0;
    jacobian->m[1][1] = dr_dq1 * s0;
    jacobian->m[1][2] = dr_dq2 * s0;

    jacobian->m[2][0] = 0.0f;
    jacobian->m[2][1] = dz_dq1;
    jacobian->m[2][2] = dz_dq2;
}

// 计算关节空间的 质量矩阵 M（3×3）
static void mikdl_compute_mass_matrix(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Matrix3* mass_matrix) {
    Mikdl_Matrix3 j1;
    Mikdl_Matrix3 j2;
    Mikdl_Matrix3 j1_t;
    Mikdl_Matrix3 j2_t;
    Mikdl_Matrix3 jtj1;
    Mikdl_Matrix3 jtj2;

    mikdl_compute_com_jacobian_1(robot, q, &j1);
    mikdl_compute_com_jacobian_2(robot, q, &j2);

    j1_t = mikdl_mat_transpose(j1);
    j2_t = mikdl_mat_transpose(j2);
    jtj1 = mikdl_mat_mul(j1_t, j1);
    jtj2 = mikdl_mat_mul(j2_t, j2);

    *mass_matrix = mikdl_mat_add(mikdl_mat_scale(jtj1, robot->m1), mikdl_mat_scale(jtj2, robot->m2));
}

// 计算 重力项 G（3×1 向量）
static void mikdl_compute_gravity_vector(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Vector3* gravity) {
    Mikdl_Matrix3 j1;
    Mikdl_Matrix3 j2;
    Mikdl_Vector3 gravity_dir = mikdl_default_gravity_dir(robot);
    Mikdl_Vector3 g_vec = mikdl_vec_scale(gravity_dir, mikdl_default_or_value(robot->gravity, MIKDL_DEFAULT_GRAVITY));

    mikdl_compute_com_jacobian_1(robot, q, &j1);
    mikdl_compute_com_jacobian_2(robot, q, &j2);

    Mikdl_Vector3 f1 = mikdl_vec_scale(g_vec, -robot->m1);
    Mikdl_Vector3 f2 = mikdl_vec_scale(g_vec, -robot->m2);

    gravity->x = mikdl_vec_dot((Mikdl_Vector3){j1.m[0][0], j1.m[1][0], j1.m[2][0]}, f1) + mikdl_vec_dot((Mikdl_Vector3){j2.m[0][0], j2.m[1][0], j2.m[2][0]}, f2);
    gravity->y = mikdl_vec_dot((Mikdl_Vector3){j1.m[0][1], j1.m[1][1], j1.m[2][1]}, f1) + mikdl_vec_dot((Mikdl_Vector3){j2.m[0][1], j2.m[1][1], j2.m[2][1]}, f2);
    gravity->z = mikdl_vec_dot((Mikdl_Vector3){j1.m[0][2], j1.m[1][2], j1.m[2][2]}, f1) + mikdl_vec_dot((Mikdl_Vector3){j2.m[0][2], j2.m[1][2], j2.m[2][2]}, f2);
}

// 计算科里奥利和离心力项 
static void mikdl_compute_coriolis_vector(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* dq, Mikdl_Vector3* coriolis) {
    const float eps = 1e-4f;
    Mikdl_Matrix3 m_plus;
    Mikdl_Matrix3 m_minus;
    Mikdl_Matrix3 dmdq[3];
    Mikdl_Vector3 q_shift;

    for (int axis = 0; axis < 3; ++axis) {
        q_shift = *q;
        if (axis == 0) {
            q_shift.x += eps;
        } else if (axis == 1) {
            q_shift.y += eps;
        } else {
            q_shift.z += eps;
        }
        mikdl_compute_mass_matrix(robot, &q_shift, &m_plus);

        q_shift = *q;
        if (axis == 0) {
            q_shift.x -= eps;
        } else if (axis == 1) {
            q_shift.y -= eps;
        } else {
            q_shift.z -= eps;
        }
        mikdl_compute_mass_matrix(robot, &q_shift, &m_minus);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                dmdq[axis].m[i][j] = (m_plus.m[i][j] - m_minus.m[i][j]) / (2.0f * eps);
            }
        }
    }

    float dq_vec[3] = {dq->x, dq->y, dq->z};
    float result[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                float gamma = 0.5f * (dmdq[k].m[i][j] + dmdq[j].m[i][k] - dmdq[i].m[j][k]);
                result[i] += gamma * dq_vec[j] * dq_vec[k];
            }
        }
    }

    coriolis->x = result[0];
    coriolis->y = result[1];
    coriolis->z = result[2];
}

// 计算 外力（末端力）映射到关节力矩
static void mikdl_compute_external_torque(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* f_ext, Mikdl_Vector3* tau_ext) {
    Mikdl_Matrix3 jacobian;
    Mikdl_Matrix3 j_t;

    mikdl_compute_jacobian(robot, q, &jacobian);
    j_t = mikdl_mat_transpose(jacobian);
    *tau_ext = mikdl_mat_mul_vec(&j_t, *f_ext);
}

// 雅可比的时间导数乘以关节速度，用于加速度映射
static void mikdl_compute_jdot_times_dq(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* dq, Mikdl_Vector3* jdot_dq) {
    float s0 = sinf(q->x);
    float c0 = cosf(q->x);
    float s1 = sinf(q->y);
    float c1 = cosf(q->y);
    float s12 = sinf(q->y + q->z);
    float c12 = cosf(q->y + q->z);

    float r = robot->l1 * c1 + robot->l2 * c12;
    float r_dot = -robot->l1 * s1 * dq->y - robot->l2 * s12 * (dq->y + dq->z);
    float r_ddot_nl = -robot->l1 * c1 * dq->y * dq->y - robot->l2 * c12 * (dq->y + dq->z) * (dq->y + dq->z);

    jdot_dq->x = r_ddot_nl * c0 - 2.0f * r_dot * s0 * dq->x - r * c0 * dq->x * dq->x;
    jdot_dq->y = r_ddot_nl * s0 + 2.0f * r_dot * c0 * dq->x - r * s0 * dq->x * dq->x;
    jdot_dq->z = -robot->l1 * s1 * dq->y * dq->y - robot->l2 * s12 * (dq->y + dq->z) * (dq->y + dq->z);
}

// 阻尼最小二乘法
static int mikdl_solve_dls(const Mikdl_Matrix3* jacobian, const Mikdl_Vector3* rhs, float lambda, Mikdl_Vector3* x_out) {
    Mikdl_Matrix3 jj_t = mikdl_mat_mul(*jacobian, mikdl_mat_transpose(*jacobian));
    Mikdl_Matrix3 regularized = jj_t;
    Mikdl_Matrix3 inverse;
    Mikdl_Matrix3 j_t = mikdl_mat_transpose(*jacobian);
    Mikdl_Vector3 y;

    regularized.m[0][0] += lambda * lambda;
    regularized.m[1][1] += lambda * lambda;
    regularized.m[2][2] += lambda * lambda;

    if (!mikdl_mat_inverse(regularized, &inverse)) {
        return 0;
    }

    y = mikdl_mat_mul_vec(&inverse, *rhs);
    *x_out = mikdl_mat_mul_vec(&j_t, y);
    return 1;
}

// 根据目标位置生成逆运动学的初始关节角猜测
static Mikdl_Vector3 mikdl_initial_guess(const Mikdl_Robot* robot, const Mikdl_Vector3* p_des, const Mikdl_Vector3* q_init) {
    if (q_init != NULL) {
        return *q_init;
    }

    float rho = sqrtf(p_des->x * p_des->x + p_des->y * p_des->y);
    float q0 = atan2f(p_des->y, p_des->x);
    float l1 = robot->l1;
    float l2 = robot->l2;
    float d = sqrtf(rho * rho + p_des->z * p_des->z);
    float cos_q2 = mikdl_clampf((d * d - l1 * l1 - l2 * l2) / (2.0f * l1 * l2), -1.0f, 1.0f);
    float sin_q2 = sqrtf(mikdl_clampf(1.0f - cos_q2 * cos_q2, 0.0f, 1.0f));
    float q2 = atan2f(sin_q2, cos_q2);
    float q1 = atan2f(p_des->z, rho) - atan2f(l2 * sin_q2, l1 + l2 * cos_q2);

    Mikdl_Vector3 guess = {q0, q1, q2};
    return guess;
}

// 判断机器人参数是否有效：指针非空且连杆长度 > 0
static int mikdl_validate_robot(const Mikdl_Robot* robot) {
    return robot != NULL && robot->l1 > 0.0f && robot->l2 > 0.0f;
}

// 初始化 Mikdl_Robot 结构体的默认值，清零后设置重力、阻尼系数、误差容限、迭代次数为宏定义值。连杆长度和质量根据实际机器人填写
void mikdl_robot_default(Mikdl_Robot* robot) {
    if (robot == NULL) {
        return;
    }

    memset(robot, 0, sizeof(*robot));
    robot->gravity = MIKDL_DEFAULT_GRAVITY;
    robot->gravity_dir.x = 0.0f;
    robot->gravity_dir.y = 0.0f;
    robot->gravity_dir.z = -1.0f;
    robot->dls_lambda = MIKDL_DEFAULT_DLS_LAMBDA;
    robot->ik_tolerance = MIKDL_DEFAULT_IK_TOLERANCE;
    robot->ik_max_iterations = MIKDL_DEFAULT_IK_MAX_ITERATIONS;
}

/**
 * @brief 正运动学：关节位置 -> 末端位置
 * @param robot 机器人模型
 * @param q 关节位置
 *        q.x: 底座旋转角
 *        q.y: 大臂俯仰角
 *        q.z: 小臂俯仰角
 * @param p_out 输出：末端位置 (x,y,z)
 */
void mikdl_forward_kinematics(const Mikdl_Robot* robot, const Mikdl_Vector3* q, Mikdl_Vector3* p_out) {
    if (!mikdl_validate_robot(robot) || q == NULL || p_out == NULL) {
        return;
    }

    mikdl_compute_fk(robot, q, p_out);
}

/**
 * @brief 逆运动学：末端位置 -> 关节位置
 * @param robot 机器人模型
 * @param p_des 末端期望位置 (x,y,z)
 * @param q_init 迭代初始值(建议传入上一时刻的关节角度，保证收敛)
 * @param q_out 输出：各关节位置
 * @return 0:成功, 非0:失败(未收敛)
 */
int mikdl_inverse_kinematics(const Mikdl_Robot* robot, const Mikdl_Vector3* p_des, const Mikdl_Vector3* q_init, Mikdl_Vector3* q_out) {
    if (!mikdl_validate_robot(robot) || p_des == NULL || q_out == NULL) {
        return MIKDL_ERR_INVALID_ARGUMENT;
    }

    Mikdl_Vector3 q = mikdl_initial_guess(robot, p_des, q_init);
    float tol = mikdl_default_tol(robot);
    float lambda = mikdl_default_dls(robot);
    int max_iter = mikdl_default_max_iter(robot);

    for (int iter = 0; iter < max_iter; ++iter) {
        Mikdl_Vector3 p_now;
        Mikdl_Vector3 error;
        Mikdl_Matrix3 jacobian;
        Mikdl_Vector3 dq;

        mikdl_compute_fk(robot, &q, &p_now);
        error = mikdl_vec_sub(*p_des, p_now);

        if (mikdl_vec_norm(&error) <= tol) {
            *q_out = q;
            return MIKDL_SUCCESS;
        }

        mikdl_compute_jacobian(robot, &q, &jacobian);
        if (!mikdl_solve_dls(&jacobian, &error, lambda, &dq)) {
            return MIKDL_ERR_NO_CONVERGENCE;
        }

        q = mikdl_vec_add(q, dq);

        if (!isfinite(q.x) || !isfinite(q.y) || !isfinite(q.z)) {
            return MIKDL_ERR_NO_CONVERGENCE;
        }
    }

    {
        Mikdl_Vector3 p_now;
        Mikdl_Vector3 error;
        mikdl_compute_fk(robot, &q, &p_now);
        error = mikdl_vec_sub(*p_des, p_now);
        if (mikdl_vec_norm(&error) <= tol * 10.0f) {
            *q_out = q;
            return MIKDL_SUCCESS;
        }
    }

    return MIKDL_ERR_NO_CONVERGENCE;
}

/**
 * @brief 速度映射：末端速度 -> 关节速度
 * @param robot 机器人模型
 * @param q 当前关节位置
 * @param v_des 末端期望速度
 * @param dq_out 输出：各关节速度
 * @return 0:成功
 */
int mikdl_cart_to_jnt_velocity(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* v_des, Mikdl_Vector3* dq_out) {
    if (!mikdl_validate_robot(robot) || q == NULL || v_des == NULL || dq_out == NULL) {
        return MIKDL_ERR_INVALID_ARGUMENT;
    }

    Mikdl_Matrix3 jacobian;
    mikdl_compute_jacobian(robot, q, &jacobian);
    if (!mikdl_solve_dls(&jacobian, v_des, mikdl_default_dls(robot), dq_out)) {
        return MIKDL_ERR_NO_CONVERGENCE;
    }

    return MIKDL_SUCCESS;
}

/**
 * @brief 加速度映射：末端加速度 -> 关节加速度
 * @param robot 机器人模型
 * @param q 当前关节位置
 * @param dq 当前关节速度
 * @param a_des 末端期望加速度
 * @param ddq_out 输出：各关节加速度
 * @return 0:成功
 */
int mikdl_cart_to_jnt_acceleration(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* dq, const Mikdl_Vector3* a_des, Mikdl_Vector3* ddq_out) {
    if (!mikdl_validate_robot(robot) || q == NULL || dq == NULL || a_des == NULL || ddq_out == NULL) {
        return MIKDL_ERR_INVALID_ARGUMENT;
    }

    Mikdl_Matrix3 jacobian;
    Mikdl_Vector3 jdot_dq;
    Mikdl_Vector3 rhs;

    mikdl_compute_jacobian(robot, q, &jacobian);
    mikdl_compute_jdot_times_dq(robot, q, dq, &jdot_dq);
    rhs = mikdl_vec_sub(*a_des, jdot_dq);

    if (!mikdl_solve_dls(&jacobian, &rhs, mikdl_default_dls(robot), ddq_out)) {
        return MIKDL_ERR_NO_CONVERGENCE;
    }

    return MIKDL_SUCCESS;
}

/**
 * @brief 逆动力学：关节状态 -> 关节力矩
 * @param robot 机器人模型
 * @param q 关节位置
 * @param dq 关节速度
 * @param ddq 关节加速度
 * @param f_ext 末端外力/补偿力(重力补偿、力反馈等)
 * @param tau_out 输出：各关节力矩
 */
void mikdl_inverse_dynamics(const Mikdl_Robot* robot, const Mikdl_Vector3* q, const Mikdl_Vector3* dq, const Mikdl_Vector3* ddq, const Mikdl_Vector3* f_ext, Mikdl_Vector3* tau_out) {
    Mikdl_Matrix3 mass_matrix;
    Mikdl_Vector3 coriolis;
    Mikdl_Vector3 gravity;
    Mikdl_Vector3 external;
    Mikdl_Vector3 inertia;

    if (!mikdl_validate_robot(robot) || q == NULL || dq == NULL || ddq == NULL || f_ext == NULL || tau_out == NULL) {
        return;
    }

    mikdl_compute_mass_matrix(robot, q, &mass_matrix);
    mikdl_compute_coriolis_vector(robot, q, dq, &coriolis);
    mikdl_compute_gravity_vector(robot, q, &gravity);
    mikdl_compute_external_torque(robot, q, f_ext, &external);

    inertia = mikdl_mat_mul_vec(&mass_matrix, *ddq);

    tau_out->x = inertia.x + coriolis.x + gravity.x + external.x;
    tau_out->y = inertia.y + coriolis.y + gravity.y + external.y;
    tau_out->z = inertia.z + coriolis.z + gravity.z + external.z;
}

/**
 * @brief 一体化控制接口：一步解算电机控制量
 * @param robot 机器人模型
 * @param p_des 你规划的末端期望位置
 * @param v_des 你规划的末端期望速度
 * @param a_des 你规划的末端期望加速度
 * @param f_ext 你的末端补偿前馈/反馈力(重力补偿、力反馈等)
 * @param q_init 上一时刻关节角度(逆解迭代起点)
 * @param q_out 输出：电机0/1/2的 位置控制量!
 * @param dq_out 输出：电机0/1/2的 速度控制量!
 * @param tau_out 输出：电机0/1/2的 力矩控制量!
 * @return 0:成功, 非0:失败
 */
int mikdl_cart_to_joint_control(const Mikdl_Robot* robot, const Mikdl_Vector3* p_des, const Mikdl_Vector3* v_des, const Mikdl_Vector3* a_des, const Mikdl_Vector3* f_ext, const Mikdl_Vector3* q_init, Mikdl_Vector3* q_out, Mikdl_Vector3* dq_out, Mikdl_Vector3* tau_out) {
    Mikdl_Vector3 q;
    Mikdl_Vector3 dq;
    Mikdl_Vector3 ddq;
    int status;

    if (!mikdl_validate_robot(robot) || p_des == NULL || v_des == NULL || a_des == NULL || f_ext == NULL || q_init == NULL || q_out == NULL || dq_out == NULL || tau_out == NULL) {
        return MIKDL_ERR_INVALID_ARGUMENT;
    }

    status = mikdl_inverse_kinematics(robot, p_des, q_init, &q);
    if (status != MIKDL_SUCCESS) {
        memset(q_out, 0, sizeof(*q_out));
        memset(dq_out, 0, sizeof(*dq_out));
        memset(tau_out, 0, sizeof(*tau_out));
        return status;
    }

    status = mikdl_cart_to_jnt_velocity(robot, &q, v_des, &dq);
    if (status != MIKDL_SUCCESS) {
        memset(q_out, 0, sizeof(*q_out));
        memset(dq_out, 0, sizeof(*dq_out));
        memset(tau_out, 0, sizeof(*tau_out));
        return status;
    }

    status = mikdl_cart_to_jnt_acceleration(robot, &q, &dq, a_des, &ddq);
    if (status != MIKDL_SUCCESS) {
        memset(q_out, 0, sizeof(*q_out));
        memset(dq_out, 0, sizeof(*dq_out));
        memset(tau_out, 0, sizeof(*tau_out));
        return status;
    }

    mikdl_inverse_dynamics(robot, &q, &dq, &ddq, f_ext, tau_out);
    *q_out = q;
    *dq_out = dq;
    return MIKDL_SUCCESS;
}

// 初始化梯形速度轨迹规划参数
void mikdl_trap_init(Mikdl_TrapProfile* prof, float start, float end, float max_vel, float max_acc) {
    float distance;
    float direction;
    float t_acc;
    float dist_acc;

    if (prof == NULL) {
        return;
    }

    memset(prof, 0, sizeof(*prof));
    prof->max_vel = max_vel > 0.0f ? max_vel : 0.0f;
    prof->max_acc = max_acc > 0.0f ? max_acc : 0.0f;
    prof->p_start = start;
    prof->p_end = end;

    distance = mikdl_fabsf(end - start);
    direction = (end >= start) ? 1.0f : -1.0f;

    if (prof->max_vel <= 0.0f || prof->max_acc <= 0.0f || distance <= 0.0f) {
        prof->t1 = 0.0f;
        prof->t2 = 0.0f;
        prof->duration = 0.0f;
        return;
    }

    t_acc = prof->max_vel / prof->max_acc;
    dist_acc = prof->max_acc * t_acc * t_acc;

    if (distance <= dist_acc) {
        prof->t1 = sqrtf(distance / prof->max_acc);
        prof->t2 = prof->t1;
        prof->duration = 2.0f * prof->t1;
        prof->max_vel = prof->max_acc * prof->t1;
    } else {
        prof->t1 = t_acc;
        prof->t2 = (distance - dist_acc) / prof->max_vel + prof->t1;
        prof->duration = prof->t2 + prof->t1;
    }

    if (direction < 0.0f) {
        prof->max_vel = -prof->max_vel;
    }
}

// 给定时间 t，输出当前时刻的 位置 p、速度 v、加速度 a
void mikdl_trap_get(Mikdl_TrapProfile* prof, float t, float* p, float* v, float* a) {
    float direction;
    float accel;
    float cruise_vel;
    float elapsed;

    if (prof == NULL) {
        return;
    }

    if (p != NULL) {
        *p = prof->p_start;
    }
    if (v != NULL) {
        *v = 0.0f;
    }
    if (a != NULL) {
        *a = 0.0f;
    }

    if (prof->duration <= 0.0f) {
        if (p != NULL) {
            *p = prof->p_end;
        }
        return;
    }

    direction = (prof->p_end >= prof->p_start) ? 1.0f : -1.0f;
    accel = mikdl_fabsf(prof->max_acc);
    cruise_vel = mikdl_fabsf(prof->max_vel);

    if (t <= 0.0f) {
        if (p != NULL) {
            *p = prof->p_start;
        }
        return;
    }

    if (t >= prof->duration) {
        if (p != NULL) {
            *p = prof->p_end;
        }
        return;
    }

    if (t < prof->t1) {
        if (a != NULL) {
            *a = direction * accel;
        }
        if (v != NULL) {
            *v = direction * accel * t;
        }
        if (p != NULL) {
            *p = prof->p_start + direction * 0.5f * accel * t * t;
        }
        return;
    }

    if (t < prof->t2) {
        elapsed = t - prof->t1;
        if (a != NULL) {
            *a = 0.0f;
        }
        if (v != NULL) {
            *v = direction * cruise_vel;
        }
        if (p != NULL) {
            *p = prof->p_start + direction * (0.5f * accel * prof->t1 * prof->t1 + cruise_vel * elapsed);
        }
        return;
    }

    elapsed = t - prof->t2;
    if (a != NULL) {
        *a = -direction * accel;
    }
    if (v != NULL) {
        *v = direction * cruise_vel - direction * accel * elapsed;
    }
    if (p != NULL) {
        float cruise_distance = 0.5f * accel * prof->t1 * prof->t1 + cruise_vel * (prof->t2 - prof->t1);
        float decel_distance = cruise_vel * elapsed - 0.5f * accel * elapsed * elapsed;
        *p = prof->p_start + direction * (cruise_distance + decel_distance);
    }
}


