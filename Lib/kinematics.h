#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

typedef struct { float x, y, z; } Vec3;

typedef struct {
    float l1, l2;
    float c1, c2;
    float gravity;
    Vec3  gravity_dir;
    float dls_lambda;
    float ik_tolerance;
    int   ik_max_iterations;
} Kinematics;

typedef struct {
    float max_vel;
    float max_acc;
    float t1, t2, duration;
    float p_start, p_end;
} TrapProfile;

#define KIN_OK              0
#define KIN_ERR_UNREACHABLE (-1)

void kin_robot_default(Kinematics* k);
void kin_fk(const Kinematics* k, const Vec3* q, Vec3* p);
int  kin_ik(const Kinematics* k, const Vec3* p_des, const Vec3* q_ref, Vec3* q_out);
void kin_trap_init(TrapProfile* tp, float start, float end, float max_vel, float max_acc);
void kin_trap_get(const TrapProfile* tp, float t, float* p, float* v, float* a);

#endif
