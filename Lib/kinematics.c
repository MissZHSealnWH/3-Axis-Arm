#include "kinematics.h"
#include <math.h>
#include <string.h>

void kin_robot_default(Kinematics* k)
{
    memset(k, 0, sizeof(*k));
    k->gravity = 9.81f;
    k->gravity_dir.z = -1.0f;
    k->dls_lambda = 0.01f;
    k->ik_tolerance = 1e-4f;
    k->ik_max_iterations = 32;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void kin_fk(const Kinematics* k, const Vec3* q, Vec3* p)
{
    float s0 = sin(q->x), c0 = cos(q->x);
    float s1 = sin(q->y), c1 = cos(q->y);
    float s12 = sin(q->y + q->z), c12 = cos(q->y + q->z);

    float r = k->l1 * c1 + k->l2 * c12;
    p->x = r * c0;
    p->y = r * s0;
    p->z = k->l1 * s1 + k->l2 * s12;
}

int kin_ik(const Kinematics* k, const Vec3* p_des, const Vec3* q_ref, Vec3* q_out)
{
    float x = p_des->x, y = p_des->y, z = p_des->z;
    float l1 = k->l1, l2 = k->l2;

    if (l1 <= 0.0f || l2 <= 0.0f) return KIN_ERR_UNREACHABLE;

    float r = sqrt(x * x + y * y);
    float d2 = r * r + z * z;

    float cos_q3 = (d2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    cos_q3 = clampf(cos_q3, -1.0f, 1.0f);

    float q3a =  acos(cos_q3);
    float q3b = -acos(cos_q3);

    float dq_a = (q_ref != NULL) ? fabs(q3a - q_ref->z) : 0.0f;
    float dq_b = (q_ref != NULL) ? fabs(q3b - q_ref->z) : 0.0f;
    float q3 = (dq_a <= dq_b) ? q3a : q3b;

    float q2 = atan2(z, r) - atan2(l2 * sin(q3), l1 + l2 * cos(q3));
    float q1 = atan2(y, x);

    if (r < 1e-9f && fabs(z) < 1e-9f) {
        q1 = (q_ref != NULL) ? q_ref->x : 0.0f;
    }

    q_out->x = q1;
    q_out->y = q2;
    q_out->z = q3;
    return KIN_OK;
}

void kin_trap_init(TrapProfile* tp, float start, float end, float max_vel, float max_acc)
{
    memset(tp, 0, sizeof(*tp));
    tp->max_vel = max_vel > 0.0f ? max_vel : 0.0f;
    tp->max_acc = max_acc > 0.0f ? max_acc : 0.0f;
    tp->p_start = start;
    tp->p_end   = end;

    float dist = fabs(end - start);
    if (tp->max_vel <= 0.0f || tp->max_acc <= 0.0f || dist <= 0.0f) {
        tp->duration = 0.0f;
        return;
    }

    float t_acc = tp->max_vel / tp->max_acc;
    float dist_acc = tp->max_acc * t_acc * t_acc;

    if (dist <= dist_acc) {
        tp->t1 = sqrt(dist / tp->max_acc);
        tp->t2 = tp->t1;
        tp->duration = 2.0f * tp->t1;
    } else {
        tp->t1 = t_acc;
        tp->t2 = (dist - dist_acc) / tp->max_vel + tp->t1;
        tp->duration = tp->t2 + tp->t1;
    }
}

void kin_trap_get(const TrapProfile* tp, float t, float* p, float* v, float* a)
{
    if (p) *p = tp->p_start;
    if (v) *v = 0.0f;
    if (a) *a = 0.0f;

    if (tp->duration <= 0.0f) { if (p) *p = tp->p_end; return; }

    float dir = (tp->p_end >= tp->p_start) ? 1.0f : -1.0f;
    float acc = tp->max_acc;
    float cruise = tp->max_vel;

    if (t <= 0.0f) { if (p) *p = tp->p_start; return; }
    if (t >= tp->duration) { if (p) *p = tp->p_end; return; }

    if (t < tp->t1) {
        if (a) *a = dir * acc;
        if (v) *v = dir * acc * t;
        if (p) *p = tp->p_start + dir * 0.5f * acc * t * t;
    } else if (t < tp->t2) {
        float dt = t - tp->t1;
        if (a) *a = 0.0f;
        if (v) *v = dir * cruise;
        if (p) *p = tp->p_start + dir * (0.5f * acc * tp->t1 * tp->t1 + cruise * dt);
    } else {
        float dt = t - tp->t2;
        if (a) *a = -dir * acc;
        if (v) *v = dir * (cruise - acc * dt);
        if (p) *p = tp->p_start + dir * (0.5f * acc * tp->t1 * tp->t1
                + cruise * (tp->t2 - tp->t1)
                + cruise * dt - 0.5f * acc * dt * dt);
    }
}
