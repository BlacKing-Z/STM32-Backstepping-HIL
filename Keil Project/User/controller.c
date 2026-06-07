#include "controller.h"
#include <math.h>

/* ---- Physical / tuning parameters ---- */
#define WHEELBASE_L   0.254f              /* car wheelbase (m)            */
#define V_LIMIT       5.0f                /* linear velocity saturation   */
#define OMEGA_LIMIT   5.0f                /* angular velocity saturation  */
#define STEER_LIMIT   (3.14159265f/4.0f)  /* max front-wheel angle ~45deg */

/* Controller gains */
static const float k1 = 5.0f;
static const float k2 = 6.0f;
static const float k3 = 3.0f;
static const float k4 = 20.0f;

/* wrap angle to [-pi, pi] */
static float wrap_pi(float a)
{
    return atan2f(sinf(a), cosf(a));
}

static float sat(float val, float lim)
{
    if (val >  lim) return  lim;
    if (val < -lim) return -lim;
    return val;
}

void Backstepping_Ctrl(const RefState *ref, const CarState *st, CtrlOut *out)
{
    /* --- global pose error --- */
    float xe  = ref->xd - st->x;
    float ye  = ref->yd - st->y;
    float the = wrap_pi(ref->thd - st->th);

    /* --- transform error into robot frame --- */
    float e1 =  cosf(st->th) * xe + sinf(st->th) * ye;
    float e2 = -sinf(st->th) * xe + cosf(st->th) * ye;
    float etheta = the;

    float vr = ref->Vd;
    float wr = ref->wd;

    /* --- backstepping control law --- */
    float c_half = cosf(etheta * 0.5f);
    float s_half = sinf(etheta * 0.5f);

    float omega = 2.0f * k3 * e2 * vr * c_half + k4 * s_half + wr;

    /* reference assumed constant -> derivatives are zero */
    float vrdot = 0.0f;
    float wrdot = 0.0f;

    float omega_dot = 2.0f * k3 * ((-omega * e1 + vr * sinf(etheta)) * vr + e2 * vrdot) * c_half
                    - k3 * e2 * vr * s_half * (wr - omega)
                    + 0.5f * k4 * c_half * (wr - omega)
                    + wrdot;

    float denom  = sqrtf(1.0f + omega * omega);
    float denom3 = denom * denom * denom;            /* (1+omega^2)^1.5 */

    float v = vr * cosf(etheta)
            + k1 * omega * e1 * (omega / denom)
            - k1 * vr * sinf(etheta) * (omega / denom)
            - k1 * omega_dot * e2 / denom3
            + k2 * (e1 - k1 * e2 * (omega / denom));

    /* --- output saturation --- */
    v     = sat(v, V_LIMIT);
    omega = sat(omega, OMEGA_LIMIT);

    /* --- invert bicycle model: omega = (v/L)*tan(fic) -> fic --- */
    float fic;
    if (fabsf(v) < 0.001f) {
        fic = 0.0f;                       /* avoid singularity */
    } else {
        fic = atanf(omega * WHEELBASE_L / v);
    }
    fic = sat(fic, STEER_LIMIT);          /* limit physical steering angle */

    out->v     = v;
    out->omega = omega;
    out->fic   = fic;
}
