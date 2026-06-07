#ifndef __CONTROLLER_H
#define __CONTROLLER_H

/* Reference (desired) state from the trajectory generator */
typedef struct {
    float xd;     /* desired x position (m)            */
    float yd;     /* desired y position (m)            */
    float thd;    /* desired heading (rad)             */
    float Vd;     /* desired linear velocity (m/s)     */
    float wd;     /* desired angular velocity (rad/s)  */
} RefState;

/* Current car state (from the kinematic model on the MCU) */
typedef struct {
    float x;      /* current x position (m) */
    float y;      /* current y position (m) */
    float th;     /* current heading (rad)  */
} CarState;

/* Controller output */
typedef struct {
    float v;      /* commanded linear velocity (m/s)        */
    float omega;  /* commanded body angular velocity (rad/s) */
    float fic;    /* commanded FRONT WHEEL steering angle (rad) -> servo */
} CtrlOut;

/* Backstepping trajectory-tracking controller (ported from MATLAB) */
void Backstepping_Ctrl(const RefState *ref, const CarState *st, CtrlOut *out);

#endif
