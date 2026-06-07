/**
 * main_hil.c  -  Backstepping trajectory-tracking, Hardware-In-the-Loop demo
 *
 * What runs on the STM32F103:
 *   1) a reference trajectory generator (a circle),
 *   2) the ported backstepping controller (controller.c),
 *   3) an Ackermann/bicycle KINEMATIC MODEL integrated in real time,
 *   4) the commanded front-wheel angle fic -> drives the real SERVO,
 *   5) reference path vs tracked path drawn on the OLED,
 *   6) every step streamed over UART (115200) as integer CSV for MATLAB compare.
 *
 * The rotary encoder adjusts the target speed Vd live (turn it while running).
 *
 * there is no DC drive motor / wheel odometry here. The car body
 * is simulated on the MCU (HIL); the SERVO is the real steering actuator and
 * physically shows the angle the controller commands.
 */

#include "stm32f10x.h"
#include "delay.h"
#include "oled.h"
#include "encoder.h"
#include "init.h"          /* USART1_Init() */
#include "controller.h"
#include "servo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI            3.14159265f
#define DT            0.02f       /* control period = 20 ms (50 Hz) */
#define TRAJ_R        1.0f        /* reference circle radius (m)    */
#define WHEELBASE_L   0.254f      /* match controller.c        			*/

/* ---------- OLED world->screen mapping ---------- */
#define SCR_CX        64
#define SCR_CY        32
#define SCR_SCALE     22.0f       /* pixels per meter */

#define HIST_MAX      120
static uint8_t hist_x[HIST_MAX];
static uint8_t hist_y[HIST_MAX];
static int     hist_n = 0;

/* ---------- minimal UART1 TX (USART1 is configured by USART1_Init) ---------- */
static void uart_send_str(const char *s)
{
    while (*s) {
        USART_SendData(USART1, (uint8_t)*s++);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

static void world_to_screen(float wx, float wy, int *sx, int *sy)
{
    *sx = SCR_CX + (int)(wx * SCR_SCALE);
    *sy = SCR_CY - (int)(wy * SCR_SCALE);   /* screen y grows downward */
}

/* draw the reference circle (sampled) into the OLED buffer */
static void draw_reference(void)
{
    int i, sx, sy;
    for (i = 0; i < 60; i++) {
        float a = (2.0f * PI * i) / 60.0f;
        world_to_screen(TRAJ_R * cosf(a), TRAJ_R * sinf(a), &sx, &sy);
        if (sx >= 0 && sx < 128 && sy >= 0 && sy < 64)
            OLED_Pixel((unsigned char)sx, (unsigned char)sy, 1);
    }
}

static void draw_history(void)
{
    int i;
    for (i = 0; i < hist_n; i++)
        OLED_Pixel(hist_x[i], hist_y[i], 1);
}

int main(void)
{
    CarState  st;
    RefState  ref;
    CtrlOut   out;
    char      buf[96];
    float     t   = 0.0f;
    float     Vd  = 0.5f;          /* target linear speed (m/s) */
    int       frame = 0;

    delay_init();
    USART1_Init(115200);
    Servo_Init();
    Encoder_Init();
    OLED_IIC_Init();
    OLED_Init();

    /* initial state: off the path, to show convergence */
    st.x  = 0.7f;
    st.y  = -0.2f;
    st.th = PI / 2.0f;

    uart_send_str("t_ms,xd_mm,yd_mm,x_mm,y_mm,th_mdeg,v_mmps,fic_mdeg\r\n");

    while (1) {
        /* --- live tuning: rotary encoder changes target speed --- */
        int16_t d = Encoder_Get_Diff();
        if (d) {
            Vd += (float)d * 0.02f;
            if (Vd < 0.1f) Vd = 0.1f;
            if (Vd > 1.0f) Vd = 1.0f;
        }

        /* --- reference trajectory: circle of radius R at speed Vd --- */
        float wd  = Vd / TRAJ_R;
        float phi = wd * t;
        ref.xd  = TRAJ_R * cosf(phi);
        ref.yd  = TRAJ_R * sinf(phi);
        ref.thd = phi + PI / 2.0f;     /* CCW tangent direction */
        ref.Vd  = Vd;
        ref.wd  = wd;

        /* --- controller --- */
        Backstepping_Ctrl(&ref, &st, &out);

        /* --- drive the REAL servo with the front-wheel angle --- */
        Servo_SetAngleRad(out.fic);

        /* --- integrate the kinematic bicycle model --- */
        st.th += (out.v / WHEELBASE_L) * tanf(out.fic) * DT;
        st.th  = atan2f(sinf(st.th), cosf(st.th));   /* wrap */
        st.x  += out.v * cosf(st.th) * DT;
        st.y  += out.v * sinf(st.th) * DT;

        /* --- record tracked path for OLED --- */
        if ((frame % 3) == 0) {       /* downsample so the buffer covers a full lap */
            int sx, sy;
            world_to_screen(st.x, st.y, &sx, &sy);
            if (sx >= 0 && sx < 128 && sy >= 0 && sy < 64) {
                if (hist_n < HIST_MAX) {
                    hist_x[hist_n] = (uint8_t)sx;
                    hist_y[hist_n] = (uint8_t)sy;
                    hist_n++;
                } else {                       /* shift out oldest */
                    int k;
                    for (k = 1; k < HIST_MAX; k++) {
                        hist_x[k-1] = hist_x[k];
                        hist_y[k-1] = hist_y[k];
                    }
                    hist_x[HIST_MAX-1] = (uint8_t)sx;
                    hist_y[HIST_MAX-1] = (uint8_t)sy;
                }
            }
        }

        /* --- redraw OLED every few frames (IIC is slow) --- */
        if ((frame % 5) == 0) {
            OLED_Clear_BUF();
            draw_reference();
            draw_history();
            OLED_Refresh_BUF();
        }

        /* --- stream data to PC as integer CSV --- */
        sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                (int)(t * 1000.0f),
                (int)(ref.xd * 1000.0f), (int)(ref.yd * 1000.0f),
                (int)(st.x  * 1000.0f),  (int)(st.y  * 1000.0f),
                (int)(st.th * 57295.78f),               /* milli-degrees */
                (int)(out.v * 1000.0f),
                (int)(out.fic * 57295.78f));
        uart_send_str(buf);

        t += DT;
        frame++;
        delay_ms(20);                 /* ~50 Hz loop */
    }
}
