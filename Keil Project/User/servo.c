#include "servo.h"

/* TIM1 clock = 72 MHz.
   PSC = 72-1   -> timer counts at 1 MHz (1 tick = 1 us)
   ARR = 20000-1-> period = 20000 us = 20 ms = 50 Hz
   Pulse (CCR) in microseconds:
     1500 us = center (0 deg)
     1000 us = -90 deg, 2000 us = +90 deg  (~11.11 us/deg) */

#define SERVO_CENTER_US   1500.0f
#define SERVO_US_PER_DEG  11.11f
#define SERVO_MIN_US      500.0f
#define SERVO_MAX_US      2500.0f

void Servo_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;     /* alternate-function push-pull */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;          /* PA8 = TIM1_CH1 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = 20000 - 1;   /* ARR -> 50 Hz */
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 72 - 1;      /* PSC -> 1 MHz */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = (uint16_t)SERVO_CENTER_US;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);   /* required for advanced timer TIM1 */
    TIM_Cmd(TIM1, ENABLE);
}

void Servo_SetAngleDeg(float deg)
{
    float us = SERVO_CENTER_US + deg * SERVO_US_PER_DEG;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    TIM_SetCompare1(TIM1, (uint16_t)us);
}

void Servo_SetAngleRad(float rad)
{
    Servo_SetAngleDeg(rad * 57.29578f);   /* 180/pi */
}
