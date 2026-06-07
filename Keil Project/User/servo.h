#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* Servo on TIM1_CH1 -> PA8, 50Hz (20ms) standard hobby servo. */
void  Servo_Init(void);

/* Set servo to an angle in degrees, range about [-90, +90] (0 = center). */
void  Servo_SetAngleDeg(float deg);

/* Set servo directly from the controller's front-wheel angle (radians). */
void  Servo_SetAngleRad(float rad);

#endif
