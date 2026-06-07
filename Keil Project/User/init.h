#ifndef __INIT_H
#define __INIT_H

#include "stm32f10x.h"

void USART1_Init(unsigned int baud);
void USART1_IRQHandler(void);

#endif
