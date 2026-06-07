#include "encoder.h"

void Encoder_Init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// RCC开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);								// PA6 PA7上拉输入
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructue;
	TIM_TimeBaseInitStructue.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructue.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructue.TIM_Period = 65536 - 1;			// ARR为最大值
	TIM_TimeBaseInitStructue.TIM_Prescaler = 1-1;					// 不预分频
	TIM_TimeBaseInitStructue.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructue);		// 配置时基单元
	
	TIM_ICInitTypeDef TIM_InitStructure;
	TIM_ICStructInit(&TIM_InitStructure);
	TIM_InitStructure.TIM_Channel = TIM_Channel_1;
	TIM_InitStructure.TIM_ICFilter = 0xF;
	TIM_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	// 高低电平不反相
	TIM_ICInit(TIM3, &TIM_InitStructure);
	TIM_InitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_InitStructure);										// 配置定时器输入通道
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM3, ENABLE);
}

int16_t Encoder_Get_Diff(void) {
	int diff;
	diff = TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	return diff;
}
