#include "delay.h"

static uint32_t g_fac_us = 0;	// us延时倍乘数

void delay_init(void)
{
    // 配置 SysTick 定时器
		SysTick -> CTRL &= ~(1 << 0);
		SysTick -> CTRL |= (1 << 2);
		g_fac_us = 72;
		SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL |= (1 << 0);
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick -> LOAD;	// 获取重装载值
    
    ticks = nus * g_fac_us;             // 需要的时钟节拍数
    
    told = SysTick->VAL;                // 进去函数时的计数器当前值
    while (1)
    {
        tnow = SysTick->VAL;            // 当前计数器值
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;    // 递减计数器，相减得到走过节拍
            }
            else
            {
                tcnt += reload - tnow + told;	// 溢出处理
            }
            told = tnow;
            
            if (tcnt >= ticks)          // 延迟时间到，退出循环
            {
                break;
            }
		}
    }
}


void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)nms * 1000);
}
