#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"
#include "delay.h"

// SCL -> PC10
#define OLED_SCL_Clr() GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define OLED_SCL_Set() GPIO_SetBits(GPIOC, GPIO_Pin_10)

// SDA -> PB15
#define OLED_SDA_Clr() GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define OLED_SDA_Set() GPIO_SetBits(GPIOB, GPIO_Pin_15)

void OLED_IIC_Init(void);
void OLED_WrDat(unsigned char dat);
void OLED_WrCmd(unsigned char cmd);

void OLED_CLS(void);
void OLED_Init(void);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Display_On(void);
void OLED_Display_Off(void);

void OLED_Refresh_BUF(void);
void OLED_Clear_BUF(void);
void OLED_Pixel(unsigned char x, unsigned char y, unsigned char t);
void OLED_Line(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1);
void OLED_Lineto(unsigned char x, unsigned char y);

void OLED_P6x8Str(unsigned char x, unsigned char y, char ch[]);
void OLED_P8x16Str(unsigned char x, unsigned char y, char ch[]);
void OLED_Test_Output(void);

#endif
