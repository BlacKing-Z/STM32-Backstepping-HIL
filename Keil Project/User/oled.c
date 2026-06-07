#include "oled.h"
#include "oledfont.h"
#include <math.h>

unsigned char xx, yy;	// 记录当前坐标
unsigned char SCR_BUF[128][8];

void OLED_IIC_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	// PC10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	// PB15
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_SCL_Set();
	OLED_SDA_Set();
}

void IIC_Start() {
	OLED_SCL_Set();
	OLED_SDA_Set();
	delay_us(2);
	OLED_SDA_Clr();
	delay_us(2);
	OLED_SCL_Clr();
	delay_us(2);
}

void IIC_Stop() {
	OLED_SCL_Clr();
	OLED_SDA_Clr();
	delay_us(2);
	OLED_SCL_Set();
	OLED_SDA_Set();
	delay_us(2);
}

void Write_IIC_Byte(unsigned char IIC_Byte) {
	for (unsigned char i = 0; i < 8; i ++) {
		if (IIC_Byte & 0x80) OLED_SDA_Set();
		else OLED_SDA_Clr();
		
		delay_us(2);
		OLED_SCL_Set();
		delay_us(2);
		OLED_SCL_Clr();
		delay_us(2);
		
		IIC_Byte <<= 1;
	}
	
	OLED_SDA_Set();
	delay_us(2);
	OLED_SCL_Set();
	delay_us(2);
	OLED_SCL_Clr();
}

void OLED_WrDat(unsigned char dat) {
	IIC_Start();
	Write_IIC_Byte(0x78);	// Slave address, SA0 = 0
	Write_IIC_Byte(0x40);	// write data
	Write_IIC_Byte(dat);
	IIC_Stop();
}

void OLED_WrCmd(unsigned char cmd) {
	IIC_Start();
	Write_IIC_Byte(0x78);
	Write_IIC_Byte(0x00);	// write command
	Write_IIC_Byte(cmd);
	IIC_Stop();
}

// 复位/清屏
void OLED_CLS() {
	for(unsigned char i = 0; i < 8; i++) {
		OLED_WrCmd(0xb0 + i);
		OLED_WrCmd(0x00);
		OLED_WrCmd(0x10);
		for(unsigned char j = 0; j < 128; j++) OLED_WrDat(0);
	}
}

void OLED_Init() {
	delay_ms(200);	// 上电后延时稳定

	OLED_WrCmd(0xAE);

	OLED_WrCmd(0xD5);
	OLED_WrCmd(0x80);

	OLED_WrCmd(0xA8);
	OLED_WrCmd(0x3F);

	OLED_WrCmd(0xD3);
	OLED_WrCmd(0x00);

	OLED_WrCmd(0x40);
	OLED_WrCmd(0x8D);
	OLED_WrCmd(0x14);

	OLED_WrCmd(0x20);
	OLED_WrCmd(0x02);

	OLED_WrCmd(0xA1);
	OLED_WrCmd(0xC8);

	OLED_WrCmd(0xDA);
	OLED_WrCmd(0x12);

	OLED_WrCmd(0x81);
	OLED_WrCmd(0xEF);

	OLED_WrCmd(0xD9); 
	OLED_WrCmd(0xF1);

	OLED_WrCmd(0xDB);
	OLED_WrCmd(0x30);

	OLED_WrCmd(0xA4);
	OLED_WrCmd(0xA6);

	OLED_CLS();

	OLED_WrCmd(0xAF);
}

// 设置显示坐标
void OLED_Set_Pos(unsigned char x, unsigned char y) {
	OLED_WrCmd(0xb0 + y);
	OLED_WrCmd(((x & 0xf0) >> 4) | 0x10);
	OLED_WrCmd((x & 0x0f));
}

// 开启OLED显示
void OLED_Display_On() {
	OLED_WrCmd(0X8D);
	OLED_WrCmd(0X14);
	OLED_WrCmd(0XAF);
}

// 关闭OLED显示
void OLED_Display_Off() {
	OLED_WrCmd(0X8D);
	OLED_WrCmd(0X10);
	OLED_WrCmd(0XAE);
}

// 更新显存到OLED
void OLED_Refresh_BUF() {
	unsigned char i, j;
	for (i = 0; i < 8; i ++) {
		OLED_Set_Pos(0, i);	// 指针自动累加
		for (j = 0; j < 128; j ++) OLED_WrDat(SCR_BUF[j][i]);
	}
	/*
	for (i = 0; i < 128; i ++) {
		for (j = 0; j < 8; j ++) {
			OLED_Set_Pos(i, j);
			OLED_WrDat(SCR_BUF[i][j]);
		}
	}
	*/
}

void OLED_Clear_BUF() {
	unsigned char i, j;
	for (i = 0; i < 8; i ++) {
		for (j = 0; j < 128; j ++) SCR_BUF[j][i] = 0x00;
	}
}

// x: 0~127; y: 0~63; t: 1 填充, 0 清空
void OLED_Pixel(unsigned char x, unsigned char y, unsigned char t) {
	unsigned char s, num, temp = 0;
	if (x > 127 || y >63) return;
	
	s = y / 8;		// 定位纵坐标第几屏
	num = y % 8;	// 定位小竖棍第几位
	
	temp = 0x01 << num;	// 写入/清除数据
	if (t) SCR_BUF[x][s] |= temp;
	else SCR_BUF[x][s] &= ~temp;
	
	xx = x;
	yy = y;
}

void OLED_Line(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1) {
	int delta_x, delta_y, xerr = 0, yerr = 0, distance;
	int dirx, diry, now_x, now_y;
	
	delta_x = x1 - x0;	// 计算坐标增量
	delta_y = y1 - y0;
	now_x = x0;
	now_y = y0;
	
	if (delta_x > 0) dirx = 1;	// 设置单步方向
	else if (delta_x == 0) dirx = 0;
	else {
		dirx = -1;
		delta_x = -delta_x;
	}
	if (delta_y > 0) diry = 1;	// 设置单步方向
	else if (delta_y == 0) diry = 0;
	else {
		diry = -1;
		delta_y = -delta_y;
	}
	
	distance = (delta_x > delta_y)? delta_x: delta_y;
	
	for (int i = 0; i <= distance; i ++) {
		OLED_Pixel(now_x, now_y, 1);
		xerr += delta_x;
		yerr += delta_y;
		
		if (xerr >= distance) {
			xerr -= distance;
			now_x += dirx;
		}
		if (yerr >= distance) {
			yerr -= distance;
			now_y += diry;
		}
	}
	
	xx = x1;
	yy = y1;
}

void OLED_Lineto(unsigned char x, unsigned char y) {
	OLED_Line(xx, yy, x, y);
	xx = x;
	yy = y;
}

void OLED_P6x8Str(unsigned char x, unsigned char y, char ch[]) {
	unsigned char c = 0, i = 0, j = 0;
	while (ch[j] != '\0') {          // 遇到结束符 '\0' 停止
		c = ch[j] - 32;              // 计算 ASCII 字符偏移量
		if (x > 126) { x = 0; y++; } // 自动换行

		OLED_Set_Pos(x, y);
		for (i = 0; i < 6; i++) {
			OLED_WrDat(F6x8[c][i]);  // 写入 6 列数字
		}
		x += 6;
		j++;
	}
}

void OLED_P8x16Str(unsigned char x, unsigned char y, char ch[]) {
	unsigned char c = 0, i = 0, j = 0;
	while (ch[j] != '\0') {
		c = ch[j] - 32;
		if (x > 120) { x = 0; y += 2; }
        
		OLED_Set_Pos(x, y);             // 写上半部分
		for (i = 0; i < 8; i++) {
			OLED_WrDat(F8X16[c * 16 + i]); 
		}

		OLED_Set_Pos(x, y + 1);			// 写下半部分
		for (i = 0; i < 8; i++) {
			OLED_WrDat(F8X16[c * 16 + i + 8]);
		}

		x += 8;
		j++;
	}
}

/*
void OLED_P16x16Ch(unsigned char x, unsigned char y, char N) {
	unsigned char wm = 0;

	OLED_Set_Pos(x, y);
	for (wm = 0; wm < 16; wm++) {
		OLED_WrDat(Hzk[N][wm]);
	}

	OLED_Set_Pos(x, y + 1);
	for (wm = 0; wm < 16; wm++) {
		OLED_WrDat(Hzk[N][wm + 16]);
	}
} */

void OLED_Test_Output(void) {
	OLED_Clear_BUF();
	OLED_Refresh_BUF();
	OLED_P8x16Str(0, 0, "Hello STM32!");
	OLED_P6x8Str(0, 4, "Temp: 25.5 C");
}
