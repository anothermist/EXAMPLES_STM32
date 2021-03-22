#include "hd44780.h"

uint8_t customChar_LT[8] = { 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
uint8_t customChar_UB[8] = { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t customChar_RT[8] = { 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
uint8_t customChar_LL[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x07 };
uint8_t customChar_LB[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F };
uint8_t customChar_LR[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1C };
uint8_t customChar_MB[8] = { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x1F };
uint8_t customChar_BL[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };

void HD44780_WriteData(uint8_t dt)
{
	if(((dt >> 3)&0x01)==1) {d7_set();} else {d7_reset();}
	if(((dt >> 2)&0x01)==1) {d6_set();} else {d6_reset();}
	if(((dt >> 1)&0x01)==1) {d5_set();} else {d5_reset();}
	if((dt&0x01)==1) {d4_set();} else {d4_reset();}
}

void HD44780_ResetPins(void)
{
	d4_reset();
	d5_reset();
	d6_reset();
	d7_reset();
}

void HD44780_Command(uint8_t dt)
{
	rs0;
	HD44780_WriteData(dt>>4);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_WriteData(dt);
	e1;
	HAL_Delay(1);
	e0;
}

void HD44780_Data(uint8_t dt)
{
	rs1;
	HD44780_WriteData(dt>>4);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_WriteData(dt);
	e1;
	HAL_Delay(1);
	e0;
}

void HD44780_SendChar(char ch)
{
	HD44780_Data((uint8_t) ch);
	HAL_Delay(1);
}

void HD44780_String(char* st)
{
	uint8_t i=0;
	while(st[i]!=0)
	{
		if (st[i] >= 0xD0)
		{
			i++;
			if (st[i-1] == 0xD0)
				HD44780_Data(st[i] + 0x30);
			if (st[i-1] == 0xD1)
				HD44780_Data(st[i] + 0x70);
		}
		else HD44780_Data(st[i]);
		HAL_Delay(1);
		i++;
	}
}

void HD44780_SetPos(uint8_t y, uint8_t x)
{
	switch(y)
	{
		case 0:
			HD44780_Command(x|0x80);
			HAL_Delay(1);
			break;
		case 1:
			HD44780_Command((0x40+x)|0x80);
			HAL_Delay(1);
			break;
		case 2:
			HD44780_Command((0x14+x)|0x80);
			HAL_Delay(1);
			break;
		case 3:
			HD44780_Command((0x54+x)|0x80);
			HAL_Delay(1);
			break;
	}
}

void HD44780_Clear(void)
{
	HD44780_Command(0x01);
	HAL_Delay(2);
}

void HD44780_Init(void)
{
	HAL_Delay(40);
	rs0;
	HD44780_WriteData(3);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_WriteData(3);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_WriteData(3);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_WriteData(2);
	e1;
	HAL_Delay(1);
	e0;
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_Command(0x2A); //4-bit mode, 2 lines, font 5x8
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_Command(0x0C); //display on (D=1), cursors on
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_Command(0x01); //delete crap
	HD44780_ResetPins();
	HAL_Delay(2);
	HD44780_Command(0x06); //left write
	HD44780_ResetPins();
	HAL_Delay(1);
	HD44780_Command(0x02); //back cursor to zero
	HD44780_ResetPins();
	HAL_Delay(2);
	
	HD44780_CreateChar(0, customChar_LT);
	HD44780_CreateChar(1, customChar_UB);
	HD44780_CreateChar(2, customChar_RT);
	HD44780_CreateChar(3, customChar_LL);
	HD44780_CreateChar(4, customChar_LB);
	HD44780_CreateChar(5, customChar_LR);
	HD44780_CreateChar(6, customChar_MB);
	HD44780_CreateChar(7, customChar_BL);
}

void HD44780_CreateChar(uint8_t location, uint8_t *data) {
	uint8_t i;
	location &= 0x07;
	HD44780_Command(0x40 | (location << 3));
	for (i = 0; i < 8; i++) {
		HD44780_Data(data[i]);
	}
}

void drawBigDigits(uint8_t digit, uint8_t place) {

	switch (digit) {
		case 0:
		HD44780_SetPos(0, place);
		HD44780_Data(0);
		HD44780_Data(1);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_Data(3);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
		case 1:
		HD44780_SetPos(0, place);
		HD44780_Data(1);
		HD44780_Data(2);
		HD44780_String(" ");
		HD44780_SetPos(1, place);
		HD44780_Data(4);
		HD44780_Data(7);
		HD44780_Data(4);
		break;
		case 2:
		HD44780_SetPos(0, place);
		HD44780_Data(6);
		HD44780_Data(6);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_Data(3);
		HD44780_Data(4);
		HD44780_Data(4);
		break;
		case 3:
		HD44780_SetPos(0, place);
		HD44780_Data(6);
		HD44780_Data(6);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_Data(4);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
		case 4:
		HD44780_SetPos(0, place);
		HD44780_Data(3);
		HD44780_Data(4);
		HD44780_Data(7);
		HD44780_SetPos(1, place);
		HD44780_String(" ");
		HD44780_String(" ");
		HD44780_Data(7);
		break;
		case 5:
		HD44780_SetPos(0, place);
		HD44780_Data(3);
		HD44780_Data(6);
		HD44780_Data(6);
		HD44780_SetPos(1, place);
		HD44780_Data(4);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
		case 6:
		HD44780_SetPos(0, place);
		HD44780_Data(0);
		HD44780_Data(6);
		HD44780_Data(6);
		HD44780_SetPos(1, place);
		HD44780_Data(3);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
		case 7:
		HD44780_SetPos(0, place);
		HD44780_Data(1);
		HD44780_Data(1);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_String(" ");
		HD44780_String(" ");
		HD44780_Data(7);
		break;
		case 8:
		HD44780_SetPos(0, place);
		HD44780_Data(0);
		HD44780_Data(6);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_Data(3);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
		case 9:
		HD44780_SetPos(0, place);
		HD44780_Data(0);
		HD44780_Data(6);
		HD44780_Data(2);
		HD44780_SetPos(1, place);
		HD44780_Data(4);
		HD44780_Data(4);
		HD44780_Data(5);
		break;
	}
}

void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) {
	HD44780_SetPos(x, y);
	HD44780_Data(location);
}
