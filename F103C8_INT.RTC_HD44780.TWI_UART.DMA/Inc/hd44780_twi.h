#ifndef HD44780_TWI_H_
#define HD44780_TWI_H_

#include "stm32f1xx_hal.h"

#define e_set() HD44780_Data(portlcd|=0x04)
#define e_reset() HD44780_Data(portlcd&=~0x04)
#define rs_set() HD44780_Data(portlcd|=0x01)
#define rs_reset() HD44780_Data(portlcd&=~0x01)
#define setled() HD44780_Data(portlcd|=0x08)
#define setwrite() HD44780_Data(portlcd&=~0x02)

void HD44780_Init(void);
void HD44780_SendChar(char ch);
void HD44780_String(char* str);
void HD44780_SetPos(uint8_t x, uint8_t y);
void HD44780_Clear(void);

#endif /* HD44780_TWI_H_ */
