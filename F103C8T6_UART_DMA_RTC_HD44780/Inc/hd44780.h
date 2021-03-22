#ifndef HD44780_H_
#define HD44780_H_

#include "stm32f1xx_hal.h"
 
#define d4_set() HAL_GPIO_WritePin(HD44780_D4_GPIO_Port, HD44780_D4_Pin, GPIO_PIN_SET)
#define d5_set() HAL_GPIO_WritePin(HD44780_D5_GPIO_Port, HD44780_D5_Pin, GPIO_PIN_SET)
#define d6_set() HAL_GPIO_WritePin(HD44780_D6_GPIO_Port, HD44780_D6_Pin, GPIO_PIN_SET)
#define d7_set() HAL_GPIO_WritePin(HD44780_D7_GPIO_Port, HD44780_D7_Pin, GPIO_PIN_SET)
#define d4_reset() HAL_GPIO_WritePin(HD44780_D4_GPIO_Port, HD44780_D4_Pin, GPIO_PIN_RESET)
#define d5_reset() HAL_GPIO_WritePin(HD44780_D5_GPIO_Port, HD44780_D5_Pin, GPIO_PIN_RESET)
#define d6_reset() HAL_GPIO_WritePin(HD44780_D6_GPIO_Port, HD44780_D6_Pin, GPIO_PIN_RESET)
#define d7_reset() HAL_GPIO_WritePin(HD44780_D7_GPIO_Port, HD44780_D7_Pin, GPIO_PIN_RESET)

#define e1		HAL_GPIO_WritePin(HD44780_EN_GPIO_Port, HD44780_EN_Pin, GPIO_PIN_SET) 
#define e0    HAL_GPIO_WritePin(HD44780_EN_GPIO_Port, HD44780_EN_Pin, GPIO_PIN_RESET)
#define rs1   HAL_GPIO_WritePin(HD44780_RS_GPIO_Port, HD44780_RS_Pin, GPIO_PIN_SET)
#define rs0   HAL_GPIO_WritePin(HD44780_RS_GPIO_Port, HD44780_RS_Pin, GPIO_PIN_RESET)

void HD44780_Init(void);
void HD44780_SendChar(char ch);
void HD44780_String(char* str);
void HD44780_SetPos(uint8_t x, uint8_t y);
void HD44780_Clear(void);
void HD44780_CreateChar(uint8_t location, uint8_t *data);
void drawBigDigits(uint8_t digit, uint8_t place);
void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);

#endif /* HD44780_H_ */
