#ifndef DS3231_H_
#define DS3231_H_

#include "stm32f1xx_hal.h"

#define DS3231_ADDRESS     0xD0

uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);
float DS3231_getTemperature(uint8_t tempMSB, uint8_t tempLSB);

#endif /* DS3231_H_ */
