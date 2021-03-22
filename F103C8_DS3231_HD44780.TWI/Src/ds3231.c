#include "ds3231.h"

uint8_t decToBcd(uint8_t val) {
	return ((val / 10 * 16) + (val % 10));
}

uint8_t bcdToDec(uint8_t val) {
	return ((val / 16 * 10) + (val % 16));
}

float DS3231_getTemperature(uint8_t tempMSB, uint8_t tempLSB) {
	float t = 0.0;
	tempLSB >>= 6;                  
         tempLSB &= 0x03;      
         t = ((float)tempLSB);  
         t *= 0.25;      
         t += tempMSB;          
         return t; 
}
