#ifndef _BME280_H_
#define _BME280_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define BME280_I2C 			hi2c1
#define BME280_ADDR_G		0x76							 //Address with SDO connected to GND
#define BME280_ADDR_V		0x77							 //Address with SDO connected to VDDIO
#define BME280_ADDR		(BME280_ADDR_G) << 1 //Left shift to match the addressing format

#define BME280_I2C_TIMEOUT 	100

void BME280_Init(void);
float BME280_getTemperature(void);
float BME280_getPressure(void);
float BME280_getHumidity(void);

#ifdef __cplusplus
}
#endif

#endif /* _BME280_H_ */
