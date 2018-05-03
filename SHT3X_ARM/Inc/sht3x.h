#ifndef _SHT3X_H_
#define _SHT3X_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define SHT3X_I2C 			hi2c1
#define SHT3X_ADDR_0		0x44
#define SHT3X_ADDR_1		0x45
#define SHT3X_ADDR		(SHT3X_ADDR_0)<<1

#define SHT3X_I2C_TIMEOUT 	100

#define HT_CMD_MSB 0x24
#define HT_CMD_LSB 0x00

void SHT3X_Update(void);
float SHT3X_getTemperature(void);
float SHT3X_getHumidity(void);

#ifdef __cplusplus
}
#endif

#endif /* _SHT3X_H_ */
